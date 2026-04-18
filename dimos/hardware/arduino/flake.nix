{
  description = "DimOS Arduino support — bridge binary + Arduino toolchain";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    dimos-lcm = {
      url = "github:dimensionalOS/dimos-lcm/main";
      flake = false;
    };
    # Patched LCM that builds cleanly on macOS (pkg-config + fdatasync
    # fixes).  On Linux this is identical to upstream pkgs.lcm.
    lcm-extended.url = "github:jeff-hykin/lcm_extended";
  };

  outputs = { self, nixpkgs, flake-utils, dimos-lcm, lcm-extended }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};

        # Single-output LCM built on top of lcm_extended.
        # We still collapse outputs to a single `out` so downstream
        # CMakeLists.txt doesn't have to juggle `lcm` vs `lcm-dev` paths.
        lcmFull = (lcm-extended.packages.${system}.lcm).overrideAttrs (old: {
          outputs = [ "out" ];
          postInstall = "";
        });

        # Bundle of external tools ArduinoModule shells out to at
        # runtime: the unwrapped arduino-cli Go binary (not the bwrap-
        # wrapped `pkgs.arduino-cli`), avrdude for physical uploads,
        # and qemu for virtual-Arduino mode (`pkgs.qemu` ships all
        # system targets including qemu-system-avr).  Exposed as a
        # single flake output so `ArduinoModule` can resolve all three
        # via one ``nix build .#dimos_arduino_tools`` — the alternative
        # is requiring the user to enter ``nix develop`` before
        # running their blueprint, which defeats the point of dimos
        # being a normal Python library you can import and run.
        # On Linux, arduino-cli ships a bwrap-wrapped FHS environment
        # that fails on sandboxed hosts.  The unwrapped Go binary
        # (pureGoPkg) works everywhere.  On macOS, pureGoPkg doesn't
        # exist — the plain package is already unwrapped.
        arduino-cli-unwrapped =
          pkgs.arduino-cli.pureGoPkg or pkgs.arduino-cli;

        dimos_arduino_tools = pkgs.symlinkJoin {
          name = "dimos-arduino-tools";
          paths = [
            arduino-cli-unwrapped
            pkgs.avrdude
            pkgs.qemu
          ];
        };

        # The generic serial↔LCM bridge
        arduino_bridge = pkgs.stdenv.mkDerivation {
          pname = "arduino_bridge";
          version = "0.1.0";
          src = ./.;

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [ lcmFull pkgs.glib ];

          cmakeFlags = [
            "-DDIMOS_LCM_DIR=${dimos-lcm}"
          ];

          # CMakeLists.txt is in cpp/ subdirectory
          cmakeDir = "../cpp";

          installPhase = ''
            mkdir -p $out/bin
            cp arduino_bridge $out/bin/
          '';
        };

      in {
        packages = {
          inherit arduino_bridge dimos_arduino_tools;
          default = arduino_bridge;
        };

        devShells.default = pkgs.mkShell {
          packages = [
            arduino_bridge
            # Reuse the same unwrapped binary defined above.
            arduino-cli-unwrapped
            pkgs.avrdude
            pkgs.picocom
            # qemu-system-avr for virtual-Arduino mode.  `pkgs.qemu` builds
            # all system targets including avr and works on darwin +
            # linux; it's ~400MB but cached via the public binary cache
            # on common platforms so first-time install is the only cost.
            pkgs.qemu
          ];
        };
      });
}
