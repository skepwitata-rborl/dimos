# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
DimOS Dockerfile Conversion Tool

Converts any Dockerfile into a DimOS module container using detection rules
to choose between Mode A -> rebase to DimOS base image or Mode B -> append footer.
"""

import re
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Optional
import typer

# Supported base images
BASE_IMAGES = {
    "cpu": "ghcr.io/dimensionalos/dimos-base:py310",
    "cuda12.8": "ghcr.io/dimensionalos/dimos-base:cuda12.8-py310",
    "cuda12.4": "ghcr.io/dimensionalos/dimos-base:cuda12.4-py310",
    "cuda11.8": "ghcr.io/dimensionalos/dimos-base:cuda11.8-py310",
}

# Footer for Mode B conversion (uses build context, not remote image)
DIMOS_FOOTER = """
# ==== DIMOS MODULE CONVERSION ====
# Copy DimOS source from build context
COPY dimos /dimos/source/dimos/
COPY pyproject.toml /dimos/source/
COPY docker/python/module-install.sh /tmp/module-install.sh

# Install DimOS and create entrypoint
RUN bash /tmp/module-install.sh /dimos/source && rm /tmp/module-install.sh

ENTRYPOINT ["/dimos/entrypoint.sh"]
"""

BUILDKIT_SYNTAX = "# syntax=docker/dockerfile:1.7\n"

# Map PyTorch CUDA wheel versions (cuXXX) to CUDA version format (XX.Y)
CUDA_VERSION_MAP = {"128": "12.8", "124": "12.4", "118": "11.8", "121": "12.1"}

class ConversionMode(str, Enum):
    AUTO = "auto"
    REBASE = "rebase"  # Mode A
    FOOTER = "footer"  # Mode B

class BaseType(str, Enum):
    CPU = "cpu"
    CUDA12_8 = "cuda12.8"
    CUDA12_4 = "cuda12.4"
    CUDA11_8 = "cuda11.8"

@dataclass
class DockerfileAnalysis:
    """Results of analyzing a Dockerfile."""
    is_gpu: bool
    cuda_version: Optional[str]
    has_cuda_extensions: bool
    original_from: str
    can_rebase: bool
    reason: str
    is_multi_stage: bool = False

def analyze_dockerfile(content: str) -> DockerfileAnalysis:
    """Analyze a Dockerfile to determine GPU/CPU and CUDA version."""
    lines = content.split("\n")

    # Find all FROM lines (for multi-stage detection)
    from_lines = [line.strip() for line in lines if line.strip().upper().startswith("FROM ")]
    from_line = from_lines[0] if from_lines else ""
    is_multi_stage = len(from_lines) > 1

    # Detection signals
    is_gpu = False
    cuda_version = None
    has_cuda_extensions = False
    can_rebase = True
    reason = "Standard Python Dockerfile"

    # Check for nvidia/cuda base (can rebase since DimOS base is also nvidia/cuda)
    if "nvidia/cuda:" in content.lower():
        is_gpu = True
        # Extract CUDA version from FROM nvidia/cuda:XX.Y...
        cuda_match = re.search(r"nvidia/cuda:(\d+\.\d+)", content, re.IGNORECASE)
        if cuda_match:
            cuda_version = cuda_match.group(1)
        reason = f"Uses nvidia/cuda base (CUDA {cuda_version})"

    # Check for CUDA toolkit usage (extension building) - case insensitive
    content_lower = content.lower()
    cuda_signals = ["cuda_home", "torch_cuda_arch_list", "force_cuda", "nvcc", "--no-build-isolation"]
    for signal in cuda_signals:
        if signal in content_lower:
            has_cuda_extensions = True
            is_gpu = True
            break

    # Check for PyTorch CUDA wheels
    torch_cuda_patterns = [
        r"--index-url\s+\S*cu(\d+)",
        r"torch.*\+cu(\d+)",
        r"download\.pytorch\.org/whl/cu(\d+)",
    ]
    for pattern in torch_cuda_patterns:
        match = re.search(pattern, content)
        if match:
            is_gpu = True
            cu_ver = match.group(1)
            cuda_version = cuda_version or CUDA_VERSION_MAP.get(cu_ver)
            break

    # Check for ROS base (can't easily rebase)
    if "ros:" in content.lower() or "osrf/ros" in content.lower():
        can_rebase = False
        reason = "Uses ROS base image - footer conversion required"

    # Check for other vendor bases
    vendor_patterns = ["mcr.microsoft.com", "gcr.io", "nvcr.io"]
    for vendor in vendor_patterns:
        if vendor in content.lower():
            can_rebase = False
            reason = f"Uses vendor base ({vendor}) - footer conversion required"
            break

    return DockerfileAnalysis(
        is_gpu=is_gpu,
        cuda_version=cuda_version,
        has_cuda_extensions=has_cuda_extensions,
        original_from=from_line,
        can_rebase=can_rebase,
        reason=reason,
        is_multi_stage=is_multi_stage,
    )

# Supported CUDA version prefixes mapped to base image keys
CUDA_BASE_MAP = {
    "12.8": "cuda12.8", "12.9": "cuda12.8",
    "12.4": "cuda12.4", "12.5": "cuda12.4", "12.6": "cuda12.4",
    "11.": "cuda11.8",
}

def get_base_image(analysis: DockerfileAnalysis, force_base: Optional[str] = None) -> tuple[str, Optional[str]]:
    """Determine which DimOS base image to use. Returns (image, warning)."""
    if force_base:
        return BASE_IMAGES.get(force_base, BASE_IMAGES["cpu"]), None

    if not analysis.is_gpu:
        return BASE_IMAGES["cpu"], None

    cuda_ver = analysis.cuda_version
    if cuda_ver:
        # Check for exact or prefix match
        for prefix, base_key in CUDA_BASE_MAP.items():
            if cuda_ver.startswith(prefix):
                return BASE_IMAGES[base_key], None

        # No exact match - warn about version mapping
        warning = f"Detected CUDA {cuda_ver} but only bases [12.8, 12.4, 11.8] exist; using 12.8"
        return BASE_IMAGES["cuda12.8"], warning

    # Default to latest CUDA (no specific version detected)
    return BASE_IMAGES["cuda12.8"], None


def convert_rebase(content: str, base_image: str, strip_base_setup: bool = False) -> str:
    """Mode A: Rebase the Dockerfile to use DimOS base image."""
    lines = content.split("\n")
    result = []

    # Find existing syntax line or use default, always place at very top
    existing_syntax = next(
        (line.strip() for line in lines if line.strip().startswith("# syntax=")),
        None
    )
    result.append(existing_syntax or BUILDKIT_SYNTAX.strip())

    # Track if we've replaced FROM
    from_replaced = False

    # Patterns to skip (only if strip_base_setup is enabled)
    skip_patterns = [
        r"^ENV\s+DEBIAN_FRONTEND",
        r"^ENV\s+PYTHONDONTWRITEBYTECODE",
        r"^ENV\s+PYTHONUNBUFFERED",
        r"^ENV\s+CONDA_DIR",
        r"^RUN.*miniconda",
        r"^RUN\s+conda\s+init",
        r"^RUN\s+conda\s+create\s+-n\s+app",
        r"ENTRYPOINT.*entrypoint",
    ]

    for line in lines:
        stripped = line.strip()

        # Skip existing syntax line (we already added it at top)
        if stripped.startswith("# syntax="):
            continue

        # Replace first FROM
        if not from_replaced and stripped.upper().startswith("FROM "):
            result.append(f"FROM {base_image}")
            from_replaced = True
            continue

        # Optionally skip lines that are already in base
        if strip_base_setup:
            should_skip = False
            for pattern in skip_patterns:
                if re.match(pattern, stripped, re.IGNORECASE):
                    should_skip = True
                    break
            if should_skip:
                continue

        result.append(line)

    return "\n".join(result)


def convert_footer(content: str) -> str:
    """Mode B: Append DimOS footer to the Dockerfile."""
    result = content.rstrip()

    # Add BuildKit syntax if missing
    if not result.strip().startswith("# syntax="):
        result = BUILDKIT_SYNTAX + result

    # Check if already has DimOS conversion
    if "module-install.sh" in result or "dimos/entrypoint.sh" in result:
        return result  # Already converted

    # Ensure exactly one newline before footer
    if not result.endswith("\n"):
        result += "\n"
    result += DIMOS_FOOTER.lstrip("\n")

    return result


def convert_dockerfile(
    input_path: Path,
    output_path: Optional[Path] = None,
    mode: ConversionMode = ConversionMode.AUTO,
    force_base: Optional[str] = None,
    dry_run: bool = False,
    strip_base_setup: bool = False,
) -> tuple[str, DockerfileAnalysis, list[str]]:
    """Convert a Dockerfile to a DimOS module Dockerfile."""
    content = input_path.read_text()
    analysis = analyze_dockerfile(content)
    warnings: list[str] = []

    # Determine conversion mode
    if mode == ConversionMode.AUTO:
        # Multi-stage builds are safer with footer mode (final stage may not be rebased)
        use_rebase = analysis.can_rebase and not analysis.is_multi_stage
    elif mode == ConversionMode.REBASE:
        use_rebase = True
    else:
        use_rebase = False

    # Perform conversion
    if use_rebase:
        base_image, cuda_warning = get_base_image(analysis, force_base)
        if cuda_warning:
            warnings.append(cuda_warning)
        if analysis.is_multi_stage:
            warnings.append("Multi-stage Dockerfile: only first stage rebased. Consider --mode footer instead")
        result = convert_rebase(content, base_image, strip_base_setup)
        mode_used = "rebase"
    else:
        result = convert_footer(content)
        mode_used = "footer"

    # Output
    if not dry_run:
        out = output_path or input_path
        out.write_text(result)

    message = f"Converted using {mode_used} mode. Analysis: {analysis.reason}"
    return message, analysis, warnings


# Typer CLI
app = typer.Typer(help="Docker conversion commands")


@app.command()
def convert(
    dockerfile: Path = typer.Argument(..., help="Path to Dockerfile to convert"),
    output: Optional[Path] = typer.Option(None, "--output", "-o", help="Output path (default: overwrite input)"),
    mode: ConversionMode = typer.Option(ConversionMode.AUTO, "--mode", "-m", help="Conversion mode"),
    force_base: Optional[str] = typer.Option(None, "--force-base", "-b", help="Force base image (cpu, cuda12.8, etc.)"),
    dry_run: bool = typer.Option(False, "--dry-run", "-n", help="Don't write, just analyze"),
    strip_base_setup: bool = typer.Option(False, "--strip-base-setup", help="Remove conda/env setup already in base image"),
) -> None:
    """Convert a Dockerfile into a DimOS module Dockerfile."""
    if not dockerfile.exists():
        typer.echo(f"Error: {dockerfile} not found", err=True)
        raise typer.Exit(1)

    if force_base and force_base not in BASE_IMAGES:
        typer.echo(f"Error: Invalid base '{force_base}'. Valid options: {list(BASE_IMAGES.keys())}", err=True)
        raise typer.Exit(1)

    message, analysis, warnings = convert_dockerfile(
        dockerfile, output, mode, force_base, dry_run, strip_base_setup
    )
    typer.echo(message)

    for warning in warnings:
        typer.echo(f"Warning: {warning}", err=True)

    if dry_run:
        typer.echo("\n--- Analysis ---")
        typer.echo(f"GPU: {analysis.is_gpu}")
        typer.echo(f"CUDA version: {analysis.cuda_version or 'N/A'}")
        typer.echo(f"CUDA extensions: {analysis.has_cuda_extensions}")
        typer.echo(f"Can rebase: {analysis.can_rebase}")
        typer.echo(f"Multi-stage: {analysis.is_multi_stage}")
        typer.echo(f"Original FROM: {analysis.original_from}")
        base_image, _ = get_base_image(analysis)
        typer.echo(f"Recommended base: {base_image}")


@app.command()
def analyze(
    dockerfile: Path = typer.Argument(..., help="Path to Dockerfile to analyze"),
) -> None:
    """Analyze a Dockerfile without converting it."""
    if not dockerfile.exists():
        typer.echo(f"Error: {dockerfile} not found", err=True)
        raise typer.Exit(1)

    content = dockerfile.read_text()
    analysis = analyze_dockerfile(content)

    typer.echo(f"GPU required: {analysis.is_gpu}")
    typer.echo(f"CUDA version: {analysis.cuda_version or 'N/A'}")
    typer.echo(f"Builds CUDA extensions: {analysis.has_cuda_extensions}")
    typer.echo(f"Can use rebase (Mode A): {analysis.can_rebase}")
    typer.echo(f"Multi-stage: {analysis.is_multi_stage}")
    typer.echo(f"Reason: {analysis.reason}")
    typer.echo(f"Original FROM: {analysis.original_from}")

    base_image, warning = get_base_image(analysis)
    typer.echo(f"Recommended DimOS base: {base_image}")
    if warning:
        typer.echo(f"Warning: {warning}", err=True)


@app.command("list-bases")
def list_bases() -> None:
    """List available DimOS base images."""
    typer.echo("Available DimOS base images:")
    for name, image in BASE_IMAGES.items():
        typer.echo(f"  {name}: {image}")


if __name__ == "__main__":
    app()
