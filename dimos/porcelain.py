# Copyright 2026 Dimensional Inc.
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

from __future__ import annotations

import atexit
import inspect
import json
import threading
from typing import TYPE_CHECKING, Any

import rpyc

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.module import ModuleBase
from dimos.robot.all_blueprints import all_modules
from dimos.robot.get_all_blueprints import get_by_name

if TYPE_CHECKING:
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.core.module import SkillInfo
    from dimos.core.rpc_client import RPCClient


class Dimos:
    def __init__(self, **config_overrides: Any) -> None:
        self._config_overrides = config_overrides
        self._coordinator: ModuleCoordinator | None = None
        self._lock = threading.RLock()
        self._rpyc_cache: dict[type[ModuleBase], tuple[rpyc.Connection, Any]] = {}
        self._stopped = False
        atexit.register(self.stop)

    def run(self, target: str | Blueprint | type[ModuleBase]) -> None:
        """Start a blueprint, module, or named configuration.

        Args:
            target: One of:
                - A string name from the blueprint/module registry
                  (e.g. ``"unitree-go2-basic"`` or ``"camera-module"``).
                - A ``Blueprint`` object.
                - A ``Module`` class (calls ``.blueprint()`` automatically).

        The first call creates the coordinator and starts the system.
        Subsequent calls add modules to the already-running system.
        """
        blueprint = _resolve_target(target)

        with self._lock:
            if self._stopped:
                raise RuntimeError("This Dimos instance has been stopped")

            if self._coordinator is None:
                from dimos.core.coordination.module_coordinator import ModuleCoordinator
                from dimos.core.global_config import global_config

                if self._config_overrides:
                    global_config.update(**self._config_overrides)
                self._coordinator = ModuleCoordinator.build(blueprint)
            else:
                self._coordinator.load_blueprint(blueprint)

    def restart(self, module_class: type[ModuleBase], *, reload_source: bool = True) -> None:
        """Restart a running module, optionally reloading its source.

        Args:
            module_class: The module class to restart.
            reload_source: If True (default), reload the module's source file
                so code changes are picked up.

        Delegates to ``ModuleCoordinator.restart_module()``.
        """
        with self._lock:
            if self._coordinator is None:
                raise RuntimeError("No modules are running")
            self._invalidate_rpyc(module_class)
            self._coordinator.restart_module(module_class, reload_source=reload_source)

    @property
    def skills(self) -> _SkillsProxy:
        """Access skills from all running modules.

        Returns a proxy that supports attribute access and pretty-printing::

            app.skills.relative_move(forward=2.0)
            print(app.skills)
        """
        with self._lock:
            if self._coordinator is None:
                raise RuntimeError("No modules are running")
            return _SkillsProxy(self._coordinator)

    def stop(self) -> None:
        """Stop all modules and clean up resources."""
        with self._lock:
            if self._stopped:
                return
            self._stopped = True

            for conn, _ in self._rpyc_cache.values():
                try:
                    conn.close()
                except Exception:
                    pass
            self._rpyc_cache.clear()

            if self._coordinator is not None:
                self._coordinator.stop()
                self._coordinator = None

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._coordinator is not None and not self._stopped

    def _invalidate_rpyc(self, module_class: type[ModuleBase]) -> None:
        """Close and remove cached RPyC connection for a module class."""
        entry = self._rpyc_cache.pop(module_class, None)
        if entry is not None:
            try:
                entry[0].close()
            except Exception:
                pass

    def _get_rpyc_proxy(self, module_class: type[ModuleBase], proxy: RPCClient) -> Any:
        """Get or create an RPyC proxy to a remote module instance."""
        if module_class in self._rpyc_cache:
            conn, module_proxy = self._rpyc_cache[module_class]
            if not conn.closed:
                return module_proxy

        actor = proxy.actor_instance
        port = actor.start_rpyc()
        conn = rpyc.connect("localhost", port, config={"sync_request_timeout": 30})
        module_proxy = conn.root.get_module(actor._module_id)
        self._rpyc_cache[module_class] = (conn, module_proxy)
        return module_proxy

    def __getattr__(self, name: str) -> Any:
        if name.startswith("_"):
            raise AttributeError(name)

        with self._lock:
            if self._coordinator is None:
                raise RuntimeError("No modules are running")

            for cls in self._coordinator._deployed_modules:
                if cls.__name__ == name:
                    proxy = self._coordinator._deployed_modules[cls]
                    return self._get_rpyc_proxy(cls, proxy)  # type: ignore[arg-type]

        known_names = _all_module_class_names()
        if name in known_names:
            raise AttributeError(
                f"{name} exists but is not running. Start it with app.run({name}) "
                f"or add it to your blueprint."
            )
        raise AttributeError(f"No module named {name!r}")

    def __repr__(self) -> str:
        with self._lock:
            if self._coordinator is None:
                return "Dimos(stopped)"
            modules = [cls.__name__ for cls in self._coordinator._deployed_modules]
            return f"Dimos(modules={modules})"

    def __enter__(self) -> Dimos:
        return self

    def __exit__(self, *args: Any) -> None:
        self.stop()

    def __dir__(self) -> list[str]:
        base = list(super().__dir__())
        with self._lock:
            if self._coordinator is not None:
                base.extend(cls.__name__ for cls in self._coordinator._deployed_modules)
        return base


def _resolve_target(target: str | Blueprint | type[ModuleBase]) -> Blueprint:
    """Convert a run() argument into a Blueprint."""

    if isinstance(target, str):
        return get_by_name(target)
    if isinstance(target, Blueprint):
        return target
    if inspect.isclass(target) and issubclass(target, ModuleBase):
        return target.blueprint()  # type: ignore[no-any-return]
    raise TypeError(
        f"run() expects a blueprint name (str), Blueprint, or Module class, got {type(target).__name__}"
    )


def _all_module_class_names() -> set[str]:
    """Return the set of all known module class names from the registry."""
    return {path.rsplit(".", 1)[-1] for path in all_modules.values()}


class _SkillCallable:
    """Callable wrapper around a remote skill method."""

    def __init__(self, proxy: RPCClient, name: str, info: SkillInfo) -> None:
        self._proxy = proxy
        self._name = name
        self._info = info

    def __call__(self, *args: Any, **kwargs: Any) -> Any:
        rpc_call = getattr(self._proxy, self._name)
        return rpc_call(*args, **kwargs)

    def __repr__(self) -> str:
        schema = json.loads(self._info.args_schema)
        params = _format_params(schema)
        return f"Skill: {self._info.class_name}.{self._name}({params})"


class _SkillsProxy:
    """Attribute-access proxy that discovers and exposes skills from all deployed modules."""

    def __init__(self, coordinator: ModuleCoordinator) -> None:
        self._coordinator = coordinator
        self._cache: dict[str, list[tuple[type[ModuleBase], RPCClient, SkillInfo]]] | None = None
        self._cache_key: frozenset[type[ModuleBase]] | None = None

    def _build_cache(self) -> None:
        modules_key = frozenset(self._coordinator._deployed_modules.keys())
        if self._cache_key == modules_key and self._cache is not None:
            return

        skill_map: dict[str, list[tuple[type[ModuleBase], RPCClient, SkillInfo]]] = {}
        for cls, proxy in self._coordinator._deployed_modules.items():
            try:
                skills: list[SkillInfo] = proxy.get_skills()  # type: ignore[attr-defined]
            except Exception:
                continue
            for info in skills:
                skill_map.setdefault(info.func_name, []).append((cls, proxy, info))  # type: ignore[arg-type]

        self._cache = skill_map
        self._cache_key = modules_key

    def __getattr__(self, name: str) -> _SkillCallable:
        if name.startswith("_"):
            raise AttributeError(name)
        self._build_cache()
        assert self._cache is not None

        if name not in self._cache:
            raise AttributeError(f"No skill named {name!r}")

        entries = self._cache[name]
        if len(entries) > 1:
            modules = [cls.__name__ for cls, _, _ in entries]
            raise AttributeError(
                f"Ambiguous skill {name!r} found in modules: {modules}. "
                f"Call via module directly: app.{modules[0]}.{name}()"
            )
        _cls, proxy, info = entries[0]
        return _SkillCallable(proxy, name, info)

    def __repr__(self) -> str:
        self._build_cache()
        assert self._cache is not None

        if not self._cache:
            return "Skills: (none)"

        lines = ["Skills:"]
        for name in sorted(self._cache):
            for cls, _, info in self._cache[name]:
                schema = json.loads(info.args_schema)
                params = _format_params(schema)
                lines.append(f"  {name}({params})")
                desc = schema.get("description", "")
                if desc:
                    lines.append(f"    {desc}")
                lines.append(f"    [{cls.__name__}]")
        return "\n".join(lines)

    def __dir__(self) -> list[str]:
        self._build_cache()
        assert self._cache is not None
        return list(self._cache.keys())


def _format_params(schema: dict[str, Any]) -> str:
    """Format JSON Schema properties into a Python-style parameter string."""
    props = schema.get("properties", {})
    required = set(schema.get("required", []))
    parts: list[str] = []
    for pname, pdef in props.items():
        ptype = pdef.get("type", "Any")
        type_map = {"number": "float", "integer": "int", "string": "str", "boolean": "bool"}
        ptype = type_map.get(ptype, ptype)
        if "default" in pdef:
            parts.append(f"{pname}: {ptype} = {pdef['default']!r}")
        elif pname not in required:
            parts.append(f"{pname}: {ptype} = None")
        else:
            parts.append(f"{pname}: {ptype}")
    return ", ".join(parts)
