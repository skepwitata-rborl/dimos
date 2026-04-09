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
# type: ignore

from __future__ import annotations

import threading

import pytest

from dimos.core.tests.stress_test_module import StressTestModule
from dimos.porcelain import Dimos, _resolve_target


@pytest.fixture
def app():
    instance = Dimos()
    try:
        yield instance
    finally:
        instance.stop()


def test_resolve_module_class():
    bp = _resolve_target(StressTestModule)
    assert bp is not None


def test_resolve_blueprint_object():
    bp = StressTestModule.blueprint()
    assert _resolve_target(bp) is bp


def test_resolve_invalid_type():
    with pytest.raises(TypeError, match="run\\(\\) expects"):
        _resolve_target(42)  # type: ignore[arg-type]


def test_default_construction(app):
    assert not app.is_running


def test_construction_with_overrides():
    instance = Dimos(n_workers=4)
    try:
        assert instance._config_overrides == {"n_workers": 4}
    finally:
        instance.stop()


def test_repr_when_stopped(app):
    assert "stopped" in repr(app)


def test_skills_before_run(app):
    with pytest.raises(RuntimeError, match="No modules are running"):
        _ = app.skills


def test_restart_before_run(app):
    with pytest.raises(RuntimeError, match="No modules are running"):
        app.restart(StressTestModule)


def test_run_after_stop(app):
    app.stop()
    with pytest.raises(RuntimeError, match="stopped"):
        app.run(StressTestModule)


def test_getattr_unknown_module(app):
    app.run(StressTestModule)
    with pytest.raises(AttributeError, match="No module named"):
        _ = app.Nonexistent


def test_getattr_exists_but_not_running(app):
    app.run(StressTestModule)
    with pytest.raises(AttributeError, match="exists but is not running"):
        _ = app.CameraModule


def test_run_module_class():
    with Dimos(n_workers=1) as app:
        app.run(StressTestModule)
        assert app.is_running
        assert "StressTestModule" in repr(app)
    assert not app.is_running


def test_run_blueprint_object():
    bp = StressTestModule.blueprint()
    with Dimos(n_workers=1) as app:
        app.run(bp)
        assert app.is_running


def test_context_manager():
    with Dimos(n_workers=1) as app:
        app.run(StressTestModule)
        assert app.is_running
    assert not app.is_running


@pytest.mark.slow
def test_skills_discovery():
    with Dimos(n_workers=1) as app:
        app.run(StressTestModule)
        skills = app.skills
        assert "echo" in dir(skills)
        assert "ping" in dir(skills)
        rep = repr(skills)
        assert "echo" in rep
        assert "ping" in rep


@pytest.mark.slow
def test_skill_call():
    with Dimos(n_workers=1) as app:
        app.run(StressTestModule)
        result = app.skills.echo(message="hello")
        assert result == "hello"


@pytest.mark.slow
def test_skill_ping():
    with Dimos(n_workers=1) as app:
        app.run(StressTestModule)
        result = app.skills.ping()
        assert result == "pong"


@pytest.mark.slow
def test_rpyc_module_access():
    with Dimos(n_workers=1) as app:
        app.run(StressTestModule)
        module = app.StressTestModule
        # Access an attribute from ModuleBase
        assert module._module_closed is False


@pytest.mark.slow
def test_dir_lists_modules():
    with Dimos(n_workers=1) as app:
        app.run(StressTestModule)
        d = dir(app)
        assert "StressTestModule" in d
        assert "run" in d
        assert "stop" in d


@pytest.mark.slow
def test_thread_safety():
    with Dimos(n_workers=1) as app:
        app.run(StressTestModule)
        results: list[str] = []
        errors: list[Exception] = []

        def call_skill():
            try:
                r = app.skills.ping()
                results.append(r)
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=call_skill) for _ in range(5)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=30)

        assert not errors, f"Errors in threads: {errors}"
        assert len(results) == 5
        assert all(r == "pong" for r in results)


@pytest.mark.slow
def test_restart_no_reload():
    with Dimos(n_workers=1) as app:
        app.run(StressTestModule)
        app.restart(StressTestModule, reload_source=False)
        result = app.skills.ping()
        assert result == "pong"
