# PGO Go2 SmartNav Integration Test — Context Dump

## What was done

### 1. PGO: `unregister_input` config (DONE)
**File:** `dimos/navigation/smartnav/modules/pgo/pgo.py`
- Added `unregister_input: bool = True` to `PGOConfig` (line 57)
- Wrapped body-frame transform in `_on_scan` (line 450-454) with conditional:
  ```python
  if self.config.unregister_input:
      body_pts = (r_local.T @ (points[:, :3].T - t_local[:, None])).T
  else:
      body_pts = points[:, :3]
  ```

### 2. OdomAdapter module (DONE)
**Files created:**
- `dimos/navigation/smartnav/modules/odom_adapter/__init__.py` (empty)
- `dimos/navigation/smartnav/modules/odom_adapter/odom_adapter.py`

Streams:
- `raw_odom: In[PoseStamped]` → converts to → `odometry: Out[Odometry]` (for PGO)
- `corrected_odometry: In[Odometry]` → converts to → `odom: Out[PoseStamped]` (for planner)

Uses `._transport.subscribe()` pattern same as PGO.

### 3. Blueprint (DONE)
**File:** `dimos/robot/unitree/go2/blueprints/smart/unitree_go2_smartnav.py`

```python
unitree_go2_smartnav = autoconnect(
    unitree_go2_basic,
    PGO.blueprint(),
    odom_adapter(),
    cost_mapper(),
    replanning_a_star_planner(),
    wavefront_frontier_explorer(),
).global_config(n_workers=8, robot_model="unitree_go2").remappings([
    (GO2Connection, "lidar", "registered_scan"),
    (GO2Connection, "odom", "raw_odom"),
])
```

### 4. Integration test (IN PROGRESS)
**File:** `dimos/e2e_tests/test_smartnav_replay.py`

Test was written but couldn't run on macOS because:
- `gtsam` has no arm64 macOS wheel (only x86_64 macOS and Linux)
- Need to run on Linux machine

## How replay works

`GO2Connection` checks `global_config.unitree_connection_type`:
- If `replay=True`, it creates a `ReplayConnection(dataset=replay_dir)`
- `ReplayConnection` (in `dimos/robot/unitree/go2/connection.py:111`) extends `UnitreeWebRTCConnection`
- It uses `TimedSensorReplay` (which is `LegacyPickleStore`) to load pickle files from `data/{dataset}/lidar/`, `data/{dataset}/odom/`, `data/{dataset}/video/`
- Default dataset: `"go2_sf_office"` — has 74 lidar frames, 182 odom frames
- `.stream()` returns an RxPY Observable that replays with original timing

## How to run the test

```bash
# Set replay mode via global_config
global_config.update(viewer="none", replay=True, replay_dir="go2_sf_office", n_workers=1)

# Build minimal pipeline (no planner needed for data flow test)
bp = autoconnect(
    unitree_go2_basic,
    PGO.blueprint(),
    odom_adapter(),
    cost_mapper(),
).global_config(n_workers=1, robot_model="unitree_go2").remappings([
    (GO2Connection, "lidar", "registered_scan"),
    (GO2Connection, "odom", "raw_odom"),
])
coord = bp.build()
coord.start()
```

## Key patterns for tests

### Finding modules in coordinator
```python
for mod in coord.all_modules:
    if isinstance(mod, PGO):
        pgo_mod = mod
```

### Subscribing to outputs
```python
collector = []
pgo_mod.corrected_odometry._transport.subscribe(lambda msg: collector.append(msg))
```

### Existing test patterns
- See `dimos/core/test_e2e_daemon.py` for blueprint build/start/stop lifecycle
- See `dimos/e2e_tests/conftest.py` for LCM spy fixture pattern
- Tests use `@pytest.mark.slow` marker
- CI env: `monkeypatch.setenv("CI", "1")` to skip sysctl interactive prompt

## Key imports

```python
from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.costmapper import cost_mapper, CostMapper
from dimos.navigation.smartnav.modules.pgo.pgo import PGO
from dimos.navigation.smartnav.modules.odom_adapter.odom_adapter import OdomAdapter, odom_adapter
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
```

## Data flow to verify

```
GO2Connection.lidar (remapped→registered_scan) → PGO.registered_scan
GO2Connection.odom (remapped→raw_odom) → OdomAdapter.raw_odom
OdomAdapter.odometry → PGO.odometry
PGO.corrected_odometry → OdomAdapter.corrected_odometry
OdomAdapter.odom → (planner would consume)
PGO.global_map → CostMapper.global_map
CostMapper.global_costmap → (planner would consume)
```

## Test file already written at `dimos/e2e_tests/test_smartnav_replay.py`

Just needs gtsam available. Add `pytest.importorskip("gtsam")` at top if you want graceful skip on machines without it.
