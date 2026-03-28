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

"""Re-exports from dimos.simulation.mujoco.drone_sim_connection.

The SimulatedDroneConnection module lives under dimos/simulation/mujoco/.
This file re-exports symbols for backward compatibility.
"""

from dimos.simulation.mujoco.drone_sim_connection import (
    EARTH_R,
    GPS_ORIGIN_LAT,
    GPS_ORIGIN_LON,
    SimulatedDroneConnection,
    _local_to_gps,
)

__all__ = [
    "EARTH_R",
    "GPS_ORIGIN_LAT",
    "GPS_ORIGIN_LON",
    "SimulatedDroneConnection",
    "_local_to_gps",
]
