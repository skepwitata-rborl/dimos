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

"""Blueprint for the MuJoCo drone simulation.

Usage:
    from dimos.robot.drone.blueprints.sim.drone_sim import drone_sim_agentic
    drone_sim_agentic().build().loop()
"""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.web_human_input import WebInput
from dimos.core.blueprints import Blueprint, autoconnect
from dimos.simulation.mujoco.drone_sim_connection import SimulatedDroneConnection
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

DRONE_SIM_SYSTEM_PROMPT = """\
You are controlling a simulated Skydio X2 quadrotor drone in a MuJoCo city environment.
You have access to these drone control skills:
- move(x, y, z, duration): Move with velocity (m/s). x=forward, y=right, z=up.
- move_with_yaw(vx, vy, vz, yaw_rate, duration): Move with velocity AND yaw rotation.
- takeoff(altitude): Takeoff to altitude in meters.
- land(): Land the drone.
- arm() / disarm(): Arm or disarm motors.
- set_mode(mode): Set flight mode (GUIDED, LAND, RTL, STABILIZE).
- fly_to(lat, lon, alt): Fly to GPS coordinates.

The drone is already hovering at 3m altitude, armed in GUIDED mode."""


def drone_sim_agentic(
    scene: str = "city",
    system_prompt: str = DRONE_SIM_SYSTEM_PROMPT,
    model: str = "gpt-4o",
    headless: bool = False,
) -> Blueprint:
    """Simulated drone with Nightwatch agent.

    Args:
        scene: Scene name under data/mujoco_sim/ (e.g. "city" loads city_scene.xml)
        system_prompt: Agent system prompt
        model: LLM model name
        headless: Skip camera rendering for faster simulation
    """
    return autoconnect(
        SimulatedDroneConnection.blueprint(
            scene=scene,
            headless=headless,
        ),
        WebsocketVisModule.blueprint(),
        McpServer.blueprint(),
        McpClient.blueprint(system_prompt=system_prompt, model=model),
        WebInput.blueprint(),
    )
