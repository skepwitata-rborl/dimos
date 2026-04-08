#!/usr/bin/env python3
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

"""Load a full GLB scene with physics colliders."""

from dimos.robot.sim.scene_client import SceneClient

COLLISION_WORLD = "/proxy?url=https://threejs.org/examples/models/gltf/collision-world.glb"

with SceneClient() as scene:
    result = scene.load_map(
        url=COLLISION_WORLD,
        name="environment",
        collider="trimesh",
    )
    print(f"Scene loaded: {result['name']} (scale: {result.get('scaleFactor', 1.0)})")
