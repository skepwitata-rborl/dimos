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

"""Record G1 joint states from LCM to a JSON file.

Run in a separate terminal while ``unitree-g1-lowlevel`` is active.

Usage:
    RECORD_FILE=macarena.json python -m dimos.control.examples.g1_record
"""

from __future__ import annotations

import json
import os
import threading
import time

from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs import JointState

_TOPIC = "/coordinator/joint_state"


def main() -> None:
    out_file = os.getenv("RECORD_FILE", "g1_recording.json")

    transport = LCMTransport(_TOPIC, JointState)

    joint_names: list[str] = []
    samples: list[dict] = []
    lock = threading.Lock()

    def on_joint_state(msg: JointState) -> None:
        nonlocal joint_names
        with lock:
            if not joint_names:
                joint_names = list(msg.name)
            samples.append({"ts": msg.ts, "position": list(msg.position)})

    transport.subscribe(on_joint_state)
    print(f"Recording from {_TOPIC} → {out_file}  (Ctrl+C to stop)")

    try:
        while True:
            with lock:
                n = len(samples)
            if n > 0 and n % 500 == 0:
                print(f"  {n} samples recorded...")
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass

    with lock:
        recording = {"joint_names": joint_names, "samples": samples}

    with open(out_file, "w") as f:
        json.dump(recording, f)

    print(f"\nSaved {len(samples)} samples to {out_file}")


if __name__ == "__main__":
    main()
