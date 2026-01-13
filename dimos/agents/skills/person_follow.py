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

from threading import Event, RLock, Thread
import time
from typing import TYPE_CHECKING

import numpy as np

from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.rpc_client import RpcCall
from dimos.core.skill_module import SkillModule
from dimos.core.stream import In
from dimos.models.qwen.video_query import BBox
from dimos.models.segmentation.edge_tam import EdgeTAMProcessor
from dimos.models.vl.qwen import QwenVlModel
from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.msgs.sensor_msgs import CameraInfo, Image
from dimos.navigation.visual.query import get_object_bbox_from_image
from dimos.protocol.skill.skill import skill
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.models.vl.base import VlModel

logger = setup_logger()


class PersonFollowSkillContainer(SkillModule):
    """Skill container for following a person using visual servoing with EdgeTAM.

    This skill uses:
    - A VL model (QwenVlModel) to initially detect a person from a text description.
    - EdgeTAM for continuous tracking across frames.
    - Visual servoing to control robot movement towards the person.
    - Does not do obstacle avoidance; assumes a clear path.
    """

    rpc_calls: list[str] = ["GO2Connection.move"]

    color_image: In[Image]

    # Visual servoing parameters
    _target_distance: float = 1.5  # meters - target distance to maintain from person
    _min_distance: float = 0.8  # meters - stop if closer than this
    _max_linear_speed: float = 0.5  # m/s - maximum forward/backward speed
    _max_angular_speed: float = 0.8  # rad/s - maximum turning speed
    _frequency: float = 20.0  # Hz - control loop frequency
    _max_lost_frames: int = 15  # number of frames to wait before declaring person lost

    # Distance is estimated by assuming the width of a person. We don't use height because
    # when the robot is close to the person, it often cannot see his full height.
    _assumed_person_width: float = 0.45  # meters (average shoulder width)

    def __init__(self, camera_info: CameraInfo, global_config: GlobalConfig) -> None:
        super().__init__()
        self._global_config: GlobalConfig = global_config
        self._latest_image: Image | None = None
        self._vl_model: VlModel = QwenVlModel()
        self._tracker: EdgeTAMProcessor | None = None
        self._should_stop: Event = Event()
        self._lock = RLock()
        self._thread: Thread | None = None
        self._thread_lock = RLock()
        self._camera_info: CameraInfo = camera_info

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(self.color_image.subscribe(self._on_color_image))

    @rpc
    def stop(self) -> None:
        self._stop_following()

        with self._lock:
            if self._tracker is not None:
                self._tracker.stop()
                self._tracker = None

        self._vl_model.stop()
        super().stop()

    @skill()
    def follow_person(self, query: str) -> str:
        """Follow a person matching the given description using visual servoing.

        The robot will continuously track and follow the person, maintaining
        a safe distance while keeping them centered in the camera view.

        Uses EdgeTAM for robust video object tracking and segmentation.

        Args:
            query: Description of the person to follow (e.g., "man with blue shirt")

        Returns:
            Status message indicating the result of the following action.

        Example:
            follow_person("man with blue shirt")
            follow_person("person in the doorway")
        """

        self._stop_following()

        self._should_stop.clear()

        with self._lock:
            latest_image = self._latest_image

        if latest_image is None:
            return "No image available to detect person."

        initial_bbox = get_object_bbox_from_image(
            self._vl_model,
            latest_image,
            query,
        )

        if initial_bbox is None:
            return f"Could not find '{query}' in the current view."

        with self._thread_lock:
            self._thread = Thread(
                target=self._follow_loop,
                args=(query, initial_bbox),
                daemon=True,
            )
            self._thread.start()

        return "Started following. Call stop_following() to stop."

    @skill()
    def stop_following(self) -> str:
        """Stop following the current person.

        Returns:
            Confirmation message.
        """
        self._stop_following()

        try:
            move_rpc = self.get_rpc_calls("GO2Connection.move")
            move_rpc(Twist.zero())
        except Exception:
            pass

        return "Stopped following."

    def _on_color_image(self, image: Image) -> None:
        with self._lock:
            self._latest_image = image

    def _follow_loop(self, query: str, initial_bbox: BBox) -> None:
        x1, y1, x2, y2 = initial_bbox
        box = np.array([x1, y1, x2, y2], dtype=np.float32)

        move_rpc = self.get_rpc_calls("GO2Connection.move")

        with self._lock:
            if self._tracker is None:
                self._tracker = EdgeTAMProcessor()
            tracker = self._tracker
            latest_image = self._latest_image
            if latest_image is None:
                self._error_when_following(move_rpc, "No image available to start tracking.")
                return

        initial_detections = tracker.init_track(
            image=latest_image,
            box=box,
            obj_id=1,
        )

        if len(initial_detections) == 0:
            self._error_when_following(move_rpc, f"EdgeTAM failed to segment '{query}'.")
            return

        logger.info(f"EdgeTAM initialized with {len(initial_detections)} detections")

        lost_count = 0
        period = 1.0 / self._frequency
        next_time = time.monotonic()

        while not self._should_stop.is_set():
            next_time += period

            with self._lock:
                latest_image = self._latest_image
                assert latest_image is not None

            detections = tracker.process_image(latest_image)

            if len(detections) == 0:
                move_rpc(Twist.zero())

                lost_count += 1
                if lost_count > self._max_lost_frames:
                    self._error_when_following(move_rpc, f"Lost track of '{query}'. Stopping.")
                    return
            else:
                lost_count = 0
                best_detection = max(detections.detections, key=lambda d: d.bbox_2d_volume())
                twist = self._compute_visual_servo_twist(
                    best_detection.bbox,
                    latest_image.width,
                )
                move_rpc(twist)

            now = time.monotonic()
            sleep_duration = next_time - now
            if sleep_duration > 0:
                time.sleep(sleep_duration)

        move_rpc(Twist.zero())

    def _error_when_following(self, move_rpc: RpcCall, message: str) -> None:
        move_rpc(Twist.zero())

        # TODO: These errors should be communicated back to the agent.
        logger.error(message)
        self._should_stop.set()

    def _stop_following(self) -> None:
        self._should_stop.set()

        with self._thread_lock:
            if self._thread is None:
                return
            if self._thread.is_alive():
                self._thread.join(timeout=2.0)
            self._thread = None

    def _get_normalized_x(self, pixel_x: float) -> float | None:
        """Convert pixel x coordinate to normalized camera coordinate.

        Uses inverse K matrix: x_norm = (pixel_x - cx) / fx

        Args:
            pixel_x: x coordinate in pixels

        Returns:
            Normalized x coordinate (tan of angle), or None if no camera info
        """
        fx = self._camera_info.K[0]  # focal length x
        cx = self._camera_info.K[2]  # optical center x
        return (pixel_x - cx) / fx

    def _estimate_distance(self, bbox: tuple[float, float, float, float]) -> float | None:
        """Estimate distance to person based on bounding box size and camera intrinsics.

        Uses the pinhole camera model:
            pixel_width / fx = real_width / distance
            distance = (real_width * fx) / pixel_width

        Uses bbox width instead of height because ground robot can't see full
        person height when close. Width (shoulders) is more consistently visible.
        """
        bbox_width = bbox[2] - bbox[0]  # x2 - x1

        if bbox_width <= 0:
            return None

        # Pinhole camera model: distance = (real_width * fx) / pixel_width
        fx = self._camera_info.K[0]  # focal length x in pixels
        estimated_distance = (self._assumed_person_width * fx) / bbox_width

        return estimated_distance

    def _compute_visual_servo_twist(
        self,
        bbox: tuple[float, float, float, float],
        image_width: int,
    ) -> Twist:
        """Compute twist command to servo towards the person.

        Args:
            bbox: Bounding box (x1, y1, x2, y2) in pixels
            image_width: Width of the image

        Returns:
            Twist command for the robot
        """
        x1, _, x2, _ = bbox
        bbox_center_x = (x1 + x2) / 2.0

        # Get normalized x coordinate using inverse K matrix
        # Positive = person is to the right of optical center
        x_norm = self._get_normalized_x(bbox_center_x)
        if x_norm is None:
            # Fallback: assume cx at image center, fx ~= image_width (rough approx)
            x_norm = (bbox_center_x - image_width / 2.0) / image_width

        estimated_distance = self._estimate_distance(bbox)

        if estimated_distance is None:
            return Twist.zero()

        # Calculate distance error (positive = too far, need to move forward)
        distance_error = estimated_distance - self._target_distance

        # Compute angular velocity (turn towards person)
        # Negative because positive angular.z is counter-clockwise (left turn)
        angular_z = -x_norm * 1.0  # Proportional gain (reduced from 2.0 to avoid oscillation)
        angular_z = float(np.clip(angular_z, -self._max_angular_speed, self._max_angular_speed))

        # Compute linear velocity - ALWAYS move forward/backward based on distance
        # Reduce forward speed when turning sharply to maintain stability
        turn_factor = 1.0 - min(abs(x_norm) * 2.0, 0.7)  # Range: [0.3, 1.0]

        if estimated_distance < self._min_distance:
            # Too close, back up (don't reduce speed for backing up)
            linear_x = -self._max_linear_speed * 0.6
        else:
            # Move forward based on distance error with proportional gain
            # Gain of 0.8 means at 1m error, we'd want 0.8 m/s (before clamping)
            linear_x = distance_error * 0.8 * turn_factor
            linear_x = float(np.clip(linear_x, -self._max_linear_speed, self._max_linear_speed))

        return Twist(
            linear=Vector3(linear_x, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_z),
        )


person_follow_skill = PersonFollowSkillContainer.blueprint

__all__ = ["PersonFollowSkillContainer", "person_follow_skill"]
