# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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
Simple script to control a robot from teleoperation.

Example:

```shell
python -m lerobot.teleoperate \
    --robot.type=so101_follower \
    --robot.port=/dev/tty.usbmodem58760431541 \
    --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 1920, height: 1080, fps: 30}}" \
    --robot.id=black \
    --teleop.type=so101_leader \
    --teleop.port=/dev/tty.usbmodem58760431551 \
    --teleop.id=blue \
    --display_data=true
```

Example teleoperation with bimanual so100:

```shell
python -m lerobot.teleoperate \
  --robot.type=bi_so100_follower \
  --robot.left_arm_port=/dev/tty.usbmodem5A460851411 \
  --robot.right_arm_port=/dev/tty.usbmodem5A460812391 \
  --robot.id=bimanual_follower \
  --robot.cameras='{
    left: {"type": "opencv", "index_or_path": 0, "width": 1920, "height": 1080, "fps": 30},
    top: {"type": "opencv", "index_or_path": 1, "width": 1920, "height": 1080, "fps": 30},
    right: {"type": "opencv", "index_or_path": 2, "width": 1920, "height": 1080, "fps": 30}
  }' \
  --teleop.type=bi_so100_leader \
  --teleop.left_arm_port=/dev/tty.usbmodem5A460828611 \
  --teleop.right_arm_port=/dev/tty.usbmodem5A460826981 \
  --teleop.id=bimanual_leader \
  --display_data=true
```

"""

import logging
import time
from dataclasses import asdict, dataclass
from pprint import pformat

import draccus
import rerun as rr

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.cameras.realsense.configuration_realsense import (
    RealSenseCameraConfig,
)  # noqa: F401
from lerobot.robots import (  # noqa: F401
    Robot,
    RobotConfig,
    bi_so100_follower,
    hope_jr,
    koch_follower,
    make_robot_from_config,
    so100_follower,
    so101_follower,
    remote_sender,
)
from lerobot.teleoperators import (  # noqa: F401
    Teleoperator,
    TeleoperatorConfig,
    bi_so100_leader,
    gamepad,
    homunculus,
    koch_leader,
    make_teleoperator_from_config,
    so100_leader,
    so101_leader,
    remote_receiver,
)
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import init_logging, move_cursor_up
from lerobot.utils.visualization_utils import _init_rerun, log_rerun_data
import select
import errno
import select
import time
from typing import Any


@dataclass
class TeleoperateConfig:
    # TODO: pepijn, steven: if more robots require multiple teleoperators (like lekiwi) its good to make this possibele in teleop.py and record.py with List[Teleoperator]
    teleop: TeleoperatorConfig
    robot: RobotConfig
    # Limit the maximum frames per second.
    fps: int = 60
    teleop_time_s: float | None = None
    # Display all cameras on screen
    display_data: bool = False


# ------------------------------------------------------------------------- #
# Helper to fetch-→visualize-→send-→print  (shared by both branches)
# ------------------------------------------------------------------------- #
def _make_process_once(
    teleop,
    robot,
    display_data: bool,
    fps: int,
    display_len: int,
):
    def _inner() -> bool:
        try:
            action = teleop.get_action()
        except OSError as e:
            # Ignore “would block” and quietly continue; re-raise other errors
            if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return False
            raise

        if not action:
            return False

        if display_data:
            observation = robot.get_observation()
            log_rerun_data(observation, action)

        robot.send_action(action)

        # Pretty print ---------------------------------------------------- #
        print("\n" + "-" * (display_len + 10))
        print(f"{'NAME':<{display_len}} | {'NORM':>7}")
        for motor, value in action.items():
            print(f"{motor:<{display_len}} | {value:>7.2f}")
        print(f"\ntime: {1_000 / fps:.2f}ms ({fps} Hz)")
        move_cursor_up(len(action) + 5)
        # ----------------------------------------------------------------- #
        return True

    return _inner


# ------------------------------------------------------------------------- #
# Main loop
# ------------------------------------------------------------------------- #
def teleop_loop(
    teleop: Teleoperator,
    robot: Robot,
    fps: int,
    display_data: bool = False,
    duration: float | None = None,
):
    timeout_ms = int(1_000 / fps)

    # ---------- event-driven setup (only if FD is valid & poll exists) ---- #
    fd = getattr(teleop, "socket_fileno", None)
    event_driven = isinstance(fd, int) and fd >= 0 and hasattr(select, "poll")

    poller = select.poll() if event_driven else None
    if event_driven:
        try:
            poller.register(fd, select.POLLIN | select.POLLERR | select.POLLHUP)
        except ValueError:  # FD turned out to be invalid
            event_driven = False
            poller = None

    display_len = max((len(k) for k in robot.action_features), default=0)
    process_once = _make_process_once(teleop, robot, display_data, fps, display_len)
    start = time.perf_counter()

    while True:
        if event_driven:
            # Wait for up to timeout_ms; may return an empty list.
            for _, mask in poller.poll(timeout_ms):
                # We only care about “read ready.”
                if mask & select.POLLIN:
                    sent = process_once()
                    if (
                        sent
                        and duration is not None
                        and time.perf_counter() - start >= duration
                    ):
                        return
                # You might handle POLLERR / POLLHUP here if desired.
        else:
            loop_start = time.perf_counter()

            sent = process_once()
            if (
                sent
                and duration is not None
                and time.perf_counter() - start >= duration
            ):
                return

            # Sleep to maintain target FPS
            sleep = max(0.0, 1.0 / fps - (time.perf_counter() - loop_start))
            if sleep:
                time.sleep(sleep)


@draccus.wrap()
def teleoperate(cfg: TeleoperateConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))
    if cfg.display_data:
        _init_rerun(session_name="teleoperation")

    teleop = make_teleoperator_from_config(cfg.teleop)
    robot = make_robot_from_config(cfg.robot)

    teleop.connect()
    robot.connect()

    try:
        teleop_loop(
            teleop,
            robot,
            cfg.fps,
            display_data=cfg.display_data,
            duration=cfg.teleop_time_s,
        )
    except KeyboardInterrupt:
        pass
    finally:
        if cfg.display_data:
            rr.rerun_shutdown()
        teleop.disconnect()
        robot.disconnect()


if __name__ == "__main__":
    teleoperate()
