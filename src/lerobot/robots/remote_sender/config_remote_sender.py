from dataclasses import dataclass

from ..config import RobotConfig


@RobotConfig.register_subclass("remote_sender")
@dataclass
class RemoteSenderConfig(RobotConfig):
    """
    Robot that *sends* actions over UDP to a remote receiver.

    Fields inherited from RobotConfig:
      - id: str | None
      - calibration_dir: Path | None (unused here)

    Extra fields:
      - host: destination IP or hostname (where the follower's RemoteReceiver listens)
      - port: destination UDP port
      - keys: ordered action keys used for packing; must match the receiver's expectations
    """

    host: str
    port: int
    # Explicit keys so teleoperate() has something to display and to keep packing stable.
    keys: tuple[str, ...] = (
        "shoulder_pan.pos",
        "shoulder_lift.pos",
        "elbow_flex.pos",
        "wrist_flex.pos",
        "wrist_roll.pos",
        "gripper.pos",
    )
