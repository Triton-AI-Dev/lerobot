from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("remote_receiver")
@dataclass
class RemoteReceiverConfig(TeleoperatorConfig):
    port: int = 5555
    default_action: float = 0.0  # safety: value when nothing received
