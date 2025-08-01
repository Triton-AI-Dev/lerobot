from dataclasses import dataclass, field
from typing import Optional
from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("remote_sender")
@dataclass
class RemoteSenderConfig(TeleoperatorConfig):
    host: str
    port: int = 5555

    local_type: str = "gamepad"
    local_port: str | None = None
