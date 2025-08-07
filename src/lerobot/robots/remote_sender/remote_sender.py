#!/usr/bin/env python
# Copyright 2025 The HuggingFace Inc. team.

from __future__ import annotations

import itertools
import socket
import struct
from typing import Any

from lerobot.net.transport import UDPSender

from ..robot import Robot
from .config_remote_sender import RemoteSenderConfig


class RemoteSender(Robot):
    """
    Robot that forwards actions over UDP to a remote follower.

    Wire format: 28 bytes = <I6f
      - seq: uint32 (monotonic, wraps at 2^32-1)
      - six floats in SO-100 order:
          pan, lift, elbow, wrist_flex, wrist_roll, grip
    """

    cfg: RemoteSenderConfig
    name = "remote_sender"
    config_class = RemoteSenderConfig

    # 1×uint32 sequence + 6×float => 28 bytes, little-endian
    _PACK = struct.Struct("<I6f").pack

    def __init__(self, config: RemoteSenderConfig):
        super().__init__(config)
        self.cfg = config
        self._connected = False

        # UDP socket bound for the remote receiver
        self._sender = UDPSender(self.cfg.host, self.cfg.port)
        # Best-effort QoS hints (silently ignored on some platforms)
        try:
            self._sender.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 16 * 1024)
        except OSError:
            pass
        try:
            if hasattr(socket, "SO_PRIORITY"):
                self._sender.sock.setsockopt(socket.SOL_SOCKET, socket.SO_PRIORITY, 6)
        except OSError:
            pass
        try:
            self._sender.sock.setsockopt(
                socket.IPPROTO_IP, socket.IP_TOS, 0x2E
            )  # AF41 DSCP
        except OSError:
            pass

        # Monotonic sequence number generator (wrap inside pack)
        self._seq_next = itertools.count(1).__next__
        # Freeze ordered keys from config for stable packing
        self._keys = tuple(self.cfg.keys)

    # ------------------------------------------------------------------ #
    # Robot abstract API                                                  #
    # ------------------------------------------------------------------ #
    @property
    def observation_features(self) -> dict:
        # This robot is a pure network sink; it doesn't produce observations.
        return {}

    @property
    def action_features(self) -> dict[str, type]:
        # Used by teleoperate() for printing and validation
        return {k: float for k in self._keys}

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(
        self, calibrate: bool = True
    ) -> None:  # noqa: ARG002 (calibrate unused)
        # There's no handshake for UDP; mark connected so the higher-level loop can proceed.
        self._connected = True

    @property
    def is_calibrated(self) -> bool:
        # Not applicable for a network sink
        return True

    def calibrate(self) -> None:
        # No-op
        pass

    def configure(self) -> None:
        # No runtime configuration needed
        pass

    def get_observation(self) -> dict[str, Any]:
        # No local sensors
        return {}

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """
        Pack the action values in the configured order and send them via UDP.
        Missing keys default to 0.0, extra keys are ignored.
        """
        # Extract values in the expected order with 0.0 defaults
        values = (
            float(action.get("shoulder_pan.pos", 0.0)),
            float(action.get("shoulder_lift.pos", 0.0)),
            float(action.get("elbow_flex.pos", 0.0)),
            float(action.get("wrist_flex.pos", 0.0)),
            float(action.get("wrist_roll.pos", 0.0)),
            float(action.get("gripper.pos", 0.0)),
        )

        buf = self._PACK(self._seq_next() & 0xFFFFFFFF, *values)
        self._sender.send(buf)
        # Return the action actually “sent” (coerced to floats + filtered to known keys)
        return {k: v for k, v in zip(self._keys, values)}

    def disconnect(self) -> None:
        self._connected = False
        self._sender.close()
