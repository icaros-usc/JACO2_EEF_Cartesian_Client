"""OpenPI WebSocket transport; never call ROS from the inference thread."""

import time
from threading import Lock

import numpy as np
import websocket
from openpi_client import msgpack_numpy
from PIL import Image

from .core import finite_vector, parse_actions


def image_rgb(msg):
    """Return a Pillow RGB image at the ROS Image's original width and height.

    Respect row padding and support rgb8, bgr8, rgba8, bgra8 and mono8.
    Reject inconsistent dimensions or byte counts before decoding.
    """
    modes = {
        "rgb8": ("RGB", "RGB", 3),
        "bgr8": ("RGB", "BGR", 3),
        "rgba8": ("RGBA", "RGBA", 4),
        "bgra8": ("RGBA", "BGRA", 4),
        "mono8": ("L", "L", 1),
    }
    if msg.encoding not in modes:
        raise ValueError(f"Unsupported image encoding {msg.encoding}")
    mode, raw_mode, channel_count = modes[msg.encoding]
    row_bytes = msg.width * channel_count
    if msg.width < 1 or msg.height < 1 or msg.step < row_bytes:
        raise ValueError("Invalid image dimensions or step")
    data = bytes(msg.data)
    if len(data) != msg.height * msg.step:
        raise ValueError("Image byte count does not match height*step")
    return Image.frombytes(
        mode, (msg.width, msg.height), data, "raw", raw_mode, msg.step, 1
    ).convert("RGB")


class PolicyConnection:
    """One lazy OpenPI connection owned by the inference worker.

    infer() calls are serial. close() may also be called from the ROS thread
    to interrupt blocked socket I/O. A closed connection requires a client restart.
    """

    def __init__(self, url, timeout):
        if not url.startswith(("ws://", "wss://")):
            raise ValueError("server_url must use ws:// or wss://")
        self.url, self.timeout = url, timeout
        self.connection = None
        self.embedding = None
        self.closed = False
        self._close_lock = Lock()

    @staticmethod
    def _remaining(deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("Policy inference deadline exceeded")
        return remaining

    def _receive(self, deadline):
        self.connection.settimeout(self._remaining(deadline))
        message = self.connection.recv()
        if isinstance(message, str):
            raise RuntimeError(f"Policy server error: {message}")
        if len(message) > 1024 * 1024:
            raise ValueError("Policy response exceeds 1 MiB")
        return msgpack_numpy.unpackb(message)

    def infer(self, overview, wrist, state, instruction):
        """Send RGB images and world-frame state (8,); return actions (N, 7).

        Images are uint8 HWC at capture resolution; preprocessing belongs to the
        server. State is float32 XYZ, rotation vector, and two proximal angles.
        Returned translation and rotation deltas use world axes.
        An optional response embedding is exposed as self.embedding.
        """
        if self.closed:
            raise RuntimeError("Policy connection is closed")
        deadline = time.monotonic() + self.timeout
        payload = {
            "observation/image": np.asarray(image_rgb(overview)),
            "observation/wrist_image": np.asarray(image_rgb(wrist)),
            "observation/state": finite_vector(state, 8).astype(np.float32),
            "prompt": instruction,
        }
        try:
            if self.connection is None:
                self.connection = websocket.create_connection(
                    self.url,
                    timeout=self._remaining(deadline),
                    enable_multithread=True,
                    redirect_limit=0,
                )
                # A stop may arrive during the bounded connection handshake.
                if self.closed:
                    raise RuntimeError("Policy connection is closed")
                # serve_policy sends metadata before the first inference response.
                if not isinstance(self._receive(deadline), dict):
                    raise ValueError("Expected OpenPI server metadata")
            self.connection.settimeout(self._remaining(deadline))
            self.connection.send_binary(msgpack_numpy.packb(payload))
            response = self._receive(deadline)
            actions = parse_actions(response)
            self.embedding = response.get("embedding")
            return actions
        except Exception:
            # Never reuse a timed-out socket: its next frame may be a stale response.
            self.close()
            raise

    def close(self):
        """Interrupt pending reads/writes and close without waiting for the server."""
        with self._close_lock:
            self.closed = True
            if self.connection is not None:
                self.connection.abort()
                self.connection.shutdown()
