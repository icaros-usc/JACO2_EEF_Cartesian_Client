#!/usr/bin/env python3
"""Serve Nintendo controller actions using the OpenPI WebSocket protocol."""

import argparse
import asyncio
import os

import numpy as np
from openpi_client import msgpack_numpy
from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed


def controller_action(axes, rotation_mode, grip):
    """Map four stick axes to a world-frame policy action [XYZ, rotation, grip].

    Deltas use policy units; the ROS client applies its configured scales and
    frame conversion. Mapping matches collect_jaco_trajs.py and the dev teleop.
    """
    axes = np.asarray(axes, dtype=float)
    if axes.shape != (4,) or not np.isfinite(axes).all() or np.any(np.abs(axes) > 1):
        raise ValueError("Expected four finite controller axes in [-1, 1]")
    lx, ly, rx, ry = np.where(np.abs(axes) > 0.08, axes, 0.0)
    if rotation_mode:
        return np.array([0, 0, 0, -0.15 * rx, -0.15 * ly, -0.15 * lx, grip])
    return np.array([-0.3 * lx, 0.3 * ly, -0.3 * ry, 0, 0, 0, grip])


class StopRequested(Exception):
    """The operator pressed Minus."""


class Controller:
    """Read the Nintendo sticks and latch the rotation mode and gripper state."""

    def __init__(self, index=0):
        # SDL still needs a display/event loop when running from a terminal.
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
        os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
        import pygame

        self.pygame = pygame
        pygame.display.init()
        pygame.display.set_mode((1, 1))
        pygame.joystick.init()
        if not 0 <= index < pygame.joystick.get_count():
            pygame.quit()
            raise RuntimeError("No controller at that index; check the USB connection")
        self.joystick = pygame.joystick.Joystick(index)
        if self.joystick.get_numaxes() < 4 or self.joystick.get_numbuttons() < 10:
            pygame.quit()
            raise RuntimeError("Controller does not expose the expected axes/buttons")
        self.rotation_mode = False
        self.grip = -1.0
        print(f"Controller: {self.joystick.get_name()}", flush=True)
        print(
            "ZL: translation/rotation | ZR: open/close | Minus or Ctrl-C: stop",
            flush=True,
        )

    def sample(self):
        """Return the latest action (7,), including latched -1/open or +1/close."""
        pygame = self.pygame
        for event in pygame.event.get():
            if (
                event.type == pygame.JOYDEVICEREMOVED
                and event.instance_id == self.joystick.get_instance_id()
            ):
                raise RuntimeError("Controller disconnected")
            if (
                event.type == pygame.JOYBUTTONDOWN
                and event.instance_id == self.joystick.get_instance_id()
            ):
                if event.button == 8:
                    raise StopRequested
                if event.button == 6:
                    self.rotation_mode = not self.rotation_mode
                if event.button == 7:
                    self.grip = -self.grip
        axes = [self.joystick.get_axis(i) for i in range(4)]
        return controller_action(axes, self.rotation_mode, self.grip)

    def close(self):
        self.pygame.quit()


async def serve_controller(controller, host="127.0.0.1", port=8011):
    """Return one current action per observation request; close sockets on stop.

    Observations are ignored. Polling continues without requests so disconnects
    and the stop button close existing connections even while the client is idle.
    """
    action = controller.sample()

    async def handle(connection):
        try:
            await connection.send(msgpack_numpy.packb({"source": "nintendo"}))
            async for _ in connection:
                await connection.send(
                    msgpack_numpy.packb(
                        {"actions": np.asarray(action, dtype=np.float32)[None, :]}
                    )
                )
        except ConnectionClosed:
            pass

    async with serve(
        handle, host, port, compression=None, max_size=None, close_timeout=1
    ):
        print(
            f"Nintendo policy server: ws://{host}:{port} (set replan_steps: 1 in control.yaml)",
            flush=True,
        )
        while True:
            action = controller.sample()
            await asyncio.sleep(0.01)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8011, help="Local WebSocket port")
    parser.add_argument("--controller", type=int, default=0, help="SDL joystick index")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    controller = None
    try:
        controller = Controller(args.controller)
        asyncio.run(serve_controller(controller, port=args.port))
    except (KeyboardInterrupt, StopRequested):
        print("Nintendo policy server stopped", flush=True)
    except (OSError, RuntimeError) as exc:
        parser.exit(1, f"Nintendo policy server: {exc}\n")
    finally:
        if controller is not None:
            controller.close()


if __name__ == "__main__":
    main()
