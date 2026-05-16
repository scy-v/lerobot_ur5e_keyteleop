#!/usr/bin/env python

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

import logging
import os
import sys
import time
from queue import Queue
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation as R

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.teleoperators.keyboard.configuration_keyboard import KeyboardEndEffectorTeleopConfig, KeyboardTeleopConfig
from .config_teleop import UR5eTeleopConfig
PYNPUT_AVAILABLE = True
try:
    if ("DISPLAY" not in os.environ) and ("linux" in sys.platform):
        logging.info("No DISPLAY set. Skipping pynput import.")
        raise ImportError("pynput blocked intentionally due to no display.")

    from pynput import keyboard
except ImportError:
    keyboard = None
    PYNPUT_AVAILABLE = False
except Exception as e:
    keyboard = None
    PYNPUT_AVAILABLE = False
    logging.info(f"Could not import pynput: {e}")


class KeyboardTeleop(Teleoperator):
    """
    Teleop class to use keyboard inputs for control.
    """

    config_class = KeyboardTeleopConfig
    name = "keyboard"

    def __init__(self, config: KeyboardTeleopConfig):
        super().__init__(config)
        self.config = config
        self.robot_type = config.type

        self.event_queue = Queue()
        self.current_pressed = {}
        self.listener = None
        self.logs = {}

    @property
    def action_features(self) -> dict:
        return {
            "dtype": "float32",
            "shape": (len(self.arm),),
            "names": {"motors": list(self.arm.motors)},
        }

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return PYNPUT_AVAILABLE and isinstance(self.listener, keyboard.Listener) and self.listener.is_alive()

    @property
    def is_calibrated(self) -> bool:
        pass

    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(
                "Keyboard is already connected. Do not run `robot.connect()` twice."
            )

        if PYNPUT_AVAILABLE:
            logging.info("pynput is available - enabling local keyboard listener.")
            self.listener = keyboard.Listener(
                on_press=self._on_press,
                on_release=self._on_release,
            )
            self.listener.start()
        else:
            logging.info("pynput not available - skipping local keyboard listener.")
            self.listener = None

    def calibrate(self) -> None:
        pass

    def _on_press(self, key):
        self.event_queue.put((key, True))

    def _on_release(self, key):
        if key == keyboard.Key.esc:
            logging.info("ESC pressed, disconnecting.")
            self.disconnect()
        self.event_queue.put((key, False))

    def _drain_pressed_keys(self):
        while not self.event_queue.empty():
            key_char, is_pressed = self.event_queue.get_nowait()
            self.current_pressed[key_char] = is_pressed

    def configure(self):
        pass

    def get_action(self) -> dict[str, Any]:
        before_read_t = time.perf_counter()

        if not self.is_connected:
            raise DeviceNotConnectedError(
                "KeyboardTeleop is not connected. You need to run `connect()` before `get_action()`."
            )

        self._drain_pressed_keys()

        # Generate action based on current key states
        action = {key for key, val in self.current_pressed.items() if val}
        self.logs["read_pos_dt_s"] = time.perf_counter() - before_read_t

        return dict.fromkeys(action, None)

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(
                "KeyboardTeleop is not connected. You need to run `robot.connect()` before `disconnect()`."
            )
        if self.listener is not None:
            self.listener.stop()


class UR5eTeleop(KeyboardTeleop):
    """
    Teleop class to use keyboard inputs for end effector control.
    Designed to be used with the `So100FollowerEndEffector` robot.
    """

    config_class = UR5eTeleopConfig
    name = "ur5e_keyboard_ee"

    def __init__(self, config: UR5eTeleopConfig):
        super().__init__(config)
        self.config = config
        self.step_size = config.step_size
        self.rot_step_size = config.rot_step_size
        self.reference_frame = config.reference_frame
        self.robot = None
        self.gripper_action = 1.0

    def set_robot(self, robot) -> None:
        self.robot = robot

    def _pose_to_matrix(self, tcp_pose: list[float]) -> np.ndarray:
        transform = np.eye(4)
        transform[:3, :3] = R.from_rotvec(tcp_pose[3:]).as_matrix()
        transform[:3, 3] = np.array(tcp_pose[:3], dtype=float)
        return transform

    @property
    def action_features(self) -> dict:
        if self.config.use_gripper:
            return {
                "dtype": "float32",
                "shape": (7,),
                "names": {
                    "delta_x": 0,
                    "delta_y": 1,
                    "delta_z": 2,
                    "delta_rx": 3,
                    "delta_ry": 4,
                    "delta_rz": 5,
                    "gripper_position": 6,
                },
            }
        else:
            return {
                "dtype": "float32",
                "shape": (6,),
                "names": {"delta_x": 0, "delta_y": 1, "delta_z": 2, "delta_rx": 3, "delta_ry": 4, "delta_rz": 5},
            }

    def _to_reference_delta(self, action_values: dict[str, float]) -> dict[str, float]:
        if self.reference_frame == "base":
            return action_values
        if self.reference_frame != "tcp":
            raise ValueError(f"Unsupported reference_frame: {self.reference_frame}")
        if not any(abs(value) > 0.0 for value in action_values.values()):
            return action_values
        if self.robot is None:
            raise ValueError("TCP reference frame requires a robot object on teleop.")

        current_transform = self._pose_to_matrix(self.robot.get_ee_pose())
        current_rotation = current_transform[:3, :3]
        delta_position_base = np.array(
            [action_values["delta_x"], action_values["delta_y"], action_values["delta_z"]],
            dtype=float,
        )
        delta_rotation_base = R.from_euler(
            "xyz",
            [action_values["delta_rx"], action_values["delta_ry"], action_values["delta_rz"]],
        ).as_matrix()

        delta_position_tcp = current_rotation.T @ delta_position_base
        delta_rotation_tcp = current_rotation.T @ delta_rotation_base @ current_rotation
        delta_euler_tcp = R.from_matrix(delta_rotation_tcp).as_euler("xyz")
        return {
            "delta_x": float(delta_position_tcp[0]),
            "delta_y": float(delta_position_tcp[1]),
            "delta_z": float(delta_position_tcp[2]),
            "delta_rx": float(delta_euler_tcp[0]),
            "delta_ry": float(delta_euler_tcp[1]),
            "delta_rz": float(delta_euler_tcp[2]),
        }

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(
                "KeyboardTeleop is not connected. You need to run `connect()` before `get_action()`."
            )

        self._drain_pressed_keys()
        action_values = {axis: 0.0 for axis in ("delta_x", "delta_y", "delta_z", "delta_rx", "delta_ry", "delta_rz")}

        key_mapping = {
            keyboard.KeyCode.from_char("s"): ("delta_x", 1.0, self.step_size),
            keyboard.KeyCode.from_char("w"): ("delta_x", -1.0, self.step_size),
            keyboard.KeyCode.from_char("d"): ("delta_y", 1.0, self.step_size),
            keyboard.KeyCode.from_char("a"): ("delta_y", -1.0, self.step_size),
            keyboard.KeyCode.from_char("q"): ("delta_z", 1.0, self.step_size),
            keyboard.KeyCode.from_char("e"): ("delta_z", -1.0, self.step_size),
            keyboard.KeyCode.from_char("r"): ("delta_rx", 1.0, self.rot_step_size),
            keyboard.KeyCode.from_char("t"): ("delta_rx", -1.0, self.rot_step_size),
            keyboard.KeyCode.from_char("g"): ("delta_ry", 1.0, self.rot_step_size),
            keyboard.KeyCode.from_char("f"): ("delta_ry", -1.0, self.rot_step_size),
            keyboard.KeyCode.from_char("b"): ("delta_rz", 1.0, self.rot_step_size),
            keyboard.KeyCode.from_char("v"): ("delta_rz", -1.0, self.rot_step_size),
        }

        # Generate action from sustained key states. This allows held keys and
        # multi-axis combinations such as forward + left.
        for key, val in self.current_pressed.items():
            if key in key_mapping and val:
                axis, sign, step = key_mapping[key]
                action_values[axis] += sign * step
            elif key == keyboard.KeyCode.from_char("o") and val:
                self.gripper_action = 1         # O -> open, latched like conrft
            elif key == keyboard.KeyCode.from_char("l") and val:
                self.gripper_action = 0         # L -> close, latched like conrft

        action_dict = self._to_reference_delta(action_values)

        if self.config.use_gripper:
            action_dict["gripper_position"] = self.gripper_action

        return action_dict
