import os
import select
import sys
import termios
import time
import tty
from numbers import Real

import rclpy
from geometry_msgs.msg import Twist
from mors_ros_msgs.msg import GaitParams
from mors_ros_msgs.srv import RobotCmd
from rclpy.node import Node


DO_NOTHING_MODE = 0
LOCOMOTION_MODE = 1
STANDING_MODE = 3

NO_ACTION = 0
STANDUP = 1
LAY_DOWN = 2

BODY_Z_MAX = 0.26
BODY_Z_MIN = 0.1

KEYBOARD_STRIDE_HEIGHT = 0.12
KEYBOARD_STRIDE_HEIGHT_STEP = 0.005
KEYBOARD_STRIDE_HEIGHT_MIN = 0.0
KEYBOARD_T_SW = 0.21
KEYBOARD_T_SW_STEP = 0.01
KEYBOARD_T_SW_MIN = 0.01
KEYBOARD_T_ST_MAX = 0.5
KEYBOARD_T_ST_MIN = 0.2
KEYBOARD_GAIT_TYPE = [0.0, 0.5, 0.5, 0.0]
KEYBOARD_BODY_Z = 0.2
KEYBOARD_BODY_Z_STEP = 0.002
KEYBOARD_DEFAULT_SPEED = 0.3
KEYBOARD_MIN_SPEED = 0.1
KEYBOARD_MAX_SPEED = 0.9
KEYBOARD_COMMAND_TIMEOUT = 0.4
KEYBOARD_TOGGLE_DEBOUNCE = 0.3

FLOAT_EPS = 1e-6
CMD_POSE_LPF_ALPHA = 0.2


class MorsKeyboardControl(Node):
    def __init__(self):
        super().__init__("mors_keyboard_control")

        self.mode_cli = self.create_client(RobotCmd, "robot_mode")
        self.req = RobotCmd.Request()

        self.action_cli = self.create_client(RobotCmd, "robot_action")

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.cmd_vel_msg = Twist()
        self.cmd_pose_pub = self.create_publisher(Twist, "cmd_pose", 10)
        self.cmd_pose_msg = Twist()
        self.gait_params_pub = self.create_publisher(GaitParams, "gait_params", 10)
        self.gait_params_msg = GaitParams()
        self.keyboard_body_z = KEYBOARD_BODY_Z
        self.keyboard_stride_height = KEYBOARD_STRIDE_HEIGHT
        self.keyboard_t_sw = KEYBOARD_T_SW
        self.cmd_pose_msg.linear.z = self.keyboard_body_z
        self.gait_params_msg.standing = True
        self.gait_params_msg.stride_height = self.keyboard_stride_height
        self.keyboard_speed = KEYBOARD_DEFAULT_SPEED
        self.gait_params_msg.t_st = self._t_st_for_speed(self.keyboard_speed)
        self.gait_params_msg.t_sw = self.keyboard_t_sw
        self.gait_params_msg.gait_offsets = KEYBOARD_GAIT_TYPE

        self.prev_cmd_vel_state = None
        self.prev_cmd_pose_state = None
        self.prev_gait_params_state = None
        self.filtered_cmd_pose_state = None

        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.is_sleeping = True
        self.robot_mode = DO_NOTHING_MODE
        self.prev_robot_mode = DO_NOTHING_MODE
        self.action_num = NO_ACTION

        self.keyboard_linear_dir = 0.0
        self.keyboard_linear_until = 0.0
        self.keyboard_lateral_dir = 0.0
        self.keyboard_lateral_until = 0.0
        self.keyboard_angular_dir = 0.0
        self.keyboard_angular_until = 0.0
        self.keyboard_toggle_requested = False
        self.keyboard_last_toggle_time = 0.0
        self.keyboard_mode_toggle_requested = False
        self.keyboard_mode_last_toggle_time = 0.0
        self.keyboard_escape_sequence = ""
        self.keyboard_fd = None
        self.keyboard_term_settings = None
        self.stdout_is_tty = sys.stdout.isatty()
        self.prev_status_lines = None
        self.ui_initialized = False

        self._setup_keyboard_input()
        self._initialize_ui()

    def _supports_color(self):
        return self.stdout_is_tty

    def _style(self, text: str, *codes: str) -> str:
        if not self._supports_color() or not codes:
            return text

        prefix = "".join(f"\x1b[{code}m" for code in codes)
        return f"{prefix}{text}\x1b[0m"

    def _section_title(self, title: str) -> str:
        return self._style(title, "1", "96")

    def _section_rule(self) -> str:
        return self._style("=" * 40, "90")

    def _subsection_rule(self) -> str:
        return self._style("---", "90")

    def _key_label(self, text: str) -> str:
        return self._style(text, "93")

    def _status_label(self, text: str) -> str:
        return self._style(text, "32")

    def _setup_keyboard_input(self):
        if not sys.stdin.isatty():
            return

        self.keyboard_fd = sys.stdin.fileno()
        try:
            self.keyboard_term_settings = termios.tcgetattr(self.keyboard_fd)
            tty.setcbreak(self.keyboard_fd)
        except termios.error:
            self.keyboard_fd = None
            self.keyboard_term_settings = None

    def _status_lines(self):
        return [
            f"{self._status_label('mode')}: {self.robot_mode_name()}",
            f"{self._status_label('on legs')}: {'true' if not self.is_sleeping else 'false'}",
            f"{self._status_label('v_x max')}: {self.keyboard_speed:.1f}",
            f"{self._status_label('v_z max')}: {self.keyboard_speed:.1f}",
            f"{self._status_label('robot height')}: {self.keyboard_body_z:.3f}",
            f"{self._status_label('stride height')}: {self.keyboard_stride_height:.3f}",
            f"{self._status_label('t_sw')}: {self.keyboard_t_sw:.2f}",
            f"{self._status_label('t_st')}: {self._t_st_for_speed(self.keyboard_speed):.4f}",
        ]

    def _write_plain(self, text: str):
        sys.stdout.write(text)
        sys.stdout.flush()

    def _initialize_ui(self):
        status_lines = self._status_lines()
        ui_lines = [
            self._section_rule(),
            self._section_title("Greeting"),
            "Hello! This is mors_keyboard_control.",
            self._section_rule(),
            self._section_title("Control"),
            self._subsection_rule(),
            f"{self._key_label('Enter')}: stand up / lay down",
            f"{self._key_label('Space')}: toggle STANDING_MODE / LOCOMOTION_MODE",
            self._subsection_rule(),
            f"{self._key_label('W/S')}: move forward/backward",
            f"{self._key_label('Q/E')}: move left/right",
            f"{self._key_label('A/D')}: rotate CCW/CW",
            self._subsection_rule(),
            f"{self._key_label('Up')}: increase body height Z by 0.002 m",
            f"{self._key_label('Down')}: decrease body height Z by 0.002 m",
            f"{self._key_label('Ctrl+Up')}: increase t_sw by 0.01 s",
            f"{self._key_label('Ctrl+Down')}: decrease t_sw by 0.01 s",
            f"{self._key_label('Shift+Up')}: increase stride height by 0.005 m",
            f"{self._key_label('Shift+Down')}: decrease stride height by 0.005 m",
            self._subsection_rule(),
            f"{self._key_label('1..9')}: set speed to 0.1 .. 0.9",
            self._section_rule(),
            self._section_title("Status"),
            *status_lines,
            self._section_rule(),
        ]

        if self.stdout_is_tty:
            self._write_plain("\x1b[2J\x1b[H\x1b[?25l")

        self._write_plain("\n".join(ui_lines) + "\n")
        self.prev_status_lines = status_lines
        self.ui_initialized = True

    def _render_status(self, force: bool = False):
        status_lines = self._status_lines()

        if not self.ui_initialized:
            self._initialize_ui()
            return

        if not force and status_lines == self.prev_status_lines:
            return

        if not self.stdout_is_tty:
            self.prev_status_lines = status_lines
            return

        self._write_plain(f"\x1b[{len(status_lines) + 1}F")
        for line in status_lines:
            self._write_plain(f"\x1b[2K{line}\n")
        self._write_plain(f"\x1b[2K{self._section_rule()}\n")

        self.prev_status_lines = status_lines

    def _t_st_for_speed(self, speed: float) -> float:
        clamped_speed = min(max(speed, KEYBOARD_MIN_SPEED), KEYBOARD_MAX_SPEED)
        speed_span = KEYBOARD_MAX_SPEED - KEYBOARD_MIN_SPEED
        if speed_span <= FLOAT_EPS:
            return KEYBOARD_T_ST_MAX

        speed_ratio = (clamped_speed - KEYBOARD_MIN_SPEED) / speed_span
        return KEYBOARD_T_ST_MAX + speed_ratio * (KEYBOARD_T_ST_MIN - KEYBOARD_T_ST_MAX)

    def robot_mode_name(self) -> str:
        if self.robot_mode == DO_NOTHING_MODE:
            return "DO_NOTHING_MODE"
        if self.robot_mode == LOCOMOTION_MODE:
            return "LOCOMOTION_MODE"
        if self.robot_mode == STANDING_MODE:
            return "STANDING_MODE"
        return str(self.robot_mode)

    def _adjust_keyboard_body_z(self, delta_z: float):
        new_z = min(max(self.keyboard_body_z + delta_z, BODY_Z_MIN), BODY_Z_MAX)
        if abs(new_z - self.keyboard_body_z) <= FLOAT_EPS:
            return

        self.keyboard_body_z = new_z

    def _adjust_keyboard_t_sw(self, delta_t_sw: float):
        new_t_sw = max(self.keyboard_t_sw + delta_t_sw, KEYBOARD_T_SW_MIN)
        if abs(new_t_sw - self.keyboard_t_sw) <= FLOAT_EPS:
            return

        self.keyboard_t_sw = new_t_sw

    def _adjust_keyboard_stride_height(self, delta_height: float):
        new_stride_height = max(
            self.keyboard_stride_height + delta_height,
            KEYBOARD_STRIDE_HEIGHT_MIN,
        )
        if abs(new_stride_height - self.keyboard_stride_height) <= FLOAT_EPS:
            return

        self.keyboard_stride_height = new_stride_height

    def _handle_escape_sequence(self, sequence: str):
        if len(sequence) < 3 or not sequence.startswith("\x1b["):
            return

        final_key = sequence[-1]
        if final_key not in ("A", "B"):
            return

        params = sequence[2:-1]
        modifier_code = 1
        if ";" in params:
            parts = [part for part in params.split(";") if part]
            if parts:
                try:
                    modifier_code = int(parts[-1])
                except ValueError:
                    modifier_code = 1

        has_shift = modifier_code in (2, 4, 6, 8)
        has_ctrl = modifier_code in (5, 6, 7, 8)

        direction = 1.0 if final_key == "A" else -1.0

        if has_ctrl:
            self._adjust_keyboard_t_sw(direction * KEYBOARD_T_SW_STEP)
            return

        if has_shift:
            self._adjust_keyboard_stride_height(
                direction * KEYBOARD_STRIDE_HEIGHT_STEP
            )
            return

        self._adjust_keyboard_body_z(direction * KEYBOARD_BODY_Z_STEP)

    def _restore_keyboard_input(self):
        if self.keyboard_fd is None or self.keyboard_term_settings is None:
            return

        try:
            termios.tcsetattr(
                self.keyboard_fd,
                termios.TCSADRAIN,
                self.keyboard_term_settings,
            )
        except termios.error:
            pass
        finally:
            self.keyboard_fd = None
            self.keyboard_term_settings = None

    def destroy_node(self):
        self._restore_keyboard_input()
        if self.stdout_is_tty:
            self._write_plain("\x1b[?25h\n")
        return super().destroy_node()

    def _values_changed(self, prev, current):
        if prev is None:
            return True

        if isinstance(current, tuple):
            return any(self._values_changed(p, c) for p, c in zip(prev, current))

        if isinstance(current, list):
            if len(prev) != len(current):
                return True
            return any(self._values_changed(p, c) for p, c in zip(prev, current))

        if isinstance(current, Real) and not isinstance(current, bool):
            return abs(prev - current) > FLOAT_EPS

        return prev != current

    def _twist_state(self, msg: Twist):
        return (
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        )

    def _gait_params_state(self, msg: GaitParams):
        return (
            msg.standing,
            msg.stride_height,
            msg.t_st,
            msg.t_sw,
            tuple(msg.gait_offsets),
        )

    def _set_twist_state(self, msg: Twist, state):
        msg.linear.x = state[0]
        msg.linear.y = state[1]
        msg.linear.z = state[2]
        msg.angular.x = state[3]
        msg.angular.y = state[4]
        msg.angular.z = state[5]

    def _apply_cmd_pose_low_pass(self):
        target_state = self._twist_state(self.cmd_pose_msg)

        if self.filtered_cmd_pose_state is None:
            self.filtered_cmd_pose_state = target_state
        else:
            self.filtered_cmd_pose_state = tuple(
                prev + CMD_POSE_LPF_ALPHA * (target - prev)
                for prev, target in zip(self.filtered_cmd_pose_state, target_state)
            )

        self._set_twist_state(self.cmd_pose_msg, self.filtered_cmd_pose_state)

    def _publish_if_changed(self, publisher, msg, state_getter, prev_state_attr: str):
        current_state = state_getter(msg)
        prev_state = getattr(self, prev_state_attr)

        if self._values_changed(prev_state, current_state):
            publisher.publish(msg)
            setattr(self, prev_state_attr, current_state)

    def _poll_keyboard(self):
        if self.keyboard_fd is None:
            return

        while True:
            ready, _, _ = select.select([self.keyboard_fd], [], [], 0.0)
            if not ready:
                break

            try:
                key = os.read(self.keyboard_fd, 1)
            except OSError:
                break

            if not key:
                break

            self._handle_keyboard_key(key.decode(errors="ignore"))

    def _handle_keyboard_key(self, key: str):
        if not key:
            return

        if self.keyboard_escape_sequence:
            if self.keyboard_escape_sequence == "\x1b" and key != "[":
                self.keyboard_escape_sequence = ""
            else:
                self.keyboard_escape_sequence += key
                if key.isalpha() or key == "~":
                    self._handle_escape_sequence(self.keyboard_escape_sequence)
                    self.keyboard_escape_sequence = ""
                return

        if key == "\x1b":
            self.keyboard_escape_sequence += key
            return

        now = time.monotonic()

        if key in ("\r", "\n"):
            if now - self.keyboard_last_toggle_time > KEYBOARD_TOGGLE_DEBOUNCE:
                self.keyboard_toggle_requested = True
                self.keyboard_last_toggle_time = now
            return

        if key == " ":
            if now - self.keyboard_mode_last_toggle_time > KEYBOARD_TOGGLE_DEBOUNCE:
                self.keyboard_mode_toggle_requested = True
                self.keyboard_mode_last_toggle_time = now
            return

        key = key.lower()

        if key == "w":
            self.keyboard_linear_dir = 1.0
            self.keyboard_linear_until = now + KEYBOARD_COMMAND_TIMEOUT
        elif key == "s":
            self.keyboard_linear_dir = -1.0
            self.keyboard_linear_until = now + KEYBOARD_COMMAND_TIMEOUT
        elif key == "q":
            self.keyboard_lateral_dir = 1.0
            self.keyboard_lateral_until = now + KEYBOARD_COMMAND_TIMEOUT
        elif key == "e":
            self.keyboard_lateral_dir = -1.0
            self.keyboard_lateral_until = now + KEYBOARD_COMMAND_TIMEOUT
        elif key == "a":
            self.keyboard_angular_dir = 1.0
            self.keyboard_angular_until = now + KEYBOARD_COMMAND_TIMEOUT
        elif key == "d":
            self.keyboard_angular_dir = -1.0
            self.keyboard_angular_until = now + KEYBOARD_COMMAND_TIMEOUT
        elif key in "123456789":
            self.keyboard_speed = int(key) / 10.0

    def send_mode_request(self, mode: int):
        self.req.data = mode
        resp = self.mode_cli.call_async(self.req)
        return resp.result()

    def send_action_request(self, mode: int):
        self.req.data = mode
        resp = self.action_cli.call_async(self.req)
        return resp.result()

    def _handle_keyboard_control(self):
        self._poll_keyboard()

        if self.keyboard_toggle_requested:
            self.keyboard_toggle_requested = False
            if self.is_sleeping:
                self.action_num = STANDUP
                self.send_action_request(self.action_num)
                self.is_sleeping = False
                self.robot_mode = STANDING_MODE
            else:
                self.action_num = LAY_DOWN
                self.send_action_request(self.action_num)
                self.is_sleeping = True
                self.robot_mode = DO_NOTHING_MODE

        if (
            self.keyboard_mode_toggle_requested
            and not self.is_sleeping
            and self.robot_mode in (STANDING_MODE, LOCOMOTION_MODE)
        ):
            self.keyboard_mode_toggle_requested = False
            if self.robot_mode == STANDING_MODE:
                self.robot_mode = LOCOMOTION_MODE
            else:
                self.robot_mode = STANDING_MODE
        else:
            self.keyboard_mode_toggle_requested = False

        if self.is_sleeping:
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.linear.y = 0.0
            self.cmd_vel_msg.linear.z = 0.0
            self.cmd_vel_msg.angular.x = 0.0
            self.cmd_vel_msg.angular.y = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_pose_msg.linear.z = self.keyboard_body_z
            self.cmd_pose_msg.angular.x = 0.0
            self.cmd_pose_msg.angular.y = 0.0
            self.cmd_pose_msg.angular.z = 0.0
            self.gait_params_msg.standing = True
            return

        now = time.monotonic()
        linear_dir = self.keyboard_linear_dir if now <= self.keyboard_linear_until else 0.0
        lateral_dir = (
            self.keyboard_lateral_dir if now <= self.keyboard_lateral_until else 0.0
        )
        angular_dir = (
            self.keyboard_angular_dir if now <= self.keyboard_angular_until else 0.0
        )

        if linear_dir == 0.0:
            self.keyboard_linear_dir = 0.0
        if lateral_dir == 0.0:
            self.keyboard_lateral_dir = 0.0
        if angular_dir == 0.0:
            self.keyboard_angular_dir = 0.0

        self.cmd_pose_msg.linear.z = self.keyboard_body_z
        self.cmd_pose_msg.angular.x = 0.0
        self.cmd_pose_msg.angular.y = 0.0
        self.cmd_pose_msg.angular.z = 0.0

        if self.robot_mode == STANDING_MODE:
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.linear.y = 0.0
            self.cmd_vel_msg.linear.z = 0.0
            self.cmd_vel_msg.angular.x = 0.0
            self.cmd_vel_msg.angular.y = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.gait_params_msg.standing = True
            return

        self.robot_mode = LOCOMOTION_MODE
        self.gait_params_msg.standing = False
        self.gait_params_msg.stride_height = self.keyboard_stride_height
        self.gait_params_msg.t_st = self._t_st_for_speed(self.keyboard_speed)
        self.gait_params_msg.t_sw = self.keyboard_t_sw
        self.gait_params_msg.gait_offsets = KEYBOARD_GAIT_TYPE

        self.cmd_vel_msg.linear.x = linear_dir * self.keyboard_speed
        self.cmd_vel_msg.linear.y = lateral_dir * self.keyboard_speed * 0.5
        self.cmd_vel_msg.linear.z = 0.0
        self.cmd_vel_msg.angular.x = 0.0
        self.cmd_vel_msg.angular.y = 0.0
        self.cmd_vel_msg.angular.z = angular_dir * self.keyboard_speed

    def timer_callback(self):
        self._handle_keyboard_control()

        if self.prev_robot_mode != self.robot_mode:
            self.send_mode_request(self.robot_mode)

        if self.robot_mode != DO_NOTHING_MODE:
            self._apply_cmd_pose_low_pass()

        self._publish_if_changed(
            self.cmd_vel_pub,
            self.cmd_vel_msg,
            self._twist_state,
            "prev_cmd_vel_state",
        )
        self._publish_if_changed(
            self.cmd_pose_pub,
            self.cmd_pose_msg,
            self._twist_state,
            "prev_cmd_pose_state",
        )
        self._publish_if_changed(
            self.gait_params_pub,
            self.gait_params_msg,
            self._gait_params_state,
            "prev_gait_params_state",
        )

        self.prev_robot_mode = self.robot_mode
        self._render_status()


def main(args=None):
    rclpy.init(args=args)

    keyboard_control = MorsKeyboardControl()

    try:
        rclpy.spin(keyboard_control)
    finally:
        keyboard_control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
