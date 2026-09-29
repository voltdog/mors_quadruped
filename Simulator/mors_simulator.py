import os
import sys
import time
import warnings
import zlib
from pathlib import Path
from threading import Event, Thread

import lcm
import glfw

BASE_DIR = Path(__file__).resolve().parent
LCM_MSG_ROOT = BASE_DIR.parent / "lcm_msgs"
sys.path.insert(0, str(BASE_DIR))
sys.path.insert(0, str(LCM_MSG_ROOT))

if os.environ.get("XDG_SESSION_TYPE", "").lower() == "wayland":
    warnings.filterwarnings(
        "ignore",
        message=r".*Wayland: The platform does not provide the window position.*",
        category=glfw.GLFWError,
    )

from mors_msgs.servo_cmd_msg import servo_cmd_msg
from mors_msgs.servo_state_msg import servo_state_msg 
from mors_msgs.imu_lcm_data import imu_lcm_data
from mors_msgs.robot_state_msg import robot_state_msg
from mors_msgs.contact_sensor_msg import contact_sensor_msg
from mors_msgs.odometry_msg import odometry_msg
from mors_msgs.depth_image_msg import depth_image_msg

from additional.mors_env import MorsMujocoEnv

import yaml
import numpy as np
import math
from scipy.spatial.transform import Rotation

class Hardware_Level_Sim():
    DEPTH_IMAGE_DEFAULT_FPS = 10
    DEPTH_IMAGE_DEFAULT_SIZE = [424, 240]
    DEPTH_IMAGE_ALLOWED_SIZES = {(424, 240), (640, 480)}

    def __init__(self):
        self.shutdown_event = Event()
        self.ref_joint_pos = [0]*12
        self.ref_joint_vel = [0]*12
        self.ref_joint_torq = [0]*12
        self.kp = [0]*12
        self.kd = [0]*12
        self.cur_joint_pos = [0]*12
        self.cur_joint_vel = [0]*12
        self.cur_joint_torq = [0]*12

        self.read_config()

        # init LCM thread for commands
        self.cmd_th = Thread(target=self.get_cmd, args=())
        self.cmd_th.daemon = True
        self.cmd_th.start()

        # init LCM for states
        self.servo_state_msg = servo_state_msg()
        self.lcm_imu_msg = imu_lcm_data()
        self.lcm_robot_state_msg = robot_state_msg()
        self.lcm_odom_msg = odometry_msg()
        self.lcm_contact_sensor_msg = contact_sensor_msg()
        if self.depth_image_enabled:
            self.lcm_depth_image_msg = depth_image_msg()

        self.lc_servo_state = self._create_lcm(self.lcm_servo_url)
        self.lc_imu = self._create_lcm(self.lcm_locomotion_url)
        self.lc_robot_state = self._create_lcm(self.lcm_locomotion_url)
        self.lc_odom = self._create_lcm(self.lcm_locomotion_url)
        self.lc_contact = self._create_lcm(self.lcm_locomotion_url)
        if self.depth_image_enabled:
            self.lc_depth_image = self._create_lcm(self.lcm_locomotion_url)
        
        self.lcm_imu_msg.orientation_covariance = [2.603e-07, 0.0, 0.0, 0.0, 2.603e-07, 0.0, 0.0, 0.0, 0.0]
        self.lcm_imu_msg.angular_velocity_covariance = [2.5e-05, 0.0, 0.0, 0.0, 2.5e-05, 0.0, 0.0, 0.0, 2.5e-05]
        self.lcm_imu_msg.linear_acceleration_covariance = [2.5e-05, 0.0, 0.0, 0.0, 2.5e-05, 0.0, 0.0, 0.0, 2.5e-05]
        
        self.sim_it = 0
        self.init_simulation()
        if self.depth_image_enabled:
            self.depth_image_th = Thread(target=self.depth_image_loop, args=())
            self.depth_image_th.daemon = True
            self.depth_image_th.start()

        # init variables
        self.body_quaternion = [0.0, 0.0, 0.0, 1.0]
        self.body_lin_pos = [0.0]*3
        self.body_ang_vel = [0.0]*3
        self.body_lin_vel = np.array([0]*3, float)
        self.body_lin_acc = np.array([0]*3, float)
        self.force_dir = 1

        self.imu_data = [0]*13

        self.leg_data_transform = [ 1,  1,  1, 
                                    1,  1,  1,
                                    1,  1,  1,
                                    1,  1,  1]
        self.leg_data_order = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]

        # yaw transformation stuff
        self.yaw = 0.0
        self.pre_yaw = 0.0
        self.first_yaw = True
        self.offset_yaw = 0.0
        self.yaw_final = 0.0

    def read_config(self):
        with open(f"{BASE_DIR}/../config/simulation.yaml", "r") as f:
            sim_config = yaml.safe_load(f)

        self.sim_freq = sim_config.get("frequency", 500)
        self.mjcf_root = str((BASE_DIR / sim_config.get("mjcf_root", "MJCF/mors.xml")).resolve())
        self.render_quality = str(sim_config.get("render_quality", "high")).strip().lower()
        self.scene = self._parse_scene_config(sim_config.get("scene", None))
        self.init_motor_angles = sim_config.get("init_motor_angles", [0.0, -1.57, 3.14,
                                                                      -0.0, 1.57, -3.14,
                                                                      -0.0, -1.57, 3.14,
                                                                      0.0, 1.57, -3.14])
        self.foot_contacts_enabled = sim_config.get("foot_contacts", True)
        self.full_state_enabled = sim_config.get("full_state", False)
        self.lcm_odometry_enabled = sim_config.get("odometry", True)
        self.foot_positions_type = sim_config.get("foot_positions_type", "local") # "global" or "local"

        self.external_disturbance_enabled = sim_config.get("external_disturbance", False)
        self.external_disturbance_value = sim_config.get("external_disturbance_value", 4000)
        self.external_disturbance_duration = sim_config.get("external_disturbance_duration", 0.02)
        self.external_disturbance_interval = sim_config.get("external_disturbance_interval", 2)

        self.joint_dir = sim_config.get("joint_dir", [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
        self.joint_offset = sim_config.get("joint_offset", [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

        # параметры приводов общие с реальным роботом
        with open(f"{BASE_DIR}/../config/robot.yaml", "r") as f:
            robot_config = yaml.safe_load(f)["robot_config"]

        self.vel_max = robot_config["vel_max"]
        self.tau_max = robot_config["tau_max"]
        self.gear_ratio = robot_config["gear_ratio"]
        self.kt = robot_config["kt"]

        depth_image_enabled = sim_config.get("depth_image", False)
        if not isinstance(depth_image_enabled, bool):
            raise ValueError(
                f"Invalid `depth_image`: expected boolean True/False, got {depth_image_enabled!r}"
            )

        self.depth_image_enabled = depth_image_enabled
        self.depth_image_fps = self.DEPTH_IMAGE_DEFAULT_FPS
        self.depth_image_size = self.DEPTH_IMAGE_DEFAULT_SIZE[:]
        if self.depth_image_enabled:
            # симулируемая камера повторяет поток реальной D435i
            with open(f"{BASE_DIR}/../config/sensors.yaml", "r") as f:
                d435i_stream = yaml.safe_load(f)["realsense_camera_d435i"]["stream"]
            depth_image_fps = d435i_stream["fps"]
            depth_image_size = [d435i_stream["width"], d435i_stream["height"]]
            self._validate_depth_config(depth_image_fps, depth_image_size)
            self.depth_image_fps = depth_image_fps
            self.depth_image_size = [depth_image_size[0], depth_image_size[1]]
            self.depth_image_period = 1.0 / float(self.depth_image_fps)

        with open(f"{BASE_DIR}/../config/channels.yaml", "r") as f:
            lcm_config = yaml.safe_load(f)

        self.lcm_servo_cmd_channel = lcm_config["servo_cmd"]
        self.lcm_servo_state_channel = lcm_config["servo_state"]
        self.lcm_imu_channel = lcm_config["imu_data"]
        self.lcm_odom_channel = lcm_config["odometry"]
        self.lcm_contact_sensor_channel = lcm_config["contact_state"]
        self.lcm_robot_state_channel = lcm_config["robot_state"]
        self.depth_image_channel = lcm_config["depth_image"]

        self.lcm_locomotion_url = os.environ.get("LCM_LOCOMOTION_URL")
        self.lcm_servo_url = os.environ.get("LCM_SERVO_URL")

        self.sim_period = 1.0/self.sim_freq
        self.first_step = True

    @staticmethod
    def _create_lcm(url):
        return lcm.LCM(url) if url else lcm.LCM()

    def _validate_depth_config(self, depth_image_fps, depth_image_size):
        if isinstance(depth_image_fps, bool) or not isinstance(depth_image_fps, int):
            raise ValueError(
                f"Invalid `depth_image_fps`: expected positive integer, got {depth_image_fps!r}"
            )
        if depth_image_fps <= 0:
            raise ValueError(
                f"Invalid `depth_image_fps`: expected > 0, got {depth_image_fps}"
            )

        if not isinstance(depth_image_size, list) or len(depth_image_size) != 2:
            raise ValueError(
                "Invalid `depth_image_size`: expected list of two integers [width, height]"
            )
        if any((isinstance(v, bool) or not isinstance(v, int)) for v in depth_image_size):
            raise ValueError(
                "Invalid `depth_image_size`: expected list of two integers [width, height]"
            )
        depth_image_size_tuple = (depth_image_size[0], depth_image_size[1])
        if depth_image_size_tuple not in self.DEPTH_IMAGE_ALLOWED_SIZES:
            raise ValueError(
                f"Unsupported `depth_image_size`: {depth_image_size}. "
                f"Allowed values: {sorted(self.DEPTH_IMAGE_ALLOWED_SIZES)}"
            )

    def _parse_scene_config(self, scene_config):
        if isinstance(scene_config, dict):
            scene_config = scene_config.get("type")

        if scene_config is None:
            return None

        scene_name = str(scene_config).strip()
        if scene_name == "" or scene_name.lower() in ("none", "robot", "base"):
            return None

        return scene_name

    def init_simulation(self):
        self.env = MorsMujocoEnv(xml_path=self.mjcf_root,
                 render_quality=self.render_quality,
                 scene=self.scene,
                 sim_freq=self.sim_freq,
                 motor_kp=0.0,
                 motor_kd=0.0,
                 ext_disturbance_enabled=self.external_disturbance_enabled,
                 init_motor_angles=self.init_motor_angles,
                 depth_image_enabled=self.depth_image_enabled,
                 depth_image_size=self.depth_image_size)

        if self.external_disturbance_enabled:
            self.env.set_ext_forces_params(self.external_disturbance_value, 
                                        self.external_disturbance_duration, 
                                        self.external_disturbance_interval)
        
    def loop(self):
        step_start = time.time()
        self.sim_it += 1

        self.env.set_kpkd(self.kp, self.kd)
        self.env.step(self.ref_joint_pos, self.ref_joint_vel, self.ref_joint_torq)

        self.contact_flags = self.env.get_contact_flags()

        self.cur_joint_pos = self.env.get_motor_angles()
        self.cur_joint_vel = self.env.get_motor_velocities()
        self.cur_joint_torq = self.env.get_motor_torques()

        self.imu_data[0:3] = self.env.get_base_lin_acc()
        self.imu_data[3:7] = self.env.get_base_orientation()
        self.imu_data[7:10] = self.env.get_base_ang_vel()
        self.imu_data[10:] = self.env.get_base_orientation_euler()
        
        self.body_lin_pos = self.env.get_base_position()
        self.body_lin_vel = self.env.get_base_lin_vel()

        if self.foot_positions_type == "global":
            self.foot_pos = self.env.get_global_foot_positions()
            self.foot_vel = self.env.get_global_foot_velocities()
        else:
            yaw = self.transform_yaw(self.imu_data[12])
            self.foot_pos = self.env.get_local_foot_positions(yaw)
            self.foot_vel = self.env.get_local_foot_velocities(yaw)
        
        self.__pub_servo_state(self.cur_joint_pos, self.cur_joint_vel, self.cur_joint_torq)
        self.__pub_imu_msg(self.imu_data)

        if self.foot_contacts_enabled:
            self.__pub_foot_contacts(self.contact_flags)
        
        if self.lcm_odometry_enabled:
            self.__pub_lcm_odom_msg(self.body_lin_pos, self.body_lin_vel, self.imu_data)

        if self.full_state_enabled:
            self.__pub_robot_state(imu_data=self.imu_data,
                                base_pos=self.body_lin_pos,
                                base_vel=self.body_lin_vel,
                                contact_states=self.contact_flags,
                                r1_pos=self.foot_pos[0],
                                l1_pos=self.foot_pos[1],
                                r2_pos=self.foot_pos[2],
                                l2_pos=self.foot_pos[3],
                                r1_vel=self.foot_vel[0],
                                l1_vel=self.foot_vel[1],
                                r2_vel=self.foot_vel[2],
                                l2_vel=self.foot_vel[3])
            
        elapsed = time.time() - step_start
        if elapsed < self.sim_period:
            time.sleep(self.sim_period - elapsed)

    def __pub_servo_state(self, joint_pos, joint_vel, joint_torq):
        for i in range(12):
            self.servo_state_msg.position[i] = self.joint_dir[i]*joint_pos[i] - self.joint_offset[i]
            self.servo_state_msg.velocity[i] = self.joint_dir[i]*joint_vel[i]
            self.servo_state_msg.torque[i] = self.joint_dir[i]*joint_torq[i] * self.gear_ratio / self.kt

        self.lc_servo_state.publish(self.lcm_servo_state_channel, self.servo_state_msg.encode())

    def __pub_imu_msg(self, imu_data : list):
        self.lcm_imu_msg.linear_acceleration = imu_data[:3]
        self.lcm_imu_msg.angular_velocity = imu_data[7:10]
        self.lcm_imu_msg.orientation_euler = imu_data[10:]
        self.lcm_imu_msg.orientation_quaternion = imu_data[3:7]
        
        self.lc_imu.publish(self.lcm_imu_channel, self.lcm_imu_msg.encode())

    def __pub_robot_state(self, imu_data, base_pos, base_vel, contact_states,
                           r1_pos, l1_pos, r2_pos, l2_pos,
                           r1_vel, l1_vel, r2_vel, l2_vel):
        yaw = self.transform_yaw(imu_data[12])
        rpy = [imu_data[10], imu_data[11], yaw]
        quat_xyzw = Rotation.from_euler('xyz', rpy, degrees=False).as_quat().tolist()

        self.lcm_robot_state_msg.timestamp = time.time_ns()
        self.lcm_robot_state_msg.body.position = base_pos[:]
        self.lcm_robot_state_msg.body.orientation = rpy[:]
        self.lcm_robot_state_msg.body.orientation_quaternion = quat_xyzw
        self.lcm_robot_state_msg.body.lin_vel = base_vel[:]
        self.lcm_robot_state_msg.body.ang_vel = imu_data[7:10]

        self.lcm_robot_state_msg.legs.r1_pos = r1_pos[:]
        self.lcm_robot_state_msg.legs.l1_pos = l1_pos[:]
        self.lcm_robot_state_msg.legs.r2_pos = r2_pos[:]
        self.lcm_robot_state_msg.legs.l2_pos = l2_pos[:]

        self.lcm_robot_state_msg.legs.r1_vel = r1_vel[:]
        self.lcm_robot_state_msg.legs.l1_vel = l1_vel[:]
        self.lcm_robot_state_msg.legs.r2_vel = r2_vel[:]
        self.lcm_robot_state_msg.legs.l2_vel = l2_vel[:]

        self.lcm_robot_state_msg.legs.contact_states = contact_states[:]

        self.lc_robot_state.publish(self.lcm_robot_state_channel, self.lcm_robot_state_msg.encode())

    def __pub_lcm_odom_msg(self, body_lin_pos, body_lin_vel, imu_data):
        self.lcm_odom_msg.position = body_lin_pos[:]
        self.lcm_odom_msg.orientation = imu_data[10:]
        self.lcm_odom_msg.orientation_quaternion = imu_data[3:7]
        self.lcm_odom_msg.lin_vel = body_lin_vel[:]
        self.lcm_odom_msg.ang_vel = imu_data[7:10]
        self.lc_odom.publish(self.lcm_odom_channel, self.lcm_odom_msg.encode())
    
    def __pub_foot_contacts(self, contact_flags):
        self.lcm_contact_sensor_msg.contact_states = contact_flags[:]
        self.lc_contact.publish(self.lcm_contact_sensor_channel, self.lcm_contact_sensor_msg.encode())

    def __pub_depth_image(self, image):
        depth_m = np.asarray(image, dtype=np.float32)
        if depth_m.ndim != 2:
            raise ValueError(f"Depth image must be 2D, got shape {depth_m.shape}")

        height, width = depth_m.shape
        depth_mm = np.zeros((height, width), dtype=np.uint16)

        depth_mm_float = np.rint(depth_m * 1000.0)
        valid_mask = np.isfinite(depth_m)
        valid_mask &= depth_mm_float >= 1.0
        valid_mask &= depth_mm_float <= float(np.iinfo(np.uint16).max)
        depth_mm[valid_mask] = depth_mm_float[valid_mask].astype(np.uint16)

        raw_payload = depth_mm.astype(np.dtype("<u2"), copy=False).tobytes(order="C")
        compressed_payload = zlib.compress(raw_payload, level=1)

        if len(compressed_payload) < len(raw_payload):
            payload = compressed_payload
            compression = 1
        else:
            payload = raw_payload
            compression = 0

        self.lcm_depth_image_msg.timestamp = time.time_ns()
        self.lcm_depth_image_msg.width = width
        self.lcm_depth_image_msg.height = height
        self.lcm_depth_image_msg.compression = compression
        self.lcm_depth_image_msg.data_size = len(payload)
        self.lcm_depth_image_msg.data = payload

        self.lc_depth_image.publish(self.depth_image_channel, self.lcm_depth_image_msg.encode())

    def depth_image_loop(self):
        next_deadline = time.perf_counter()
        while not self.shutdown_event.is_set():
            next_deadline += self.depth_image_period
            try:
                image = self.env.get_depth_image()
                if image is not None:
                    self.__pub_depth_image(image)
            except Exception as exc:
                print(f"[DEPTH_IMAGE] Failed to process frame: {exc}")

            sleep_time = next_deadline - time.perf_counter()
            if sleep_time > 0.0:
                if self.shutdown_event.wait(sleep_time):
                    break
            else:
                skipped = int((-sleep_time) / self.depth_image_period) + 1
                next_deadline += skipped * self.depth_image_period

    def get_cmd(self):
        # init LCM
        lc = self._create_lcm(self.lcm_servo_url)
        subscription = lc.subscribe(self.lcm_servo_cmd_channel, self.cmd_handler)
        try:
            while not self.shutdown_event.is_set():
                lc.handle_timeout(100)
        except KeyboardInterrupt:
            pass


    def cmd_handler(self, channel, data):
        msg = servo_cmd_msg.decode(data)
        for i in range(12):
            self.ref_joint_pos[i] = self.joint_dir[i] * msg.position[i] - self.joint_offset[i]
            self.ref_joint_vel[i] = self.joint_dir[i] * msg.velocity[i]
            self.ref_joint_torq[i] = self.joint_dir[i] * msg.torque[i] * self.kt / self.gear_ratio
            self.kp[i] = msg.kp[i]
            self.kd[i] = msg.kd[i]

    def get_sim_period(self):
        return self.sim_period

    def transform_yaw(self, yaw_raw):
        yaw_tmp = math.fmod(
            (2 * math.pi + yaw_raw - self.pre_yaw),
            (2 * math.pi)
        )

        if yaw_tmp > math.pi:
            self.yaw += (yaw_tmp - 2 * math.pi)
        elif yaw_tmp < -math.pi:
            self.yaw += (yaw_tmp + 2 * math.pi)
        else:
            self.yaw += yaw_tmp

        self.pre_yaw = self.yaw

        if self.first_yaw and self.yaw != 0.0:
            self.first_yaw = False
            self.offset_yaw = self.yaw
            self.yaw_final = 0.0
        else:
            self.yaw_final = self.yaw - self.offset_yaw

        return self.yaw_final

    def close(self):
        self.shutdown_event.set()

        if hasattr(self, "depth_image_th") and self.depth_image_th.is_alive():
            self.depth_image_th.join(timeout=1.0)
        if self.cmd_th.is_alive():
            self.cmd_th.join(timeout=1.0)

        self.env.close()
    

def main(args=None):
    hw_lvl_sim = Hardware_Level_Sim()
    try:
        while True:
            hw_lvl_sim.loop()
    except KeyboardInterrupt:
        pass
    finally:
        hw_lvl_sim.close()

if __name__ == '__main__':
    main()
