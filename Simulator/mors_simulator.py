import os
import sys
import time
import warnings
from pathlib import Path
from threading import Thread

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

from additional.mors_env import MorsMujocoEnv

import yaml
import numpy as np
import math
from scipy.spatial.transform import Rotation

class Hardware_Level_Sim():
    def __init__(self):
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

        self.lc_servo_state = lcm.LCM()
        self.lc_imu = lcm.LCM()
        self.lc_robot_state = lcm.LCM()
        self.lc_odom = lcm.LCM()
        self.lc_contact = lcm.LCM()
        
        self.lcm_imu_msg.orientation_covariance = [2.603e-07, 0.0, 0.0, 0.0, 2.603e-07, 0.0, 0.0, 0.0, 0.0]
        self.lcm_imu_msg.angular_velocity_covariance = [2.5e-05, 0.0, 0.0, 0.0, 2.5e-05, 0.0, 0.0, 0.0, 2.5e-05]
        self.lcm_imu_msg.linear_acceleration_covariance = [2.5e-05, 0.0, 0.0, 0.0, 2.5e-05, 0.0, 0.0, 0.0, 2.5e-05]
        
        self.sim_it = 0
        self.init_simulation()

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

        self.vel_max = sim_config.get("vel_max", [28.0, 28.0, 14.0])
        self.tau_max = sim_config.get("tau_max", [6.0, 6.0, 12.0])
        self.gear_ratio = sim_config.get("gear_ratio", 10.0)
        self.kt = sim_config.get("kt", 0.74)

        with open(f"{BASE_DIR}/../config/channels.yaml", "r") as f:
            lcm_config = yaml.safe_load(f)

        self.lcm_servo_cmd_channel = lcm_config.get("lcm_servo_cmd_channel", "SERVO_CMD")
        self.lcm_servo_state_channel = lcm_config.get("lcm_servo_state_channel", "SERVO_STATE")
        self.lcm_imu_channel = lcm_config.get("lcm_imu_channel", "IMU_DATA")
        self.lcm_odom_channel = lcm_config.get("lcm_odom_channel", "ODOMETRY")
        self.lcm_contact_sensor_channel = lcm_config.get("lcm_contact_sensor_channel", "CONTACT_SENSOR")
        self.lcm_robot_state_channel = "ROBOT_STATE" #lcm_config.get("lcm_robot_state_channel", "ROBOT_STATE")

        self.sim_period = 1.0/self.sim_freq
        self.first_step = True

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
                 init_motor_angles=self.init_motor_angles)

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


    def get_cmd(self):
        # init LCM
        lc = lcm.LCM()
        subscription = lc.subscribe(self.lcm_servo_cmd_channel, self.cmd_handler)
        try:
            while True:
                lc.handle()
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
    

def main(args=None):
    hw_lvl_sim = Hardware_Level_Sim()
    while True:
        hw_lvl_sim.loop()

if __name__ == '__main__':
    main()
