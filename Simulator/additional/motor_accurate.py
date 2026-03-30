import numpy as np


class MotorModel():
    def __init__(self, num_motors = 1):
        self.num_motors = num_motors
        
        self.theta_ref = np.zeros(self.num_motors)
        self.theta_cur = np.zeros(self.num_motors)
        self.omega_ref = np.zeros(self.num_motors)
        self.omega_cur = np.zeros(self.num_motors)
        self.tau_ref = np.zeros(self.num_motors)
        self.tau_cur = np.zeros(self.num_motors)
        self.kp = np.zeros(self.num_motors)
        self.kd = np.zeros(self.num_motors)

        self.e = np.zeros(self.num_motors)
        self.e_prev = np.zeros(self.num_motors)
        self.d_e = np.zeros(self.num_motors)
        self.u = np.zeros(self.num_motors)

    def set_ref_angle(self, theta : np.ndarray):
        self.theta_ref = np.array(theta)
    
    def set_ref_vel(self, vel):
        self.omega_ref = np.array(vel)

    def set_ref_torque(self, torque):
        self.tau_ref = np.array(torque)

    def set_kp(self, kp):
        self.kp = np.array(kp)

    def set_kd(self, kd):
        self.kd = np.array(kd)

    def set_sensor_data(self, theta : np.ndarray, omega : np.ndarray):
        self.theta_cur = np.array(theta)
        self.omega_cur = np.array(omega)

    def step(self):
        self.e = self.theta_ref - self.theta_cur
        self.d_e = self.omega_ref - self.omega_cur

        self.p = self.kp * self.e
        self.d = self.kd * self.d_e

        self.u = self.p + self.d + self.tau_ref

        return self.u