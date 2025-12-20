import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Vector3Stamped,Vector3,Twist, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np
import math
import sympy
from sympy import symbols, Matrix, sin, cos, tan, sec

import tkinter as tk
import threading
import matplotlib.pyplot as plt
import time

# --- sym setup ---
x, y, z, U, V, W, phi, theta, psi = symbols('x y z U V W phi theta psi')
ax, ay, az, p, q, r = symbols('ax ay az p q r')
bgx, bgy, bgz = symbols('bgx bgy bgz')
g = symbols('g')

state_vector_sym = Matrix([x, y, z, U, V, W, phi, theta, psi])

c_phi, s_phi = cos(phi), sin(phi)
c_theta, s_theta = cos(theta), sin(theta)
c_psi, s_psi = cos(psi), sin(psi)

C_bn = Matrix([
    [c_theta * c_psi, -c_theta * s_psi, s_theta],
    [s_phi * s_theta * c_psi + c_phi * s_psi, s_phi * -s_theta * s_psi + c_phi * c_psi, -s_phi * c_theta],
    [-c_phi * s_theta * c_psi + s_phi * s_psi, c_phi * s_theta * s_psi + s_phi * c_psi, c_phi * c_theta]
])

vel_body = Matrix([U, V, W])
vel_nav = C_bn.T * vel_body

phi_dot = p + q * s_phi * tan(theta) + r * c_phi * tan(theta)
theta_dot = q * c_phi - r * s_phi
psi_dot = r * c_phi / c_theta + q * s_phi / c_theta

U_dot = ax + V*r - W*q + g*s_theta
V_dot = ay - U*r + W*p - g*c_theta*s_phi
W_dot = az + U*q - V*p - g*c_theta*c_phi

f_sym = Matrix([
    vel_nav[0], vel_nav[1], vel_nav[2],
    U_dot, V_dot, W_dot,
    phi_dot, theta_dot, psi_dot
])
jacobian_sym = f_sym.jacobian(state_vector_sym)

input_args_order = (x, y, z, U, V, W, phi, theta, psi, p, q, r, ax, ay, az, g)
f_numeric = sympy.lambdify(input_args_order, f_sym, modules='numpy')
jacobian_numeric = sympy.lambdify(input_args_order, jacobian_sym, modules='numpy')

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        self.x = np.zeros((9, 1))
        self.P = np.eye(9) * 1.0 # error covariance matrix

        self.Q = np.diag([
            0.5, 0.5, 0.5,
            0.25, 0.25, 0.25,
            0.02, 0.04, 0.8   
        ]) # process noise variance


        self.R = np.diag([
            0.0625, 0.0625, 0.002704
        ]) # only roll pitch yaw noise

        self.last_time = self.get_clock().now().nanoseconds / 1e9

        self.accel = None
        self.gyro = None
        self.gps = None
        self.mag = None
        self.lat_ref = None
        self.lon_ref = None
        self.tswitch = 0.0

        self.roll_out = 0.0
        self.pitch_out = 0.0
        self.filter_alpha_out = 0.05

        self.state = 'CALIBRATING_GYRO'

        self.gyro_bias_est = np.zeros(3)
        self.calib_N      = 400         
        self.calib_buf    = []

        self.init_N_a = 400
        self.init_accel_buf = []

        self.create_subscription(Imu, '/imu/data2', self.imu_callback, 10)
        self.create_subscription(Vector3Stamped, '/compass/data2', self.compass_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/ekf/odom2', 10)
        self.create_timer(0.05, self.ekf_step)

    def imu_callback(self, msg):

        if self.state == 'CALIBRATING_GYRO':
            self.calib_buf.append(
                np.array([msg.angular_velocity.x,
                          msg.angular_velocity.y,
                          msg.angular_velocity.z])
            )
            if len(self.calib_buf) >= self.calib_N:
                self.gyro_bias_est = np.mean(self.calib_buf, axis=0)
                self.get_logger().info(
                    f"Gyro bias calibrated: {self.gyro_bias_est} rad/s"
                )
                self.state = 'INITIALIZING_ATTITUDE'
            return             

        elif self.state == 'INITIALIZING_ATTITUDE':
            self.init_accel_buf.append(
                np.array([msg.linear_acceleration.x,
                          msg.linear_acceleration.y,
                          msg.linear_acceleration.z])
            )
            if len(self.init_accel_buf) >= self.init_N_a:
                ax_avg, ay_avg, az_avg = np.mean(self.init_accel_buf, axis=0)
                self.get_logger().info(f"Averaged Accel: ax={ax_avg:.2f}, ay={ay_avg:.2f}, az={az_avg:.2f}")

                initial_roll = math.atan2(ay_avg, math.sqrt(ax_avg**2 + az_avg**2))
                initial_pitch = math.atan2(-ax_avg, az_avg)
                initial_yaw = 0.0

                self.x[6, 0] = initial_roll
                self.x[7, 0] = initial_pitch
                self.x[8, 0] = initial_yaw
                self.P[6, 6] = 1e-8
                self.P[7, 7] = 1e-8

                self.state = 'RUNNING'
            return

        elif self.state == 'RUNNING':
            self.gyro = np.array([
                msg.angular_velocity.x - self.gyro_bias_est[0],
                msg.angular_velocity.y - self.gyro_bias_est[1],
                msg.angular_velocity.z - self.gyro_bias_est[2],
            ])
            self.accel = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])

    def compass_callback(self, msg):
        self.mag = msg.vector  

    def ekf_step(self):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        self.last_time = now

        if self.state != 'RUNNING' or self.gyro is None or self.accel is None or self.mag is None:
            return

        x_val, y_val, z_val = self.x[0,0], self.x[1,0], self.x[2,0]
        U_val, V_val, W_val = self.x[3,0], self.x[4,0], self.x[5,0]
        phi_val, theta_val, psi_val = self.x[6,0], self.x[7,0], self.x[8,0]
        p_val, q_val, r_val = self.gyro
        ax_val, ay_val, az_val = self.accel
        g_val = 9.81

        inputs = (x_val, y_val, z_val, U_val, V_val, W_val,
                  phi_val, theta_val, psi_val,
                  p_val, q_val, r_val,
                  ax_val, ay_val, az_val, g_val)

        x_dot = np.array(f_numeric(*inputs)).reshape(9, 1)
        F = np.array(jacobian_numeric(*inputs)).reshape(9, 9)
        
        # Prediction Step
        self.x += x_dot * dt # 9*1
        self.P = F @ self.P @ F.T + self.Q * dt # 9*9

        # --- Angle Correction ---

        # Roll and pitch from accelerometer
        ax_f, ay_f, az_f = self.accel
        roll = math.atan2(ay_f, math.sqrt(ax_f**2 + az_f**2))
        pitch = math.atan2(-ax_f, az_f)

        # Yaw from magnetometer 
        mag_x, mag_y = self.mag.x, self.mag.y
        yaw = math.atan2(mag_x, mag_y)

        z = np.array([[roll], [pitch], [yaw]])

        R_update = self.R.copy()
        accel_magnitude = np.linalg.norm(self.accel)

        #print ("accel based")
        # If acceleration is far from 1g, the drone is moving, so distrust accel for attitude
        if abs(accel_magnitude - g_val) > 2.0: 
            R_update[0, 0] *= 1.0e6  
            R_update[1, 1] *= 1.0e6  

        H = np.zeros((3, 9))
        H[0, 6] = 1
        H[1, 7] = 1
        H[2, 8] = 1
        
        # innovation
        y = z - H @ self.x # 9*1

        S = H @ self.P @ H.T + R_update # 9*9

        K = self.P @ H.T @ np.linalg.inv(S) # 9*9
        
        # Update State
        self.x += K @ y
        self.P = (np.eye(9) - K @ H) @ self.P

        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = float(self.x[0])
        odom.pose.pose.position.y = float(self.x[1])
        odom.pose.pose.position.z = float(self.x[2])

        qx, qy, qz, qw = self.euler_to_quaternion(self.x[6,0], self.x[7,0], self.x[8,0])
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = float(self.x[3])
        odom.twist.twist.linear.y = float(self.x[4])
        odom.twist.twist.linear.z = float(self.x[5])

        odom.twist.twist.angular.x = p_val
        odom.twist.twist.angular.y = q_val
        odom.twist.twist.angular.z = r_val


        odom.pose.covariance[:9] = self.P[0:3, 0:3].flatten()
        odom.twist.covariance[:9] = self.P[3:6, 3:6].flatten()

        self.odom_pub.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
