#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, PointStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np
import math
import sympy
from sympy import symbols, Matrix, sin, cos, tan, Piecewise, Abs, Function
from scipy.linalg import solve_continuous_are
import time

# =======
x_s, y_s, z_s, U_s, V_s, W_s, phi_s, theta_s, psi_s = symbols('x y z U V W phi theta psi')
ax_u, ay_u, az_u, p_u, q_u, r_u = symbols('ax ay az p q r') 
g = symbols('g')

state_vector_sym = Matrix([x_s, y_s, z_s, U_s, V_s, W_s, phi_s, theta_s, psi_s])

c_phi, s_phi = cos(phi_s), sin(phi_s)
c_theta, s_theta = cos(theta_s), sin(theta_s)
c_psi, s_psi = cos(psi_s), sin(psi_s)
    
sinc_theta = sympy.Piecewise((1, sympy.Eq(theta_s, 0)), (s_theta / theta_s, True))

sinc_phi = sympy.Piecewise((1, sympy.Eq(phi_s, 0)), (s_phi / phi_s, True))

delta = 0.07


A_matrix = Matrix([
    [0, 0, 0, c_theta * c_psi, s_phi * s_theta * c_psi + c_phi * s_psi, -c_phi * s_theta * c_psi + s_psi * s_phi, 0, 0, 0],
    [0, 0, 0, -c_theta * s_psi, -s_phi * s_theta * s_psi + c_phi * c_psi, c_phi * s_theta * s_psi + s_phi * s_psi, 0, 0, 0],
    [0, 0, 0, s_theta, -s_phi * c_theta, s_phi * c_theta, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, g * sinc_theta, 0],
    [0, 0, 0, 0, 0, 0, -g * c_theta * sinc_phi, 0, 0],
    [0, 0, 0, 0, 0, 0, -g * c_theta * c_phi * (phi_s/(phi_s**2 + delta)), 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0]
])

B_matrix = Matrix([
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, -W_s, V_s, 1, 0, 0],
    [W_s, 0, -U_s, 0, 1, 0],
    [-V_s, U_s, 0, 0, 0, 1],
    [1, s_phi * tan(theta_s), c_phi * tan(theta_s), 0, 0, 0],
    [0, c_phi, -s_phi, 0, 0, 0],
    [0, s_phi / c_theta, c_phi / c_theta, 0, 0, 0]
])

#gravity terms can be excluded from system dynamic matrix A(x) 
#for gravity terms seperate matrix D(x) will be introduce, 

#D_matrix = Matrix([0, 0, 0, g * sinc_theta, -g * c_theta * sinc_phi, - g * c_theta * c_phi, 0, 0, 0])

x_vect = Matrix([x_s, y_s, z_s, U_s, V_s, W_s, phi_s, theta_s, psi_s])
u_vect = Matrix([p_u, q_u, r_u, ax_u, ay_u, az_u])
a_sym = A_matrix * x_vect + B_matrix * u_vect #+ D_matrix (If gravity term excluded from A(x) then only)

# Lambdify process
input_order = (x_s, y_s, z_s, U_s, V_s, W_s, phi_s, theta_s, psi_s, p_u, q_u, r_u, ax_u, ay_u, az_u, g)

a_numeric = sympy.lambdify(input_order, a_sym, modules='numpy')
A_numeric = sympy.lambdify(input_order, A_matrix, modules='numpy')

class SDRFilterNode(Node):
    def __init__(self):
        super().__init__('sdre_filter_node')

        self.x = np.zeros((9, 1)) 
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        self.is_initialized = False

        self.accel = None
        self.gyro = None
        self.gps = None
        self.mag = None
        
        self.ekf_step_count = 0
        self.max_ekf_time = 0.0

        self.state = 'CALIBRATING_GYRO'

        self.gyro_bias_est = np.zeros(3)
        self.calib_N      = 400         
        self.calib_buf    = []

        self.init_N_a = 400
        self.init_accel_buf = []
        
        self.Q = np.diag([
            0.5, 0.5, 0.5,      
            1.25, 1.25, 3.0,      
            3.0, 1.5, 0.8       
        ]) # process noise variance

        self.R = np.diag([
            1.0, 1.0, 0.1,     
            0.0625, 0.0625,           
            0.002704                 
        ]) # GPS noise and roll pitch yaw noise

        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(PointStamped, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(Vector3Stamped, '/compass/data', self.compass_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/sdre/odom', 10)
        self.create_timer(0.05, self.sdre_step)

        self.get_logger().info("SDRE Filter Node has started.")

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

    def gps_callback(self, msg: PointStamped):
        self.gps = np.array([msg.point.x, msg.point.y, msg.point.z])
        if not self.is_initialized:

            self.x[0:3] = self.gps.reshape(3, 1)
            self.is_initialized = True
            self.get_logger().info(f"SDRE filter initialized with first GPS reading: {self.x[0:3].flatten()}")

    def compass_callback(self, msg: Vector3Stamped):
        self.mag = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

    def sdre_step(self):
        start_time = time.perf_counter()
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        if dt <= 0: return
        self.last_time = now

        if self.state != 'RUNNING' or self.gyro is None or self.accel is None or self.gps is None or self.mag is None:
            end_time = time.perf_counter()
            self.iteration_duration_ms = (end_time - start_time) * 1000
            return

        # Current State and Inputs
        x_val = self.x.flatten()
        p, q, r = self.gyro
        ax, ay, az = self.accel
        g_val = 9.81

        # Prediction Step 
        f_args = (*x_val, p, q, r, ax, ay, az, g_val)
        x_dot_pred = np.array(a_numeric(*f_args)).reshape(9, 1)

        A = np.array(A_numeric(*f_args)) 

        H = np.zeros((6, 9))
        H[0, 0] = 1; H[1, 1] = 1; H[2, 2] = 1 # GPS x, y, z
        H[3, 6] = 1; H[4, 7] = 1; H[5, 8] = 1 # Roll, Pitch, Yaw

        R_update = self.R.copy()
        # If acceleration is far from 1g, the drone is moving, so distrust accel for attitude
        if abs(np.linalg.norm(self.accel) - g_val) > 2.0:
            R_update[3, 3] *= 1.0e6  
            R_update[4, 4] *= 1.0e6  

        # find P from equ.AP+PA'- PH'(R^-1)HP+Q=0
        try:
            P = solve_continuous_are(A.T, H.T, self.Q, R_update) #ARE equation
        except np.linalg.LinAlgError:
            self.get_logger().warn("Riccati solution failed. Using prediction only.", throttle_duration_sec=1)
            self.x += x_dot_pred * dt
            self.publish_odom()
            return

        # filter gain Kf = P * H^T * R^-1
        Kf = P @ H.T @ np.linalg.inv(R_update)

        # Get sensor measurements
        roll_meas = math.atan2(self.accel[1], math.sqrt(self.accel[0]**2 + self.accel[2]**2))
        pitch_meas = math.atan2(-self.accel[0], self.accel[2])
        yaw_meas = math.atan2(self.mag[0], self.mag[1])

        z = np.array([[self.gps[0], self.gps[1], self.gps[2], roll_meas, pitch_meas, yaw_meas]]).T

        # innovation
        innovation = z - H @ self.x

        innovation[3:6] = (innovation[3:6] + np.pi) % (2 * np.pi) - np.pi

        # Update State
        x_dot_update = x_dot_pred + (Kf @ innovation)
        self.x += x_dot_update * dt

        self.publish_odom()
        
        end_time = time.perf_counter()
        iteration_duration_ms = (end_time - start_time) * 1000
        
        self.ekf_step_count += 1
        if self.ekf_step_count % 20 == 0:
            self.get_logger().info(f"SDRE Loop Time: {iteration_duration_ms:.3f} ms")

    def publish_odom(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = float(self.x[0])
        odom.pose.pose.position.y = float(self.x[1])
        odom.pose.pose.position.z = float(self.x[2])

        q = self.euler_to_quaternion(self.x[6,0], self.x[7,0], self.x[8,0])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = float(self.x[3])
        odom.twist.twist.linear.y = float(self.x[4])
        odom.twist.twist.linear.z = float(self.x[5])
        if self.gyro is not None:
            odom.twist.twist.angular.x = self.gyro[0]
            odom.twist.twist.angular.y = self.gyro[1]
            odom.twist.twist.angular.z = self.gyro[2]

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
    node = SDRFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
