#!/usr/bin/env python3
from controller import Robot, GPS, InertialUnit, Accelerometer, Gyro, Compass
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Header
import math
from geometry_msgs.msg import Twist, Vector3Stamped, PointStamped, Vector3
import numpy as np
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

class SensorPublisher(Node):
    def __init__(self, robot_instance, gps_device, imu_device, accel_device, gyro_device, compass_device, timestep_val):
        super().__init__('sensor_publisher')

        self.robot = robot_instance
        self.gps = gps_device
        self.imu = imu_device
        self.accel = accel_device
        self.gyro = gyro_device
        self.compass = compass_device
        self.timestep = timestep_val
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0
        self.angular_velocity_x = 0.0
        self.angular_velocity_y = 0.0
        self.angular_velocity_z = 0.0

        self.pub_gps = self.create_publisher(PointStamped, '/gps/fix', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.pub_compass = self.create_publisher(Vector3Stamped, '/compass/data', 10)
        self.sub_sdre_odom = self.create_subscription(Odometry, '/sdre/odom', self.sdre_odom_callback, 10)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.rpy_publisher = self.create_publisher(Vector3, '/rpy_angles_sdre', 10)
        self.pos_publisher = self.create_publisher(Vector3, '/pos_data_sdre', 10)
        
        self.cmd_vel = Twist()

        self.imu_compass_frequency = 500.0 
        self.gps_frequency = 10.0          

        self.timer_imu_compass = self.create_timer(1.0 / self.imu_compass_frequency, self.publish_imu_compass_data)
        self.timer_gps = self.create_timer(1.0 / self.gps_frequency, self.publish_gps_data)

        self.get_logger().info(f"Publishing IMU/Compass at {self.imu_compass_frequency} Hz")
        self.get_logger().info(f"Publishing GPS at {self.gps_frequency} Hz")
        
        
    def sdre_odom_callback(self, msg):
        self.roll_rate = msg.twist.twist.angular.x
        self.pitch_rate = msg.twist.twist.angular.y
        self.yaw_rate = msg.twist.twist.angular.z
        
        q = msg.pose.pose.orientation
        q_list = [q.x, q.y, q.z, q.w]
        self.roll, self.pitch, self.yaw = R.from_quat(q_list).as_euler('xyz', degrees=False)

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
        self.get_logger().info(f'Received cmd_vel: linear x={msg.linear.x:.2f}, y={msg.linear.y:.2f}, z={msg.linear.z:.2f}, ang_z={msg.angular.z:.2f}')

    def publish_gps_data(self):
        gps_values = self.gps.getValues()

        std_x = 1.0  
        std_y = 1.0  
        std_z = 1.0  

        gps_msg = PointStamped()
        gps_msg.header = Header()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps_link" 

        gps_msg.point.x = gps_values[0] + np.random.normal(0, std_x)
        gps_msg.point.y = gps_values[1] + np.random.normal(0, std_y)
        gps_msg.point.z = gps_values[2] + np.random.normal(0, std_z)
        
        pos_msg = Vector3()
        pos_msg.x = gps_values[0]
        pos_msg.y = gps_values[1]
        pos_msg.z = gps_values[2]
        
        self.pos_publisher.publish(pos_msg)
        self.pub_gps.publish(gps_msg)

    def publish_imu_compass_data(self):
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        rpy_msg = Vector3()
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        rpy_msg.x = roll    
        rpy_msg.y = pitch
        rpy_msg.z = yaw
        self.rpy_publisher.publish(rpy_msg)
        
        q = rpy_to_quaternion(noisy_roll, noisy_pitch, noisy_yaw)
        
        imu_msg.orientation.x = q['x']
        imu_msg.orientation.y = q['y']
        imu_msg.orientation.z = q['z']
        imu_msg.orientation.w = q['w']
        
        
        # Accelration
        accel_std = np.array([0.25, 0.25, 0.25])
        noisy_accel = np.random.normal(0, accel_std)
        accel_values = self.accel.getValues()
        
        imu_msg.linear_acceleration.x = accel_values[0] + noisy_accel[0]
        imu_msg.linear_acceleration.y = accel_values[1] + noisy_accel[1]
        imu_msg.linear_acceleration.z = accel_values[2] + noisy_accel[2]

        # Gyro
        gyro_bias = np.array([0.2, 0.2, 0.2])
        noisy_gyro  = gyro_bias
        
        gyro_scale  = np.array([0.01, 0.01, 0.01])
        gyro_values = self.gyro.getValues()
        imu_msg.angular_velocity.x = gyro_values[0] + (gyro_values[0]*gyro_scale[0]) + noisy_gyro[0]
        imu_msg.angular_velocity.y = gyro_values[1] + (gyro_values[1]*gyro_scale[1]) + noisy_gyro[1]
        imu_msg.angular_velocity.z = gyro_values[2] + (gyro_values[2]*gyro_scale[2]) + noisy_gyro[2]
        
        self.angular_velocity_x = imu_msg.angular_velocity.x
        self.angular_velocity_y = imu_msg.angular_velocity.y
        self.angular_velocity_z = imu_msg.angular_velocity.z
        
        self.pub_imu.publish(imu_msg)

        # Magnetometer 
        magnetic_north = self.compass.getValues()  
        compass_msg = Vector3Stamped()
        compass_msg.header = Header()
        compass_msg.header.stamp = self.get_clock().now().to_msg()
        compass_msg.header.frame_id = "compass_link"
        compass_msg.vector.x = magnetic_north[0]
        compass_msg.vector.y = magnetic_north[1]
        compass_msg.vector.z = magnetic_north[2]
        self.pub_compass.publish(compass_msg)
    
def clamp(val, min_val=0, max_val=100):
    return max(min(val, max_val), min_val)
    
def rpy_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = {
        'x': sr * cp * cy - cr * sp * sy,
        'y': cr * sp * cy + sr * cp * sy,
        'z': cr * cp * sy - sr * sp * cy,
        'w': cr * cp * cy + sr * sp * sy
    }
    return q

robot = Robot()
timestep = int(robot.getBasicTimeStep())

gps = robot.getDevice("gps")
gps.enable(timestep)

imu_sensor = robot.getDevice("imu") 
imu_sensor.enable(timestep)

accel = robot.getDevice("accelerometer")
accel.enable(timestep)

gyro = robot.getDevice("gyro")
gyro.enable(timestep)

compass = robot.getDevice("compass")
compass.enable(timestep)

rear_left = robot.getDevice("rear left propeller")
rear_right = robot.getDevice("rear right propeller")
front_left = robot.getDevice("front left propeller")
front_right = robot.getDevice("front right propeller")

rear_left.setPosition(float('inf'))
rear_right.setPosition(float('inf'))
front_left.setPosition(float('inf'))
front_right.setPosition(float('inf'))

rear_left.setVelocity(0.0)
rear_right.setVelocity(0.0)
front_left.setVelocity(0.0)
front_right.setVelocity(0.0)

rclpy.init()
ros_node = SensorPublisher(robot, gps, imu_sensor, accel, gyro, compass, timestep)

class PIDController:
    def __init__(self, kp, ki, kd, dt, name="PID"):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0
        self.name = name

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output

# Main simulation loop
while robot.step(timestep) != -1:

    rclpy.spin_once(ros_node, timeout_sec=0) 
    
    dt = timestep / 1000.0 
    
    roll_rate = gyro.getValues()[0]
    pitch_rate = gyro.getValues()[1]
    yaw_rate = gyro.getValues()[2]

    roll = ros_node.roll
    pitch = ros_node.pitch
    
    VERTICAL_THRUST = 66.047
    z_input = ros_node.cmd_vel.linear.z  
    total_thrust = VERTICAL_THRUST + (10.0 * z_input)
    yaw_control = ros_node.cmd_vel.angular.z
    
    pitch_angle_pid = PIDController(kp=1.0, ki=0.0, kd=0.02, dt=dt, name="PitchAngle")
    pitch_rate_pid  = PIDController(kp=0.5, ki=0.0, kd=0.002, dt=dt, name="PitchRate")
    
    roll_angle_pid = PIDController(kp=1.0, ki=0.0, kd=0.05, dt=dt, name="RollAngle")
    roll_rate_pid  = PIDController(kp=0.6, ki=0.0, kd=0.004, dt=dt, name="RollRate")
    
    try:
        forward_velocity = ros_node.cmd_vel.linear.x if ros_node.cmd_vel.linear.x is not None else 0.0
        lateral_velocity = ros_node.cmd_vel.linear.y if ros_node.cmd_vel.linear.y is not None else 0.0
    except:
        forward_velocity = 0.0
        lateral_velocity = 0.0
    
    k_angle_p = 0.06
    k_angle_r = 0.06
    desired_pitch = k_angle_p * forward_velocity
    desired_roll  = -k_angle_r * lateral_velocity
    
    # Pitch Control
    pitch_rate_desired = pitch_angle_pid.compute(desired_pitch, pitch)
    pitch_control = pitch_rate_pid.compute(pitch_rate_desired, pitch_rate)

    # Roll Control
    roll_rate_desired = roll_angle_pid.compute(desired_roll, roll)
    roll_control = roll_rate_pid.compute(roll_rate_desired, roll_rate) 
    
    front_left_speed  = total_thrust  - pitch_control + roll_control - yaw_control
    front_right_speed = total_thrust  - pitch_control - roll_control + yaw_control
    rear_left_speed   = total_thrust  + pitch_control + roll_control + yaw_control
    rear_right_speed  = total_thrust  + pitch_control - roll_control - yaw_control

    front_left.setVelocity(clamp(front_left_speed))
    front_right.setVelocity(-clamp(front_right_speed))
    rear_left.setVelocity(-clamp(rear_left_speed))
    rear_right.setVelocity(clamp(rear_right_speed))

ros_node.destroy_node()
rclpy.shutdown()