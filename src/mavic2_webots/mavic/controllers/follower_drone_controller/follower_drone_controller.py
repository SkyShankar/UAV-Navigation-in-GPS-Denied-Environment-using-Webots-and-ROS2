#!/usr/bin/env python3

from controller import Robot, GPS, InertialUnit, Accelerometer, Gyro, Compass, Lidar
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Header
import math
from geometry_msgs.msg import Twist, Vector3Stamped, PointStamped, Vector3, Point
import cv2
import numpy as np
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import onnxruntime as ort

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
        self.latest_detection = None
        self.ekf_ready = False
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0        
        self.current_roll = None
        self.current_pitch = None
        
        self.pub_imu = self.create_publisher(Imu, '/imu/data2', 10)
        self.pub_compass = self.create_publisher(Vector3Stamped, '/compass/data2', 10)
        self.sub_sdre_odom = self.create_subscription(Odometry, '/ekf/odom2', self.ekf_odom_callback, 10)
        self.pub_target_point = self.create_publisher(PointStamped, '/target/relative_position', 10)
        self.rpy_publisher = self.create_publisher(Vector3, '/rpy_angles_follower', 10)
        self.pos_publisher = self.create_publisher(Vector3, '/pos_data_follower', 10)
        
        self.cmd_vel = Twist()
        
        self.imu_compass_frequency = 500.0 
        self.gps_frequency = 10.0          

        self.timer_imu_compass = self.create_timer(1.0 / self.imu_compass_frequency, self.publish_imu_compass_data)
        self.timer_gps = self.create_timer(1.0 / self.gps_frequency, self.publish_gps_data)
        
    def publish_target_point(self, x, y, z):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "lidar_link"  
        
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)
        
        self.pub_target_point.publish(msg)
        
    def ekf_odom_callback(self, msg):
        
        q = msg.pose.pose.orientation
        q_list = [q.x, q.y, q.z, q.w]
        
        if np.linalg.norm(q_list) < 1e-6:
            self.get_logger().warn(
                "Received an invalid (zero norm) quaternion from EKF, skipping message.", 
                throttle_duration_sec=5
            )
            return
        
        self.roll_rate = msg.twist.twist.angular.x
        self.pitch_rate = msg.twist.twist.angular.y
        self.yaw_rate = msg.twist.twist.angular.z
        
        self.roll, self.pitch, self.yaw = R.from_quat(q_list).as_euler('xyz', degrees=False)
        if not self.ekf_ready:
            self.ekf_ready = True
        
    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
        self.get_logger().info(f'Received cmd_vel: linear x={msg.linear.x:.2f}, y={msg.linear.y:.2f}, z={msg.linear.z:.2f}')
        
    def publish_gps_data(self):     
        gps_values = self.gps.getValues()   
        pos_msg = Vector3()
        pos_msg.x = gps_values[0]
        pos_msg.y = gps_values[1]
        pos_msg.z = gps_values[2]
    
        self.pos_publisher.publish(pos_msg)
        
    def publish_imu_compass_data(self):
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        
        rpy_msg = Vector3()
        roll_1, pitch_1, yaw_1 = self.imu.getRollPitchYaw()
        rpy_msg.x = roll_1
        rpy_msg.y = pitch_1
        rpy_msg.z = yaw_1
        self.rpy_publisher.publish(rpy_msg)
        
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
        
class KalmanFilter:
    def __init__(self, dt):
        self.dt = dt
        self.x = np.zeros((4, 1))
        self.F = np.array([[1, 0, self.dt, 0], [0, 1, 0, self.dt], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.P = np.eye(4) * 10
        self.Q = np.eye(4) * 5.0
        self.R = np.eye(2) * 5.0
        
    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x
        
    def update(self, z):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        
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
    
def clamp(val, min_val=0, max_val=100):
    return max(min(val, max_val), min_val)
    
def letterbox(im, new_shape=(640, 640), color=(114, 114, 114)):
    shape = im.shape[:2]  
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  

    dw /= 2  
    dh /= 2

    if shape[::-1] != new_unpad:  
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  
    return im, r, (dw, dh)

robot = Robot()
timestep = int(robot.getBasicTimeStep())

gps = robot.getDevice("gps2")
gps.enable(timestep)

imu_sensor = robot.getDevice("inertial_unit2")
imu_sensor.enable(timestep)

accel = robot.getDevice("accelerometer2")
accel.enable(timestep)

gyro = robot.getDevice("gyro2")
gyro.enable(timestep)

compass = robot.getDevice("compass2")
compass.enable(timestep)

camera = robot.getDevice('camera')
camera.enable(timestep)

camera_roll_motor = robot.getDevice("camera roll")
camera_pitch_motor = robot.getDevice("camera pitch")

lidar = robot.getDevice('lidar') 
lidar.enable(timestep)
lidar.enablePointCloud()

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

dt = timestep / 1000.0
target_kf = KalmanFilter(dt=dt)
is_kf_initialized = False

model_path = "/home/akash_shankar/webots_ws_new/src/mavic2_webots/runs/detect/train11/weights/best.onnx" # YOLO file
session = ort.InferenceSession(model_path)
model_input = session.get_inputs()[0].name

rclpy.init()
ros_node = SensorPublisher(robot, gps, imu_sensor, accel, gyro, compass, timestep)

INFERENCE_FREQUENCY = 5  # frequency of YOLO
step_counter = 0


while robot.step(timestep) != -1:
    rclpy.spin_once(ros_node, timeout_sec=0)
    dt = timestep / 1000.0  

    if is_kf_initialized:
        predicted_state = target_kf.predict()
    
    if step_counter % INFERENCE_FREQUENCY == 0:
    
        image_raw = camera.getImage()
        if image_raw:
            frame_bgra = np.frombuffer(image_raw, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
            frame_bgr = frame_bgra[:, :, :3]
    
            # --- YOLO INFERENCE ---
            # Pre-process the image for the model
            image_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            
            image_letterboxed, ratio, pad = letterbox(image_rgb, (640, 640))
            
            input_tensor = image_letterboxed.transpose(2, 0, 1).astype(np.float32) / 255.0
            
            input_tensor = np.expand_dims(input_tensor, axis=0)
            
            # Run the model
            outputs = session.run(None, {model_input: input_tensor})
            
            # Post-process the results
            detections = outputs[0][0]
            best_detection = None
            max_confidence = 0.85 
    
            for detection in detections.T:
                confidence = detection[4]
                
                if confidence > max_confidence:
                    best_detection = detection
                    
            if best_detection is not None:
                box = best_detection[:4]
                
                box[0] -= pad[0]  
                box[1] -= pad[1]  
                box /= ratio      
                
                cx, cy, w, h = box
                
                measured_x = cx 
                measured_y = cy 
                
                # KF Update
                z = np.array([[measured_x], [measured_y]])
                if not is_kf_initialized:
                    target_kf.x[:2] = z
                    is_kf_initialized = True
                else:
                    target_kf.update(z)
                            
        # Use the KF's current state for control
        if is_kf_initialized:
            estimated_target_x = target_kf.x[0, 0]
            estimated_target_y = target_kf.x[1, 0]
            
            camera_width = camera.getWidth()
            camera_height = camera.getHeight()
            
            camera_center_x = camera_width / 2
            camera_center_y = camera_height / 2
            
            lidar_resolution = lidar.getHorizontalResolution()
            lidar.enablePointCloud()
            point_cloud = lidar.getPointCloud()
            
            target_lidar_index = np.clip(int((estimated_target_x / camera_width) * lidar_resolution), 0, lidar_resolution - 1)
            
            # Yaw Controll 
            
            yaw_roll = measured_x - camera_center_x
            
            yaw_pid = PIDController(kp=0.000001, ki=0.000, kd=0.00005, dt=dt, name="Yaw")
            yaw_input = yaw_pid.compute(setpoint=camera_center_x, measurement=estimated_target_x)
            
            if -0.01 < yaw_roll < 0.01:
                yaw_input = 0.0
                
            # Altitude controll
            
            HFOV = camera.getFov()
            VFOV = 2 * math.atan(math.tan(HFOV * 0.5) * (camera_height / camera_width))

            vertical_angle = ((camera_center_y - estimated_target_y) / camera_height) * VFOV
            
            error_pitch = estimated_target_y - camera_center_y
            
            relative_altitude = math.sin(vertical_angle)
            
            K_P_ALT = 10
            altitude_correction = K_P_ALT * vertical_angle
            
            HOVER_INPUT = 0.0
            alt_input_1 = altitude_correction + HOVER_INPUT
            
            if -0.125 < alt_input_1 < 0.125:
                alt_input = 0.0            
            
            else:
                alt_pid = PIDController(kp=0.08, ki=0.0, kd=0.08, dt=dt, name="Altitude")
                alt_input = -(alt_pid.compute(setpoint=0.0, measurement=relative_altitude))
                
            # Pitch controll 
            
            num_layers = lidar.getNumberOfLayers()
            
            lidar_vertical_fov = lidar.getVerticalFov()
            
            inverted_layer_index = int(((vertical_angle + lidar_vertical_fov / 2.0) / lidar_vertical_fov) * num_layers)
            
            layer_index = (num_layers - 1) - inverted_layer_index
            
            layer_index = np.clip(layer_index, 0, num_layers - 1)
            
            full_index = (layer_index * lidar_resolution) + target_lidar_index
            
            target_point = point_cloud[full_index]
            p = target_point
            ros_node.publish_target_point(p.x, p.y, p.z)
            
            distances_1 = math.sqrt(p.x**2 + p.y**2 + p.z**2)
            if distances_1 != float('inf'):
                valid_distances = distances_1
            
            VERTICAL_TOLERANCE = 5
            if abs(error_pitch) > VERTICAL_TOLERANCE:
                TARGET_DISTANCE = 5
                error_distance = valid_distances - TARGET_DISTANCE

                if -0.02 < error_distance < 0.02:
                    pitch_input = 0.0
                    
                else:
                    pitch_pid = PIDController(kp=0.008, ki=0.00, kd=0.002, dt=dt, name="pitch")
                    pitch_input = -pitch_pid.compute(setpoint=TARGET_DISTANCE, measurement=valid_distances)
            else:
                pitch_input = 0.0

    # -------------------------------------------------------------------------------------
    
    roll_rate = gyro.getValues()[0]
    pitch_rate = gyro.getValues()[1]
    yaw_rate = gyro.getValues()[2]
    
    roll = ros_node.roll
    pitch = ros_node.pitch
    yaw = ros_node.yaw
    
    VERTICAL_THRUST = 66.047
    
    if ros_node.ekf_ready:
        # Initialize PIDs
        pitch_angle_pid = PIDController(kp=1.0, ki=0.0, kd=0.09, dt=dt, name="PitchAngle")
        pitch_rate_pid  = PIDController(kp=0.05, ki=0.00, kd=0.000, dt=dt, name="PitchRate")
        
        roll_angle_pid = PIDController(kp=1.0, ki=0.0, kd=0.08, dt=dt, name="RollAngle")
        roll_rate_pid  = PIDController(kp=0.05, ki=0.000, kd=0.0006, dt=dt, name="RollRate")
        
        yaw_angle_pid = PIDController(kp=1.0, ki=0.0, kd=0.01, dt=dt, name="YawAngle")
        yaw_rate_pid  = PIDController(kp=0.0655, ki=0.000, kd=0.0006, dt=dt, name="YawRate")
        
        # Desired angles
        k_angle_p = 0.03 
        k_angle_y = 1.0
        desired_pitch = k_angle_p * clamp(pitch_input,-2.0, 2.0)
        desired_roll  = 0.0
        desired_yaw = k_angle_y * clamp(yaw_input, -1.0, 1.0)
        
        # Pitch Control
        pitch_rate_desired = pitch_angle_pid.compute(desired_pitch, pitch)
        pitch_control = pitch_rate_pid.compute(pitch_rate_desired, pitch_rate)
        
        # Roll Control
        roll_rate_desired = roll_angle_pid.compute(desired_roll, roll)
        roll_control = roll_rate_pid.compute(roll_rate_desired, roll_rate)
        
        # Yaw Controll
        yaw_rate_desired = yaw_angle_pid.compute(desired_yaw, yaw)
        yaw_control = yaw_rate_pid.compute(yaw_rate_desired, yaw_rate)

    else:
        pitch_control = 0.0
        roll_control = 0.0
        alt_input = 0.0
        yaw_control = 0.0

    # Motor Control
    total_thrust = VERTICAL_THRUST + (10.0 * alt_input)
    
    front_left_speed  = total_thrust  - pitch_control + roll_control - yaw_control
    front_right_speed = total_thrust  - pitch_control - roll_control + yaw_control
    rear_left_speed   = total_thrust  + pitch_control + roll_control + yaw_control
    rear_right_speed  = total_thrust  + pitch_control - roll_control - yaw_control

    front_left.setVelocity(clamp(front_left_speed))
    front_right.setVelocity(-clamp(front_right_speed))
    rear_left.setVelocity(-clamp(rear_left_speed))
    rear_right.setVelocity(clamp(rear_right_speed))
    
    camera_roll_motor.setPosition(-roll)
    camera_pitch_motor.setPosition(-pitch)
                         
    step_counter += 1 

ros_node.destroy_node()
rclpy.shutdown()
