import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped, Vector3
import csv
import os
from datetime import datetime
from scipy.spatial.transform import Rotation as R

class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')

        # File setup
        log_dir = os.path.expanduser('~/webots_ws_new/logs')
        os.makedirs(log_dir, exist_ok=True)
        self.log_file = os.path.join(log_dir, "ekf_follower_log_scen1.csv")

        with open(self.log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["time", "ekf_x", "ekf_y", "ekf_z", "x_o", "y_o", "z_o", "roll", "pitch", "yaw", "roll_o", "pitch_o", "yaw_o"])

        self.subscription_ekf = self.create_subscription(Odometry, '/ekf/odom2', self.ekf_callback, 10)
        self.rpy_subscriber = self.create_subscription(Vector3,'/rpy_angles_follower',self.rpy_callback,10)
        self.pos_subscriber = self.create_subscription(Vector3, '/pos_data_follower', self.pos_callback,10)

        self.ekf_position = None
        self.gps_origin = None 
        self.start_time = None
        self.last_log_time = 0.0

    def rpy_callback(self, msg):
        self.roll_o = msg.x
        self.pitch_o = msg.y
        self.yaw_o = msg.z

    def pos_callback(self, msg):
        self.xyz_data = msg

    def ekf_callback(self, msg):

        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.start_time is None:
            self.start_time = current_time

        relative_time = current_time - self.start_time

        if relative_time - self.last_log_time < 0.002:
            return

        self.last_log_time = relative_time

        ekf_pos = msg.pose.pose.position
        ekf_ori = msg.pose.pose.orientation
        ori_list = [ekf_ori.x, ekf_ori.y, ekf_ori.z, ekf_ori.w]
        roll, pitch, yaw = R.from_quat(ori_list).as_euler('xyz', degrees=False)

        xyz_msg = self.xyz_data

        x_o = xyz_msg.x
        y_o = xyz_msg.y
        z_o = xyz_msg.z

        with open(self.log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                relative_time,
                ekf_pos.x, ekf_pos.y, ekf_pos.z,
                x_o, y_o, z_o,
                roll, pitch, yaw,
                self.roll_o, self.pitch_o, self.yaw_o
            ])


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
