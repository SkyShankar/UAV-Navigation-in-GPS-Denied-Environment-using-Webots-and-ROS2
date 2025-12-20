#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import curses
import threading
import time

LINEAR_SPEED = 1.0
ALTITUDE_SPEED = 0.7
ANGULAR_SPEED = 0.3

KEY_TIMEOUT = 0.2

# Key mappings
KEY_BINDINGS = {
    'w': ('x', 1),
    's': ('x', -1),
    'a': ('y', 1),
    'd': ('y', -1),
    'r': ('z', 1),
    'f': ('z', -1),
    'q': ('yaw', 1),
    'e': ('yaw', -1),
}

class TeleopNode(Node):
    def __init__(self, stdscr):
        super().__init__('teleop_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.active_keys = set()
        self.lock = threading.Lock()
        self.stdscr = stdscr
        self.last_key_press_time = time.time()


        self.running = True
        self.key_thread = threading.Thread(target=self.key_listener_loop)
        self.key_thread.daemon = True
        self.key_thread.start()

        # --- Main ROS2 Timer ---
        self.timer = self.create_timer(0.05, self.publish_twist_message) # Publish at 20 Hz

        self.print_instructions()

    def print_instructions(self):
        self.stdscr.clear()
        self.stdscr.addstr(0, 0, "Drone Keyboard Teleop (Press ESC to Quit)")
        self.stdscr.addstr(2, 0, "Move:     W/S = Forward/Back  A/D = Left/Right")
        self.stdscr.addstr(3, 0, "Altitude: R/F = Up/Down")
        self.stdscr.addstr(4, 0, "Rotate:   Q/E = Yaw Left/Right")
        self.stdscr.addstr(6, 0, "Hold multiple keys for combined motion.")
        self.stdscr.refresh()

    def key_listener_loop(self):

        self.stdscr.nodelay(True)
        while self.running and rclpy.ok():
            try:
                key_code = self.stdscr.getch()
                if key_code != -1: # A key was pressed
                    char = chr(key_code).lower()

                    if key_code == 27: # ESC key
                        self.running = False
                        rclpy.shutdown()
                        break

                    if char in KEY_BINDINGS:
                        with self.lock:
                            self.active_keys.add(char)
                            self.last_key_press_time = time.time()

            except Exception as e:
                pass
            time.sleep(0.01)

    def publish_twist_message(self):
        with self.lock:
            # If no key has been pressed for long time
            if time.time() - self.last_key_press_time > KEY_TIMEOUT:
                self.active_keys.clear()

            twist = Twist()
            active_axes = {'x': 0, 'y': 0, 'z': 0, 'yaw': 0}

            for key in self.active_keys:
                if key in KEY_BINDINGS:
                    axis, direction = KEY_BINDINGS[key]
                    active_axes[axis] += direction

            twist.linear.x = float(active_axes['x'] * LINEAR_SPEED)
            twist.linear.y = float(active_axes['y'] * LINEAR_SPEED)
            twist.linear.z = float(active_axes['z'] * ALTITUDE_SPEED)
            twist.angular.z = float(active_axes['yaw'] * ANGULAR_SPEED)

            self.publisher_.publish(twist)

            self.stdscr.addstr(8, 0, f"Active Keys: {sorted(list(self.active_keys))}      ")
            self.stdscr.addstr(9, 0, f"Publishing: lx={twist.linear.x:.2f}, ly={twist.linear.y:.2f}, lz={twist.linear.z:.2f}, az={twist.angular.z:.2f}      ")
            self.stdscr.refresh()


    def destroy_node(self):
        self.running = False
        if self.key_thread.is_alive():
            self.key_thread.join()
        self.get_logger().info("Shutting down Teleop Controller.")
        super().destroy_node()

def main_wrapper(stdscr):
    rclpy.init()
    node = TeleopNode(stdscr)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):

    try:
        curses.wrapper(main_wrapper)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
