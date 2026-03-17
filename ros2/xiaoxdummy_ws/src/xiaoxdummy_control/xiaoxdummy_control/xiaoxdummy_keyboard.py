#!/usr/bin/env python3
import sys
import tty
import termios
import select
import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class DummyKeyboardController(Node):
    def __init__(self):
        super().__init__('dummy_keyboard_controller')

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        self.current_positions = [0.0] * 6

        self.step = 0.1  # 每次按键变化弧度
        self.limit = 3.14

        self.get_logger().info(
            '\n键盘控制说明：\n'
            'q/a -> Joint1 +/-\n'
            'w/s -> Joint2 +/-\n'
            'e/d -> Joint3 +/-\n'
            'r/f -> Joint4 +/-\n'
            't/g -> Joint5 +/-\n'
            'y/h -> Joint6 +/-\n'
            'z   -> 全部回零\n'
            'x   -> 退出\n'
        )

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                ch = sys.stdin.read(1)
            else:
                ch = ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def clamp(self, value):
        return max(min(value, self.limit), -self.limit)

    def publish_positions(self):
        msg = JointTrajectory()
        # Set stamp to 0 to execute immediately, avoiding sim_time vs wall_time issues
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in self.current_positions]
        point.time_from_start = Duration(sec=1, nanosec=0)

        msg.points = [point]
        self.publisher.publish(msg)

        self.get_logger().info(f'当前关节角: {[round(v, 3) for v in self.current_positions]}')

    def run(self):
        key_map = {
            'q': (0, +1), 'a': (0, -1),
            'w': (1, +1), 's': (1, -1),
            'e': (2, +1), 'd': (2, -1),
            'r': (3, +1), 'f': (3, -1),
            't': (4, +1), 'g': (4, -1),
            'y': (5, +1), 'h': (5, -1),
        }

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            key = self.get_key()

            if not key:
                continue

            if key == '\x03' or key == 'x':  # \x03 is Ctrl+C
                self.get_logger().info('退出键盘控制')
                break

            if key == 'z':
                self.current_positions = [0.0] * 6
                self.publish_positions()
                continue

            if key in key_map:
                idx, direction = key_map[key]
                self.current_positions[idx] = self.clamp(
                    self.current_positions[idx] + direction * self.step
                )
                self.publish_positions()


def main(args=None):
    rclpy.init(args=args)
    node = DummyKeyboardController()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
