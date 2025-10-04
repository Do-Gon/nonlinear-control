#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
import time

class Joint1VelocityNode(Node):
    def __init__(self):
        super().__init__('joint1_velocity_node')

        # Open at the correct baud for Arduino/M5Stack bridge
        self.mc = MyCobot('/dev/ttyUSB0', 1000000)
        time.sleep(1.0)

        # Joint 1 setup
        self.jid    = Angle.J1.value    # 1
        self.target = 10.0               # degrees
        self.speed  = 20                # percent

        # Kick off jogging
        self.jog_joint()

    def jog_joint(self):
        # 1) Read the current angle(s)
        angles = self.mc.get_angles()  # might be int or list[str]/list[float]
        try:
            # if it's a list, take the first element
            current = float(angles[0])
        except (TypeError, IndexError):
            # if it's a single int/float, just use it
            current = float(angles)

        # 2) Decide direction
        direction = 0 if self.target > current else 1
        self.get_logger().info(
            f'Joint 1: start {current:.2f}°, target {self.target:.2f}°, '
            f'dir={"+" if direction else "-"}, speed={self.speed}%'
        )

        # 3) Start continuous jog
        self.mc.jog_angle(self.jid, direction, self.speed)

        # 4) Poll until target reached
        while rclpy.ok():
            angles = self.mc.get_angles()
            try:
                current = float(angles[0])
            except (TypeError, IndexError):
                current = float(angles)

            if (direction == 1 and current >= self.target) or \
               (direction == 0 and current <= self.target):
                break
            time.sleep(0.05)

        # 5) Stop jogging
        self.get_logger().info(f'Joint 1 reached {current:.2f}° → stopping.')
        self.mc.stop()

def main(args=None):
    rclpy.init(args=args)
    node = Joint1VelocityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
