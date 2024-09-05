import time
import message_filters
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, NavSatFix, BatteryState
import pymavlink.dialects.v20.standard as mav
from mavros_msgs.msg import Mavlink
import math

import struct
import typing

import rclpy.time
from pymavlink import mavutil
from pymavlink.generator.mavcrc import x25crc  # noqa F401
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

from ros2_mscl_interfaces.msg import GNSSFixInfo
from ghost_manager_interfaces.msg import Heartbeat

class DummyGPSNode(Node):
    def __init__(self):
        super().__init__('DummyGPSNode')        

        self.gps_pub = self.create_publisher(NavSatFix, '/gx5/gnss1/fix', 10)

    def publish_dummy_gps(self):
        self.gps_pub.publish(NavSatFix(header=Header(), latitude=0.0, longitude=0.0, altitude=0.0))


def main(args=None):
    print('hello!')
    rclpy.init(args=args)
    node = DummyGPSNode()

    while True:
        node.publish_dummy_gps()
        time.sleep(1)


if __name__ == '__main__':
    main()
