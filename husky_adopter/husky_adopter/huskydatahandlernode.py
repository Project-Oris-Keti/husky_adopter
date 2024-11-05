import time
import message_filters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
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

from mavros import mavlink as mavros_mavlink

MAV_TYPE_WHEELROBOT = 101
MAV_AUTOPILOT_HUSKY = 101

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

class HuskyDataHanderNode(Node):
    def __init__(self):
        super().__init__('HuskyDataHandlerNode')        
        
        self.target_system = 1
        self.target_component = 1
        
        self.gps_sub = message_filters.Subscriber(self, NavSatFix, '/gnss')
        self.gps_sub.registerCallback(self.send_global_position_int)
        self.imu_sub = message_filters.Subscriber(self, Imu, '/imu/data')
        self.imu_sub.registerCallback(self.send_attitude_and_heartbeat)
        '''self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.gps_sub, self.imu_sub],
            10,
            0.04,  # defines the delay (in seconds) with which messages can be synchronized
        )
        self.ts.registerCallback(self.send_global_position_int)        '''
        
        self.pub = self.create_publisher(Mavlink, '/uas1/mavlink_sink', 10)
        
    def send_attitude_and_heartbeat(self, imu:Imu):
        time_boot_ms = 1000
        #with open('/proc/uptime','r') as uptime:    
            #time_boot_ms = int(float(uptime.readline().split()[0]) * 1000)
            
        roll_x, pitch_y, yaw_z = euler_from_quaternion(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)        
        #print(yaw_z) 
        yaw_z = (-yaw_z + math.pi * 0.5) % (math.pi * 2)
        #yaw_z = self.compass
        #print(yaw_z)

        # publish ATTITUDE
        mavlink = mav.MAVLink(None,self.target_system,self.target_component)
        # attitude_encode(self, time_boot_ms: int, roll: float, pitch: float, yaw: float, rollspeed: float, pitchspeed: float, yawspeed: float)
        mav_msg = mavlink.attitude_encode(time_boot_ms, roll_x, pitch_y, yaw_z, 0, 0, 0)
        mav_msg.pack(mavlink)
        ros_msg = mavros_mavlink.convert_to_rosmsg(mav_msg)
        
        self.pub.publish(ros_msg)
        
        # publish HEARTBEAT
        mavlink = mav.MAVLink(None,self.target_system,self.target_component)
        # heartbeat_encode(self, type: int, autopilot: int, base_mode: int, custom_mode: int, system_status: int, mavlink_version: int = 3)        
        mav_msg = mavlink.heartbeat_encode(MAV_TYPE_WHEELROBOT, MAV_AUTOPILOT_HUSKY, 0, 0, 0)
        mav_msg.pack(mavlink)
        ros_msg = mavros_mavlink.convert_to_rosmsg(mav_msg)
        
        self.pub.publish(ros_msg)

    def send_global_position_int(self, gps:NavSatFix):
        #print(rclpy.time.Time.from_msg(gps.header.stamp), rclpy.clock.ROSClock().now())
        time_boot_ms = 1000
        #with open('/proc/uptime','r') as uptime:    
            #time_boot_ms = int(float(uptime.readline().split()[0]) * 1000)
            
        lat = int(gps.latitude * 10000000)
        lon = int(gps.longitude * 10000000)
        alt = int(gps.altitude * 1000)
        #lat = int(39.74777264724913 * 10000000)
        #lon = int(-105.00999763311616 * 10000000)
        #alt = int(1000 * 1000)
        
        print(lat, lon, alt)

        relative_alt = 0
        vx = 0
        vy = 0
        vz = 0
        
        # publish GLOBAL_POSITION_INT
        mavlink = mav.MAVLink(None,self.target_system,self.target_component)        
        # global_position_int_encode(self, time_boot_ms: int, lat: int, lon: int, alt: int, relative_alt: int, vx: int, vy: int, vz: int, hdg: int)
        mav_msg = mavlink.global_position_int_encode(time_boot_ms, lat, lon, alt, relative_alt, vx, vy, vz, 0)
        mav_msg.pack(mavlink)
        ros_msg = mavros_mavlink.convert_to_rosmsg(mav_msg)
        
        self.pub.publish(ros_msg)        

def main(args=None):
    print('hello!')
    rclpy.init(args=args)
    node = HuskyDataHanderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
