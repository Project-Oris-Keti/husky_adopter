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

from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs

MAV_TYPE_WHEELROBOT = 101
MAV_AUTOPILOT_HUSKY = 101

coordinateFrameMapIdMapper = {
    "0": "0",
    "100" : "1" # 청라 테스트베드
}

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
        #self.gps_sub.registerCallback(self.send_global_position_int)
        self.imu_sub = message_filters.Subscriber(self, Imu, '/imu/data')
        #self.imu_sub.registerCallback(self.send_attitude_and_heartbeat)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.gps_sub, self.imu_sub],
            10,
            0.04,  # defines the delay (in seconds) with which messages can be synchronized
        )
        self.ts.registerCallback(self.send_global_position_int)        
        
        self.pub = self.create_publisher(Mavlink, '/uas1/mavlink_sink', 10)
        
        # tf2 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # odom 구독
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # 현재 위치 저장 변수
        self.odom_pose = None
        self.map_pose = None
        
        # 주기적으로 tf 확인
        self.timer = self.create_timer(0.1, self.update_tf_pose)
        
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
        mav_msg = mavlink.heartbeat_encode(MAV_TYPE_WHEELROBOT, MAV_AUTOPILOT_HUSKY, 0x80, 0, 0)
        mav_msg.pack(mavlink)
        ros_msg = mavros_mavlink.convert_to_rosmsg(mav_msg)
        
        self.pub.publish(ros_msg)

    def send_global_position_int(self, gps:NavSatFix, imu:Imu):
        #print(rclpy.time.Time.from_msg(gps.header.stamp), rclpy.clock.ROSClock().now())
        time_boot_ms = 1000
        #with open('/proc/uptime','r') as uptime:    
            #time_boot_ms = int(float(uptime.readline().split()[0]) * 1000)
            
        self.send_attitude_and_heartbeat(imu)

        lat = int(gps.latitude * 10000000)
        lon = int(gps.longitude * 10000000)
        alt = int(gps.altitude * 1000)
        #lat = int(39.74777264724913 * 10000000)
        #lon = int(-105.00999763311616 * 10000000)
        #alt = int(1000 * 1000)
        
        #print(lat, lon, alt)

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
        
    def odom_callback(self, msg):
        """Odometry 메시지에서 위치 업데이트"""
        self.odom_pose = msg.pose.pose
        
        position = self.get_position_as_dict()
        
        if position is None:
            return
        
        print(position)
        
        time_boot_ms = 1000
        
        # publish ODOMETRY
        mavlink = mav.MAVLink(None,self.target_system,self.target_component)
        # odometry_encode(self, time_usec: int, frame_id: int, child_frame_id: int, x: float, y: float, z: float, q: Sequence[float], vx: float, vy: float, vz: float, rollspeed: float, pitchspeed: float, yawspeed: float, pose_covariance: Sequence[float], velocity_covariance: Sequence[float], reset_counter: int = 0, estimator_type: int = 0, quality: int = 0)
        mav_msg = mavlink.odometry_encode(time_boot_ms, 100, 2, position['x'], position['y'], position['z'], [self.odom_pose.orientation.x, self.odom_pose.orientation.y, self.odom_pose.orientation.z, self.odom_pose.orientation.w], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, [0.0]*21, [0.0]*21, 1, 1, 1)
        mav_msg.pack(mavlink)
        ros_msg = mavros_mavlink.convert_to_rosmsg(mav_msg)
        
        self.pub.publish(ros_msg)
    
    def update_tf_pose(self):
        """tf를 통해 map 좌표계에서의 위치 업데이트"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            
            # transform을 pose로 변환
            pose = PoseStamped()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
                        
            self.map_pose = pose.pose
                        
        except Exception as e:
            self.get_logger().debug(f'Transform lookup failed: {str(e)}')
    
    def get_current_position(self, frame='map'):
        """현재 위치 반환"""
        if frame == 'map' and self.map_pose:
            return self.map_pose
        elif frame == 'odom' and self.odom_pose:
            return self.odom_pose
        else:
            return None
    
    def get_position_as_dict(self, frame='map'):
        """현재 위치를 딕셔너리로 반환"""
        pose = self.get_current_position(frame)
        if pose is None:
            return None
        
        # 쿼터니언을 오일러각으로 변환
        orientation = pose.orientation
        roll, pitch, yaw = euler_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        
        return {
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw
        }

def main(args=None):
    print('hello!')
    rclpy.init(args=args)
    node = HuskyDataHanderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
