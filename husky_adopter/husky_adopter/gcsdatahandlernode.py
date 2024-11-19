import rclpy
from rclpy.node import Node
import pymavlink.dialects.v20.standard as mav
from std_msgs.msg import String, UInt32, Bool
from sensor_msgs.msg import Joy
from mavros_msgs.msg import Mavlink
import struct
import typing
import rclpy.time
from pymavlink import mavutil
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Vector3, Twist, PoseArray, Pose, Point
import time
import os
import socket

from mavros import mavlink as mavros_mavlink

class GCSDataHanderNode(Node):
    def __init__(self):
        super().__init__('GCSDataHandlerNode')

        self.target_system = 1
        self.target_component = 1
        self.cur_mission_index = 0
        self.mission_count = 0
        self.waypoint = []
        self.num_zero_cmd = 0

        self.mavlink_sub = self.create_subscription(Mavlink, '/uas1/mavlink_source',self.gcs_data_handler,QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
            ))
        
        self.mavlink_pub = self.create_publisher(Mavlink, '/uas1/mavlink_sink', 10)

        self.husky_move_pub = self.create_publisher(PoseArray, '/new_waypoints', QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10
            ))
        self.husky_twist_pub = self.create_publisher(Twist, '/joy_teleop/cmd_vel', 10)
        self.husky_hold_pub = self.create_publisher(Joy, '/joy_teleop/joy', 10)

        self.stop_pub = self.create_publisher(Bool, '/abandon_waypoints', QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10
            ))

    def gcs_data_handler(self, mavros_data: Mavlink):
        mavlink = mav.MAVLink(None,self.target_system,self.target_component)
        mavlink_message = mavlink.parse_char(mavros_mavlink.convert_to_bytes(mavros_data))
        
        mavlink_message_dict = mavlink_message.to_dict()
        id = mavlink_message.id
        print(mavlink_message_dict, id, mav.MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT)
        
        if id == mav.MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: 
            lat = float(mavlink_message_dict['lat_int'])/10000000
            lon = float(mavlink_message_dict['lon_int'])/10000000
            
            # publish GPS_POSITION
            pose = Pose(position=Point(x=lat, y=lon, z=0.0))
            poseArr = PoseArray(poses=[pose])

            print(poseArr)

            self.husky_move_pub.publish(poseArr)

        elif id == mav.MAVLINK_MSG_ID_COMMAND_LONG:
            command = mavlink_message_dict['command']

            if command == mav.MAV_CMD_DO_SET_MODE:
                custom_mode = int(mavlink_message_dict['param2'])

                if custom_mode == 5:
                    print('loiter')
                    self.stop_pub.publish(Bool(data=True))

        elif id == mav.MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:

            # print(mavlink_message_dict)
            max_val = 500.0
            default_val = 1500
            default_val_side = 1495
            scale = 1000.0
            mid = 2000
            #forward_normalized = float(mavlink_message_dict['chan3_raw'] - default_val) / max_val
            forward_normalized = float(mavlink_message_dict['chan3_raw'] - mid) / scale
            #left is positive value
            #side_normalized = -float(mavlink_message_dict['chan4_raw'] - default_val_side) / max_val
            side_normalized = float(mavlink_message_dict['chan4_raw'] - mid) / scale
            #left is positive value
            #yaw_normalized = -float(mavlink_message_dict['chan1_raw'] - default_val) / max_val
            yaw_normalized = float(mavlink_message_dict['chan1_raw'] - mid) / scale
            forward_normalized = 0.0 if abs(forward_normalized) <= 0.05 or abs(forward_normalized)> 1 else forward_normalized
            side_normalized = 0.0 if abs(side_normalized) <= 0.05 or abs(side_normalized) > 1 else side_normalized
            yaw_normalized = 0.0 if abs(yaw_normalized) <= 0.05 or abs(yaw_normalized) > 1 else yaw_normalized

            stop = int(mavlink_message_dict['chan5_raw'])

            if stop != 0:
                self.husky_hold_pub.publish(Joy(buttons=(0,0,0,0,stop,0,0,0,0,0,0,0,0)))

            if yaw_normalized == 0.0 and forward_normalized == 0.0 and side_normalized == 0.0:
                self.num_zero_cmd += 1
                if self.num_zero_cmd > 3:
                    #print('ok')
                    return
            else:
                self.num_zero_cmd = 0

            #print(forward_normalized, side_normalized, yaw_normalized)
            self.husky_twist_pub.publish(Twist(linear=Vector3(x=forward_normalized, y=side_normalized), angular=Vector3(z=yaw_normalized)))

        elif id == mav.MAVLINK_MSG_ID_MISSION_CLEAR_ALL:  
            # publish MISSION_ACK
            mavlink = mav.MAVLink(None,self.target_system,self.target_component)
            # mission_ack_encode(self, target_system: int, target_component: int, type: int, mission_type: int = 0)
            mav_msg = mavlink. mission_ack_encode(self.target_system, self.target_component, mav.MAV_MISSION_ACCEPTED, mav.MAV_MISSION_TYPE_MISSION)
            mav_msg.pack(mavlink)
            self.mission_count = 0
            ros_msg = mavros_mavlink.convert_to_rosmsg(mav_msg)
            self.mavlink_pub.publish(ros_msg)

        elif id == mav.MAVLINK_MSG_ID_MISSION_COUNT:
            if self.mission_count == 0:
                self.mission_count = mavlink_message_dict['count']
                self.cur_mission_index = 0
                self.mission_type = mavlink_message_dict['mission_type']
                
                # publish MISSION_REQUEST, not MISSION_REQUEST_INT
                mavlink = mav.MAVLink(None,self.target_system,self.target_component)
                # mission_request_int_encode(self, target_system: int, target_component: int, seq: int, mission_type: int = 0)
                #mav_msg = mavlink. mission_request_int_encode(self.target_system, self.target_component, self.seq, mav.MAV_MISSION_TYPE_MISSION)
                mav_msg = mavlink. mission_request_encode(self.target_system, self.target_component, self.cur_mission_index, mav.MAV_MISSION_TYPE_MISSION)
                mav_msg.pack(mavlink)
                ros_msg = mavros_mavlink.convert_to_rosmsg(mav_msg)
                
                print('ok')
                self.mavlink_pub.publish(ros_msg)
                
        elif id == mav.MAVLINK_MSG_ID_MISSION_ITEM_INT:            
            seq = mavlink_message_dict['seq']
            frame = mavlink_message_dict['frame']
            command = mavlink_message_dict['command']
            
            if command == mav.MAV_CMD_NAV_WAYPOINT:
                lat = float(mavlink_message_dict['x'])/10000000
                lon = float(mavlink_message_dict['y'])/10000000
                alt = mavlink_message_dict['z']
                self.waypoint.append(Pose(position=Point(x=lat, y=lon)))
                
                self.cur_mission_index += 1
                
                if self.cur_mission_index < self.mission_count and self.mission_count > 0:                

                    # publish MISSION_REQUEST_INT
                    mavlink = mav.MAVLink(None,self.target_system,self.target_component)
                    # mission_request_int_encode(self, target_system: int, target_component: int, seq: int, mission_type: int = 0)
                    mav_msg = mavlink. mission_request_encode(self.target_system, self.target_component, self.cur_mission_index, mav.MAV_MISSION_TYPE_MISSION)
                    mav_msg.pack(mavlink)
                    ros_msg = mavros_mavlink.convert_to_rosmsg(mav_msg)
                    
                    self.mavlink_pub.publish(ros_msg)
                else:
                    # publish MISSION_ACK
                    mavlink = mav.MAVLink(None,self.target_system,self.target_component)
                    # mission_ack_encode(self, target_system: int, target_component: int, type: int, mission_type: int = 0)
                    mav_msg = mavlink. mission_ack_encode(self.target_system, self.target_component, mav.MAV_MISSION_ACCEPTED, mav.MAV_MISSION_TYPE_MISSION)
                    mav_msg.pack(mavlink)
                    ros_msg = mavros_mavlink.convert_to_rosmsg(mav_msg)
                    
                    self.mavlink_pub.publish(ros_msg)

                    # execute mission         
                    poseArr = PoseArray(poses=self.waypoint)

                    print(poseArr)

                    self.husky_move_pub.publish(poseArr)
                    
                    #clear variables                    
                    self.mission_count = 0
                    self.cur_mission_index = 0                    
                    self.waypoint.clear()

def main(args=None):
    print('hello!')
    rclpy.init(args=None)
    node = GCSDataHanderNode()    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
