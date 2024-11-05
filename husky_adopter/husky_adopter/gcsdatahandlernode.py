import rclpy
from rclpy.node import Node
import pymavlink.dialects.v20.standard as mav
from std_msgs.msg import String, UInt32
from mavros_msgs.msg import Mavlink
import struct
import typing
import rclpy.time
from pymavlink import mavutil
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Vector3, Twist
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
        self.ghost_host = '192.168.168.105'
        self.ghost_port = 22
        self.ghost_id = 'ghost'
        self.ghost_pwd = 'ghost'
        self.control_mode = 180
        self.si_units = 0
        #self.mission_idx = 1
        self.v60_move_mode = {
            0: 'estop',
            1: 'si_units',
            2: 'high_step',
            3: 'blind_stairs',
            4: 'run',
            5: 'dock',
            6: 'hill',
            7: 'sand'
        }

        self.mavlink_sub = self.create_subscription(Mavlink, '/uas1/mavlink_source',self.gcs_data_handler,QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
            ))
        
        self.mavlink_pub = self.create_publisher(Mavlink, '/uas1/mavlink_sink', 10)
        #self.v60_mission_pub = self.create_publisher(String, '/mm/run_mission', 10)
        #self.v60_twist_pub = self.create_publisher(Twist, '/mcu/command/manual_twist', 10)

        #self.v60_vision_mode_pub = self.create_publisher(UInt32, '/command/setVisionMode',10)
        
        #self.v60_move_mode_pub = {}
        #for idx in move_mode_topic:
        #    self.v60_move_mode_pub[idx] = self.create_publisher(UInt32, move_mode_topic[idx],10)

    
    def gcs_data_handler(self, mavros_data: Mavlink):
        mavlink = mav.MAVLink(None,self.target_system,self.target_component)
        mavlink_message = mavlink.parse_char(mavros_mavlink.convert_to_bytes(mavros_data))
        
        mavlink_message_dict = mavlink_message.to_dict()
        id = mavlink_message.id
        print(mavlink_message_dict)
        
        if id == mav.MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: 
            lat = float(mavlink_message_dict['lat_int'])/10000000
            lon = float(mavlink_message_dict['lon_int'])/10000000
            
            # publish GPS_POSITION
        
        '''if id == mav.MAVLINK_MSG_ID_MISSION_CLEAR_ALL:  
            # publish MISSION_ACK
            mavlink = mav.MAVLink(None,self.target_system,self.target_component)
            # mission_ack_encode(self, target_system: int, target_component: int, type: int, mission_type: int = 0)
            mav_msg = mavlink. mission_ack_encode(self.target_system, self.target_component, mav.MAV_MISSION_ACCEPTED, mav.MAV_MISSION_TYPE_MISSION)
            mav_msg.pack(mavlink)
            self.mission_count = 0
            ros_msg = convert_to_rosmsg(mav_msg)
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
                ros_msg = convert_to_rosmsg(mav_msg)
                
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
                self.waypoint.append((lat,lon))
                
                self.cur_mission_index += 1
                
                if self.cur_mission_index < self.mission_count and self.mission_count > 0:                

                    # publish MISSION_REQUEST_INT
                    mavlink = mav.MAVLink(None,self.target_system,self.target_component)
                    # mission_request_int_encode(self, target_system: int, target_component: int, seq: int, mission_type: int = 0)
                    mav_msg = mavlink. mission_request_encode(self.target_system, self.target_component, self.cur_mission_index, mav.MAV_MISSION_TYPE_MISSION)
                    mav_msg.pack(mavlink)
                    ros_msg = convert_to_rosmsg(mav_msg)
                    
                    self.mavlink_pub.publish(ros_msg)
                else:
                    # publish MISSION_ACK
                    mavlink = mav.MAVLink(None,self.target_system,self.target_component)
                    # mission_ack_encode(self, target_system: int, target_component: int, type: int, mission_type: int = 0)
                    mav_msg = mavlink. mission_ack_encode(self.target_system, self.target_component, mav.MAV_MISSION_ACCEPTED, mav.MAV_MISSION_TYPE_MISSION)
                    mav_msg.pack(mavlink)
                    ros_msg = convert_to_rosmsg(mav_msg)
                    
                    self.mavlink_pub.publish(ros_msg)

                    # execute mission                    
                    path = ''
                    for lat, lon in self.waypoint:
                        path += f'PathMoveToGPS({lat},{lon},0.0)\n'
                    
                    v60_mission_script = '\n'.join(
                        [
                            'Include(Odometry/ReadyGPSNavigation.txt)',
                            'UseOdometry(GPS, 60)',
                            'ControlMode = 170',
                            'Action = 2',
                            'PlannerEnable = 1',
                            'PlannerCmd = 0 ',
                            'SetPathTopic(/mpc/goal_path)',
                            'SetCurPathIndTopic(/mpc/current_path_index)',
                            'MPCSetup(/activate_mpc_gps)\n',
                            'sleep(2)',
                            #'ControlMode = 170',
                            path,
                            'PathPub(3600)\n',
                            'PlannerEnable = 0',
                            'PlannerCmd = 0',
                            'Action = 0',
                            'MPCSetup()\n',
                            #'ControlMode = 180',
                        ]
                    )
                    
                    print(v60_mission_script)
                    
                    """ssh_client = paramiko.SSHClient()
                    ssh_client.load_system_host_keys()
                    ssh_client.connect(self.ghost_host, self.ghost_port, self.ghost_id, self.ghost_pwd)
                    
                    with ssh_client.open_sftp() as sftp:
                        try:
                            sftp.mkdir('/home/ghost/Missions/Missions/GCS')
                        except:
                            pass
                        with sftp.file('/home/ghost/Missions/Missions/GCS/waypoint.txt', 'w') as remote_file:
                            remote_file.write(v60_mission_script)
                            remote_file.flush()
                            remote_file.close()
                        sftp.close()
                    ssh_client.close()"""

                    #with open('/waypoint.txt','w') as file:
                    #    file.writelines(v60_mission_script)
                    
                    #os.system('sh /send_mission.sh')

                    #with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
                    #    client_socket.connect(('192.168.168.105', 9999))  # 서버에 접속
                    #    client_socket.send(v60_mission_script)  # 서버에 메시지 전송

                    msg = String()
                    msg.data = v60_mission_script
                    self.v60_missionfile_pub.publish(msg)

                    print('write mission file')
                   
                    time.sleep(1) 

                    msg = String()
                    msg.data = 'GCS/waypoint.txt'
                    self.v60_mission_pub.publish(msg)
                    
                    print('mission start')

                    #clear variables                    
                    self.mission_count = 0
                    self.cur_mission_index = 0                    
                    self.waypoint.clear()

        elif id == mav.MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
            if self.control_mode == 170:
                if self.si_units != 0:
                    #set blind move for relative twist
                    self.ensure_mode_node.ensure_mode('si_units', 0)
                    self.si_units = 0

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
                print(forward_normalized, side_normalized, yaw_normalized)
                self.v60_twist_pub.publish(Twist(linear=Vector3(x=forward_normalized, y=side_normalized), angular=Vector3(z=yaw_normalized)))
            elif self.control_mode == 180:
                if mavlink_message_dict['chan6_raw'] > 1800:
                    self.ensure_mode_node.ensure_mode('control_mode', 170)
                    self.control_mode = 170
                    '''

def main(args=None):
    print('hello!')
    rclpy.init(args=None)
    node = GCSDataHanderNode()    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
