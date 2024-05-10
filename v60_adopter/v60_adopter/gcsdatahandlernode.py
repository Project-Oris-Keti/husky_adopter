import rclpy
from rclpy.node import Node
import pymavlink.dialects.v20.standard as mav
from std_msgs.msg import String
from mavros_msgs.msg import Mavlink
import struct
import typing
import rclpy.time
from pymavlink import mavutil
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from ghost_manager_interfaces.srv import EnsureMode
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import paramiko
from ghost_manager_interfaces.msg import Heartbeat
from geometry_msgs.msg import Vector3, Twist
import time
import pysftp
import os
import socket

def convert_to_payload64(
    payload_bytes: typing.Union[bytes, bytearray]
) -> typing.List[int]:
    """Convert payload bytes to Mavlink.payload64."""
    payload_bytes = bytearray(payload_bytes)
    payload_len = len(payload_bytes)
    payload_octets = int(payload_len / 8)
    if payload_len % 8 > 0:
        payload_octets += 1
        payload_bytes += b"\0" * (8 - payload_len % 8)

    return struct.unpack(f"<{payload_octets}Q", payload_bytes)

def convert_to_rosmsg(
    mavmsg, stamp = None
) -> Mavlink:
    """
    Convert pymavlink message to Mavlink.msg.

    Currently supports both MAVLink v1.0 and v2.0,
    but without signing.
    """
    if stamp is not None:
        header = Header(stamp=stamp)
    else:
        stamp = Time()
        stamp.sec, stamp.nanosec = rclpy.clock.Clock().now().seconds_nanoseconds()
        header = Header(stamp=stamp)

    if mavutil.mavlink20():
        # XXX Need some api to retreive signature block.
        if mavmsg.get_signed():
            raise ValueError("Signed message can't be converted to rosmsg.")       
                 
        hdr = mavmsg.get_header()
        return Mavlink(
            header=header,
            framing_status=Mavlink.FRAMING_OK,
            magic=Mavlink.MAVLINK_V20,
            len=hdr.mlen,
            incompat_flags=hdr.incompat_flags,
            compat_flags=hdr.compat_flags,
            sysid=hdr.srcSystem,            
            compid=hdr.srcComponent,
            msgid=hdr.msgId,
            checksum=mavmsg.get_crc(),
            payload64=convert_to_payload64(mavmsg.get_payload()),
            signature=[],  # FIXME #569
        )

    else:
        return Mavlink(
            header=header,
            framing_status=Mavlink.FRAMING_OK,
            magic=Mavlink.MAVLINK_V10,
            len=len(mavmsg.get_payload()),
            seq=mavmsg.get_seq(),
            sysid=mavmsg.get_srcSystem(),
            compid=mavmsg.get_srcComponent(),
            msgid=mavmsg.get_msgId(),
            checksum=mavmsg.get_crc(),
            payload64=convert_to_payload64(mavmsg.get_payload()),
        )

def convert_to_bytes(msg: Mavlink) -> bytearray:
    """
    Re-builds the MAVLink byte stream from mavros_msgs/Mavlink messages.

    Support both v1.0 and v2.0.
    """
    payload_octets = len(msg.payload64)
    if payload_octets < msg.len / 8:
        raise ValueError("Specified payload length is bigger than actual payload64")

    if msg.magic == Mavlink.MAVLINK_V10:
        msg_len = 6 + msg.len  # header + payload length
        msgdata = bytearray(
            struct.pack(
                "<BBBBBB%dQ" % payload_octets,
                msg.magic,
                msg.len,
                msg.seq,
                msg.sysid,
                msg.compid,
                msg.msgid,
                *msg.payload64,
            )
        )
    else:  # MAVLINK_V20
        msg_len = 10 + msg.len  # header + payload length
        msgdata = bytearray(
            struct.pack(
                "<BBBBBBBBBB%dQ" % payload_octets,
                msg.magic,
                msg.len,
                msg.incompat_flags,
                msg.compat_flags,
                msg.seq,
                msg.sysid,
                msg.compid,
                msg.msgid & 0xFF,
                (msg.msgid >> 8) & 0xFF,
                (msg.msgid >> 16) & 0xFF,
                *msg.payload64,
            )
        )

    if payload_octets != msg.len / 8:
        # message is shorter than payload octets
        msgdata = msgdata[:msg_len]

    # finalize
    msgdata += struct.pack("<H", msg.checksum)

    if msg.magic == Mavlink.MAVLINK_V20:
        msgdata += bytearray(msg.signature)

    return msgdata

class GCSDataHanderNode(Node):
    def __init__(self, ensure_mode_node):
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
        self.planner_en = 0
        #self.mission_idx = 1

        self.ensure_mode_node = ensure_mode_node
        
        self.mavlink_sub = self.create_subscription(Mavlink, '/uas1/mavlink_source',self.gcs_data_handler,QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        ))
        self.heartbeat_sub = self.create_subscription(Heartbeat, '/state/heartbeat',self.heartbeat_monitor,10)

        self.mavlink_pub = self.create_publisher(Mavlink, '/uas1/mavlink_sink', 10)
        self.v60_mission_pub = self.create_publisher(String, '/mm/run_mission', 10)
        self.v60_twist_pub = self.create_publisher(Twist, '/mcu/command/manual_twist', 10)

        self.v60_missionfile_pub = self.create_publisher(String, '/keti_gcs/mission',10)

    def heartbeat_monitor(self, heartbeat: Heartbeat):
        self.control_mode = heartbeat.control_mode    
        self.planner_en = heartbeat.si_units
    
    def gcs_data_handler(self, mavros_data: Mavlink):
        mavlink = mav.MAVLink(None,self.target_system,self.target_component)
        mavlink_message = mavlink.parse_char(convert_to_bytes(mavros_data))
        
        mavlink_message_dict = mavlink_message.to_dict()
        id = mavlink_message.id
        #print(mavlink_message_dict)
        
        if id == mav.MAVLINK_MSG_ID_COMMAND_LONG:
            command = mavlink_message_dict['command']

            if command == mav.MAV_CMD_DO_SET_MODE:
                action = int(mavlink_message_dict['param2'])

                if action >= 0 and action <= 2:
                    # change control mode
                    self.ensure_mode_node.ensure_mode('control_mode', 140)
                
                    # change action
                    self.ensure_mode_node.ensure_mode('action',action)
                
                    # recovery control mode
                    self.ensure_mode_node.ensure_mode("control_mode", 180)
            elif command == mav.MAV_CMD_USER_1:
                mode = int(mavlink_message_dict['param1'])
                # change control mode
                self.ensure_mode_node.ensure_mode('control_mode', mode)

        elif id == mav.MAVLINK_MSG_ID_MISSION_CLEAR_ALL:  
            # publish MISSION_ACK
            mavlink = mav.MAVLink(None,self.target_system,self.target_component)
            # mission_ack_encode(self, target_system: int, target_component: int, type: int, mission_type: int = 0)
            mav_msg = mavlink. mission_ack_encode(self.target_system, self.target_component, mav.MAV_MISSION_ACCEPTED, mav.MAV_MISSION_TYPE_MISSION)
            mav_msg.pack(mavlink)
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
                if self.planner_en != 0:
                    #set blind move for relative twist
                    self.ensure_mode_node.ensure_mode('planner_en', 0)
                    self.planner_en =0

                max_val = 500.0
                default_val = 1500
                default_val_side = 1495
                forward_normalized = float(mavlink_message_dict['chan3_raw'] - default_val) / max_val
                #left is positive value
                side_normalized = -float(mavlink_message_dict['chan4_raw'] - default_val_side) / max_val
                #left is positive value
                yaw_normalized = -float(mavlink_message_dict['chan1_raw'] - default_val) / max_val
                self.v60_twist_pub.publish(Twist(linear=Vector3(x=forward_normalized, y=side_normalized), angular=Vector3(z=yaw_normalized)))
            elif self.control_mode == 180:
                if mavlink_message_dict['chan6_raw'] > 1800:
                    self.ensure_mode_node.ensure_mode('control_mode', 170)
                    self.control_mode = 170

class EnsureModeNode(Node):
    def __init__(self):
        super().__init__('EnsureModeNode')            
        self.ensure_mode_srv = self.create_client(EnsureMode, 'ensure_mode')
        while not self.ensure_mode_srv.wait_for_service(timeout_sec=2.0):
            print('service not available, waiting again...')
        self.ensure_mode_req = EnsureMode.Request()
            
    def ensure_mode(self, field_name, valdes):
        self.ensure_mode_req.field = field_name
        self.ensure_mode_req.valdes = valdes
        future = self.ensure_mode_srv.call_async(self.ensure_mode_req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        print(response.result_str)

def main(args=None):
    print('hello!')
    rclpy.init(args=None)
    ensure_mode_node = EnsureModeNode()    
    node = GCSDataHanderNode(ensure_mode_node)    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
