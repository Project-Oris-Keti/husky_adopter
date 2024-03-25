import time
import message_filters
import rclpy
from rclpy.node import Node
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

class V60DataHanderNode(Node):
    def __init__(self):
        super().__init__('V60DataHandlerNode')        
        
        self.target_system = 1
        self.target_component = 1
        
        self.gps_sub = message_filters.Subscriber(self, NavSatFix, '/gx5/gnss1/fix')
        self.imu_sub = message_filters.Subscriber(self, Imu, '/mcu/state/imu')
        self.imu_sub.registerCallback(self.send_attitude)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.gps_sub, self.imu_sub],
            10,
            0.04,  # defines the delay (in seconds) with which messages can be synchronized
        )
        self.ts.registerCallback(self.send_global_position_int)
        
        self.battery_sub = self.create_subscription(BatteryState, '/mcu/state/battery',self.get_battery_info,10)
        self.fixinfo_sub = self.create_subscription(GNSSFixInfo, '/gx5/gnss1/fixInfo',self.send_gps_raw_int,10)
        self.heartbeat_sub = self.create_subscription(Heartbeat, '/state/heartbeat',self.send_sys_status_and_heartbeat,10)        
        
        self.pub = self.create_publisher(Mavlink, '/uas1/mavlink_sink', 10)
        
        self.voltage = 0
        self.percentage = 0

    def get_battery_info(self, battery: BatteryState):
        self.voltage = int(battery.voltage * 1000)
        self.percentage = int(battery.percentage)
        if self.percentage >= 100:
            self.percentage = 100
        
    def send_attitude(self, imu:Imu):
        time_boot_ms = 0
        with open('/proc/uptime','r') as uptime:    
            time_boot_ms = int(float(uptime.readline().split()[0]) * 1000)
            
        roll_x, pitch_y, yaw_z = euler_from_quaternion(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)        
        #print(yaw_z) 
        yaw_z = ((yaw_z % (math.pi * 2)) + math.pi * 2) % (math.pi * 2)

        # publish ATTITUDE
        mavlink = mav.MAVLink(None,self.target_system,self.target_component)
        # attitude_encode(self, time_boot_ms: int, roll: float, pitch: float, yaw: float, rollspeed: float, pitchspeed: float, yawspeed: float)
        mav_msg = mavlink.attitude_encode(time_boot_ms, roll_x, -pitch_y, yaw_z, 0, 0, 0)
        mav_msg.pack(mavlink)
        ros_msg = convert_to_rosmsg(mav_msg)
        
        self.pub.publish(ros_msg)

    def send_gps_raw_int(self, gnss_fix_info : GNSSFixInfo):        
        num_sv = gnss_fix_info.num_sv
        
        # publish GPS_RAW_INT
        mavlink = mav.MAVLink(None,self.target_system,self.target_component)        
        # gps_raw_int_encode(self, time_usec: int, fix_type: int, lat: int, lon: int, alt: int, eph: int, epv: int, vel: int, cog: int, satellites_visible: int, alt_ellipsoid: int = 0, h_acc: int = 0, v_acc: int = 0, vel_acc: int = 0, hdg_acc: int = 0, yaw: int = 0)
        mav_msg = mavlink.gps_raw_int_encode(0,0,0,0,0,0,0,0,0,0,num_sv)
        mav_msg.pack(mavlink)
        ros_msg = convert_to_rosmsg(mav_msg)
        
        self.pub.publish(ros_msg)

    def send_sys_status_and_heartbeat(self, heartbeat: Heartbeat):
        # 0: sit, 1: stand, 2: walk
        action = heartbeat.action
        
        mode = 0
        if action == 0:
            mode = 0
        elif action == 1:
            mode = 1
        elif action == 2:
            mode = 2
        
        # publish HEARTBEAT
        mavlink = mav.MAVLink(None,self.target_system,self.target_component)
        # heartbeat_encode(self, type: int, autopilot: int, base_mode: int, custom_mode: int, system_status: int, mavlink_version: int = 3)        
        mav_msg = mavlink.heartbeat_encode(mav.MAV_TYPE_QUADROTOR, mav.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, mode, 0)
        mav_msg.pack(mavlink)
        ros_msg = convert_to_rosmsg(mav_msg)
        
        self.pub.publish(ros_msg)
                        
        # estop, planner_en, high_step, blind_stairs, run, dock, roll_over, hill, sand, soft_estop
        info_pack = f'000000{heartbeat.estop}{heartbeat.planner_en}{heartbeat.high_step}{heartbeat.blind_stairs}{heartbeat.run}{heartbeat.dock}{heartbeat.roll_over}{heartbeat.hill}{heartbeat.sand}{heartbeat.soft_estop}'
        #print(info_pack)

        # publish SYS_STATUS
        mavlink = mav.MAVLink(None,self.target_system,self.target_component)        
        # sys_status_encode(self, onboard_control_sensors_present: int, onboard_control_sensors_enabled: int, onboard_control_sensors_health: int, load: int, voltage_battery: int, current_battery: int, battery_remaining: int, drop_rate_comm: int, errors_comm: int, errors_count1: int, errors_count2: int, errors_count3: int, errors_count4: int)
        mav_msg = mavlink.sys_status_encode(0,0,0,int(info_pack,2),self.voltage,heartbeat.planner_cmd,self.percentage,heartbeat.control_mode,0,0,0,0,0)
        mav_msg.pack(mavlink)
        #print(mav_msg.to_dict())
        ros_msg = convert_to_rosmsg(mav_msg)
        
        self.pub.publish(ros_msg)

    def send_global_position_int(self, gps:NavSatFix, imu:Imu):
        #print(rclpy.time.Time.from_msg(gps.header.stamp), rclpy.clock.ROSClock().now())
        time_boot_ms = 0
        with open('/proc/uptime','r') as uptime:    
            time_boot_ms = int(float(uptime.readline().split()[0]) * 1000)
            
        lat = int(gps.latitude * 10000000)
        lon = int(gps.longitude * 10000000)
        alt = int(gps.altitude * 1000)
        relative_alt = 0
        vx = 0
        vy = 0
        vz = 0
        
        _, _, yaw_z = euler_from_quaternion(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)      
        
        hdg = int(yaw_z * 180 / math.pi * 100)
        if hdg < 0:
            hdg += 36000
        
        # publish GLOBAL_POSITION_INT
        mavlink = mav.MAVLink(None,self.target_system,self.target_component)        
        # global_position_int_encode(self, time_boot_ms: int, lat: int, lon: int, alt: int, relative_alt: int, vx: int, vy: int, vz: int, hdg: int)
        mav_msg = mavlink.global_position_int_encode(time_boot_ms, lat, lon, alt, relative_alt, vx, vy, vz, hdg)
        mav_msg.pack(mavlink)
        ros_msg = convert_to_rosmsg(mav_msg)
        
        self.pub.publish(ros_msg)        

def main(args=None):
    print('hello!')
    rclpy.init(args=args)
    node = V60DataHanderNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
