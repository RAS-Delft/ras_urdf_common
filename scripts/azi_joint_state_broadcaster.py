#!/usr/bin/env python3

'''
    Converts ship actuation (currently as float32multiarray type) to joint state type, depending on the ship model and how the actuation array is defined
    This information is further used by the robot state publisher to publish the tf2 transform
    '''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import time
import math
import numpy as np
#import tf2 ros message type
import argparse
import enum 

# import ras_ros_core_control_modules/tools/display_tools package  
import ras_ros_core_control_modules.tools.display_tools as display_tools

parser = argparse.ArgumentParser()
parser.add_argument("vesselid", type=str,help="set vessel identifier")
parser.add_argument("-inactive_zero_angle_rate", type=str,help="set rate of publishing zero angle when no actuation is received, generally for display purposes. default is zero, meaning no publishing")
parser.add_argument("-r",help='ROS 2 arguments') # ROS2 arguments
parser.add_argument("-t", help='ship type',type=str)
args, unknown = parser.parse_known_args()
	
VESSEL_ID = args.vesselid
PRIO_MSG_TIMEOUT = 4.0
STATUS_TIMER_PERIOD = 4.0
PERIOD_INACTIVITY_BROADCAST = float(args.inactive_zero_angle_rate) if args.inactive_zero_angle_rate else 0.0
PERIOD_INACTIVITY_TIMEOUT = 5.0

## Enum class that defines four ship types: "Tito Neri", "Delfia", "Grey Seabax" and "Unknown"
class ShipType(enum.Enum):
    TitoNeri = 1
    Delfia = 2
    GreySeabax = 3

    Unknown = 0
    def string2type(name:str):
        """checks if name is a valid ship type name and returns the corresponding ship type enum value"""
        if name == "Tito Neri" or name == "tito neri" or name == "TitoNeri" or name == "titoneri":
            return ShipType.TitoNeri
        elif name == "Delfia" or name == "delfia":
            return ShipType.Delfia
        elif name == "Grey Seabax" or name == "grey seabax" or name == "GreySeabax" or name == "greyseabax":
            return ShipType.GreySeabax
        else:
            return ShipType.Unknown

if args.t:
    SHIP_TYPE = ShipType.string2type(args.t)
else:
    SHIP_TYPE = ShipType.string2type("unknown")

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def zero_jointstate_msg(rostime):
    joint_state_msg = JointState()
    joint_state_msg.header.stamp = rostime
    if SHIP_TYPE == ShipType.TitoNeri:
            joint_state_msg.name = [VESSEL_ID+'_SB_aft_thruster_propeller',VESSEL_ID+'_PS_aft_thruster_propeller',VESSEL_ID+'_BOW_thruster_propeller',VESSEL_ID+'_SB_aft_thruster_joint', VESSEL_ID+'_PS_aft_thruster_joint']
            joint_state_msg.position = [np.nan,np.nan,np.nan,0.0, 0.0]
            joint_state_msg.velocity = [0.0,0.0,0.0,np.nan,np.nan]
            joint_state_msg.effort = []
    elif SHIP_TYPE == ShipType.Delfia:
        print('[joint state broadcaster]: Delfia ship type not implemented yet')
    elif SHIP_TYPE == ShipType.GreySeabax:
        print('[joint state broadcaster]: GreySeabax ship type not implemented yet')
    return joint_state_msg

class ras_ships_actuation_to_joint_state_broadcaster(Node):
    '''
    Converts ship actuation (float32multiarray) to joint state (jointstate), depending on the ship type
    This information is further used by the robot state publisher to publish the tf2 transform
    '''
    def __init__(self):
        super().__init__('vessel_azi_tf2_broadcaster')
        self.subscription = self.create_subscription(Float32MultiArray,'reference/actuation',self.msg_callback_normal,10)
        self.subscription = self.create_subscription(Float32MultiArray,'reference/actuation_prio',self.msg_callback_prio,10)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.last_prio_msg_timestamp = 0.0
        self.last_msg_timestamp = 0.0

        self.statustimer = self.create_timer(STATUS_TIMER_PERIOD, self.status_callback)
        if PERIOD_INACTIVITY_BROADCAST > 0.0:
            self.inactivity_timer = self.create_timer(PERIOD_INACTIVITY_BROADCAST, self.inactivity_callback)
        self.status_publish_jointstate_counter = 0

        # Send one initial message to give zero angle as starting pose of thrusters (otherwise the robot state publisher will not publish the tf2 transform)
        #self.joint_state_publisher.publish(zero_jointstate_msg(self.get_clock().now().to_msg()))

    def inactivity_callback(self):
        now = time.time()
        if now - self.last_msg_timestamp > PERIOD_INACTIVITY_TIMEOUT:
            self.joint_state_publisher.publish(zero_jointstate_msg(self.get_clock().now().to_msg()))

    def msg_callback_normal(self, msg):
        now = time.time()
        if now - self.last_prio_msg_timestamp > PRIO_MSG_TIMEOUT:
            self.pub_jointstate(msg)
            self.last_msg_timestamp = time.time()
    
    def msg_callback_prio(self, msg):
        self.pub_jointstate(msg)
        self.last_prio_msg_timestamp = time.time()
        self.last_msg_timestamp = self.last_prio_msg_timestamp

    def pub_jointstate(self, msg:Float32MultiArray):
        self.status_publish_jointstate_counter += 1
        # Create joint state message

        # Joint names:
        # RAS_TN_DB_SB_aft_thruster_joint ->1
        # RAS_TN_DB_PS_aft_thruster_joint ->2

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        if SHIP_TYPE == ShipType.TitoNeri:
            joint_state_msg.name = [VESSEL_ID+'_SB_aft_thruster_propeller',VESSEL_ID+'_PS_aft_thruster_propeller',VESSEL_ID+'_BOW_thruster_propeller',VESSEL_ID+'_SB_aft_thruster_joint', VESSEL_ID+'_PS_aft_thruster_joint']
            joint_state_msg.position = [np.nan,np.nan,np.nan,msg.data[3], msg.data[4]]
            joint_state_msg.velocity = [msg.data[0],msg.data[1],msg.data[2],np.nan,np.nan]
            joint_state_msg.effort = []
        elif SHIP_TYPE == ShipType.Delfia:
            print('[joint state broadcaster]: Delfia ship type not implemented yet')
        elif SHIP_TYPE == ShipType.GreySeabax:
            print('[joint state broadcaster]: GreySeabax ship type not implemented yet')
        else:
            print('[joint state broadcaster]: Unknown ship type')

        # Publish joint state message
        self.joint_state_publisher.publish(joint_state_msg)

    def status_callback(self):

        # Calculate rate
        jointstate_rate = self.status_publish_jointstate_counter / STATUS_TIMER_PERIOD
        
        # Convert to strings
        printstring = display_tools.terminal_fleet_module_string(VESSEL_ID, ['jointstate_rate',jointstate_rate,'hz'])

        # Print
        self.get_logger().info(printstring)

        # Reset trackers
        self.status_publish_jointstate_counter = 0

def main(args=None):
    rclpy.init(args=args)
    node = ras_ships_actuation_to_joint_state_broadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
	main()

