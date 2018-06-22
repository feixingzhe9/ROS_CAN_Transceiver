#!/usr/bin/env python
# coding=utf-8
import sys
import rospy
import struct
import time
import json
from canMsg import canMsg
from std_msgs.msg import String
from std_msgs.msg import UInt8MultiArray
from mrobot_driver_msgs.msg import vci_can

def can_id_build():
    reserve = 0x0
    srcMacID = 0x01
    dstMacID = 0x60
    ack = 0x00
    funcID = 0x02
    sourceID = 0x81
    canID = reserve << 28 | srcMacID << 21 | dstMacID << 13 | ack << 12 | funcID << 8 | sourceID
    return canID

def trigger_test():
    can_msg = vci_can()
    can_msg.ID = can_id_build()#0x012a0103
    can_msg.DataLen = 4

    can_msg.Data = [0x00,0x00,0x00,0x00]
    print 'sent:' + repr(can_msg.Data)
    test_pub.publish(can_msg)

def data_received(can_msg):
    global sensor_array
    a = canMsg(can_msg)
    print 'received:'
    a.print_info()
    if can_msg.Data[0] == '\x40':
        for i in range(0, 7):
            sensor_array.data[i] = a.get_data()[i+1]
    elif can_msg.Data[0] == '\x81':
        for i in range(0, 7):
            sensor_array.data[i+7] = a.get_data()[i+1]
    elif can_msg.Data[0] == '\x82':
        for i in range(0, 7):
            sensor_array.data[i+14] = a.get_data()[i+1]
    elif can_msg.Data[0] == '\xC3':
        sensor_array.data[21] = can_msg.Data[1]

    pub2starline_pub.publish(sensor_array)

def main():
    global sensor_array
    sensor_array = UInt8MultiArray()
    sensor_array.data = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    rate = rospy.Rate(10) #10hz
    time.sleep(1)
    while not rospy.is_shutdown():
        try:
            trigger_test()
            rate.sleep()
        except Exception:
            rospy.logerr(sys.exc_info())
        rate.sleep()
    return


if __name__ == '__main__':
    test_pub= None
    pub2starline_pub = None
    try:
        rospy.init_node('sensor', anonymous=True)
        rospy.Subscriber("rx_sensor_node", vci_can, data_received)
        test_pub = rospy.Publisher('tx_sensor_node', vci_can, queue_size=10)
        pub2starline_pub = rospy.Publisher('sensor_to_starline_node', UInt8MultiArray, queue_size=10)
        main()
    except Exception: #rospy.ROSInterruptException:
        rospy.logerr(sys.exc_info())
        rospy.loginfo("lost connect")
        exit(1)

