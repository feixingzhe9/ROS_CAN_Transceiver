#!/usr/bin/env python
# coding=utf-8
import sys
import rospy
import struct
import time
import json
from canMsg import canMsg
from std_msgs.msg import String
from mrobot_driver_msgs.msg import vci_can


global g_tx_total
global g_rx_total

g_tx_total = 0
g_rx_total = 0
global test_data
test_data = 0

global flag
flag = 1

def can_id_build():
    reserve = 0x0
    srcMacID = 0x01
    dstMacID = 0x20
    ack = 0x00
    funcID = 0x01
    sourceID = 0x03
    canID = reserve << 28 | srcMacID << 21 | dstMacID << 13 | ack << 12 | funcID << 8 | sourceID
    return canID

def trigger_test():
    global g_tx_total
    global test_data
    global flag
    if not flag:
        return
    can_msg = vci_can()
    can_msg.ID = can_id_build()#0x012a0103
    can_msg.DataLen = 5

    can_msg.Data = '\x00' + struct.pack(">i", test_data)
    #can_msg.Data = [0x00,0x01,0x02,0x03,0x04,0x05]
    print 'sent:' + repr(can_msg.Data)
    test_pub.publish(can_msg)
    flag = 0
    g_tx_total += 1
    print 'total sent: %d'%g_tx_total

def data_received(can_msg):
    global g_rx_total
    global flag
    global test_data
    a = canMsg(can_msg)
    flag = 1
    print 'received:'
    a.print_info()
    if not (test_data == struct.unpack(">1I", can_msg.Data[1:5])[0]):
        print 'ack frame not right error'
        print repr(can_msg.Data)
        exit(1) 
    g_rx_total += 1 
    print 'total recevied: %d'%g_rx_total
    test_data += 1
    

def main():
    rate = rospy.Rate(200) #10hz
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
    try:
        rospy.init_node('test', anonymous=True)
        rospy.Subscriber("rx_test_node", vci_can, data_received)
        test_pub = rospy.Publisher('tx_test_node', vci_can, queue_size=10)
        main()
    except Exception: #rospy.ROSInterruptException:
        rospy.logerr(sys.exc_info())
        rospy.loginfo("lost connect")
        exit(1)

