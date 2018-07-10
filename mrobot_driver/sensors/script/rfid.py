#!/usr/bin/env python
# coding=utf-8
import sys
import rospy
import struct
import time
import json
import binascii
from canMsg import canMsg
from std_msgs.msg import String
from std_msgs.msg import UInt8MultiArray
from mrobot_driver_msgs.msg import vci_can


global g_tx_total
global g_rx_total
RFID_NODE_VER = "V0.2"

g_tx_total = 0
g_rx_total = 0
global test_data
test_data = 0

global flag
flag = 1

def can_id_build():
    reserve = 0x0
    srcMacID = 0x01
    dstMacID = 0xd5
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
    rfid_pub.publish(can_msg)
    a = canMsg(can_msg)
    a.print_info()
    flag = 0
    g_tx_total += 1
    print 'total sent: %d'%g_tx_total

def data_received(can_msg):
    global g_rx_total
    global flag
    global test_data
    global rfid_uid
    global rfid_type
    global rfid_data
    a = canMsg(can_msg)
    flag = 1
    print 'received:'
    a.print_info()
    g_rx_total += 1
    print 'total recevied: %d'%g_rx_total
    test_data += 1
    if a.get_sourceID() == 0x81:
        #rfid_uid = struct.unpack(">1I", can_msg.Data[1:5])[0]
        rfid_uid = rfid_uid + str(binascii.b2a_hex(can_msg.Data[1:]))
        print 'rfid_uid:' + rfid_uid
    if a.get_sourceID() == 0x82:
        if a.get_data()[0] == 0x40:
            rfid_type = rfid_type + str(binascii.b2a_hex(can_msg.Data[1:]))
        if a.get_data()[0] == 0x81:
            rfid_type = rfid_type + str(binascii.b2a_hex(can_msg.Data[1:]))
        if a.get_data()[0] == 0xC2:
            rfid_type = rfid_type + str(binascii.b2a_hex(can_msg.Data[1:]))
            print 'rfid_type:' + rfid_type
    if a.get_sourceID() == 0x83:
        if a.get_data()[0] == 0x40:
            rfid_data = rfid_data + str(binascii.b2a_hex(can_msg.Data[1:]))
        if a.get_data()[0] == 0x81:
            rfid_data = rfid_data + str(binascii.b2a_hex(can_msg.Data[1:]))
        if a.get_data()[0] == 0xC2:
            rfid_data = rfid_data + str(binascii.b2a_hex(can_msg.Data[1:]))
            print 'rfid_data:' + rfid_data

            json_info = { "pub_name":"rfid_info", "data": { "type":rfid_type, "data":rfid_data, "key":rfid_uid}, "msg":"success", "error_code": "0" }
            json_rfid = json.dumps(json_info)
            print json_rfid
            string_rfid = String()
            string_rfid.data = json_rfid
            to_app_pub.publish(string_rfid)
            rfid_uid = ''
            rfid_type = ''
            rfid_data= ''

    if a.get_sourceID() == 0x85:
        hall_msg = UInt8MultiArray()
        hall_msg.data = a.get_data()[1:]
        to_hall_pub.publish(hall_msg)


def main():
    global rfid_uid
    global rfid_type
    global rfid_data
    rfid_uid = ''
    rfid_type = ''
    rfid_data = ''
    rate = rospy.Rate(10) #10hz
    time.sleep(1)
    while not rospy.is_shutdown():
        try:
            #trigger_test()
            rate.sleep()
        except Exception:
            rospy.logerr(sys.exc_info())
        rate.sleep()
    return


if __name__ == '__main__':
    rfid_pub = None
    to_app_pub = None
    to_hall_pub = None
    try:
        rospy.init_node('rfid', anonymous=True)
        rospy.set_param("rfid_node_version", RFID_NODE_VER)
        rospy.Subscriber("rx_rfid_node", vci_can, data_received)
        rfid_pub = rospy.Publisher('tx_rfid_node', vci_can, queue_size=10)
        to_app_pub = rospy.Publisher('rfid_pub', String, queue_size=10)
        to_hall_pub = rospy.Publisher('hall_to_starline_node', UInt8MultiArray, queue_size=10)
        main()
    except Exception: #rospy.ROSInterruptException:
        rospy.logerr(sys.exc_info())
        rospy.loginfo("lost connect")
        exit(1)

