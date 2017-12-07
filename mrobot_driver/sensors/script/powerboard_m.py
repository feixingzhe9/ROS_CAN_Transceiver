#!/usr/bin/env python
# coding=utf-8

import rospy
import struct
import json
from canMsg import canMsg
from std_msgs.msg import String
from mrobot_driver_msgs.msg import vci_can

def can_id_build():
    reserve = 0x0
    srcMacID = 0x00
    dstMacID = 0x00
    ack = 0x00
    funcID = 0x01
    sourceID = 0x80
    canID = reserve << 28 | srcMacID << 21 | dstMacID << 13 | ack << 12 | funcID << 8 | sourceID
    return canID

def trigger_powerboard_m_detect():
    can_msg = vci_can()
    can_msg.ID = 0x00260180 #can_id_build()
    can_msg.DataLen = 7
    can_msg.Data = [0x01,0x00,0x00,0x00,0x00,0x00,0x00]
    rospy.loginfo(can_msg.Data)
    power_m_pub.publish(can_msg)

def data_received(can_msg):
    can_msgg = can_msg
    a = canMsg(can_msgg)
    a.print_info()
    power_m_pub.publish(can_msg)
    #print 'distance:%dcm'%a.get_data()[0]
#    rospy.loginfo("received data")
#    rospy.loginfo("ID:%08x"%data.ID)
#    rospy.loginfo("DataLen:%d"%data.DataLen)
    #rospy.loginfo("distance: %dcm"%struct.unpack("<I", can_msg.Data)[0])
#    sources_dict['is_update'] = 1
#    sources_dict['distance_cm'] = a.get_data()[0]
#    sources_obj = json.dumps(sources_dict, sort_keys=True, indent=4, separators=(',',': '), ensure_ascii=True)
    #print sources_obj 
   
    

def main():
    rate = rospy.Rate(10) #10hz
    while not rospy.is_shutdown():
        try:
#            trigger_powerboard_m_detect()
            rate.sleep()
        except Exception:
            rospy.logerr(sys.exc_info())

        rate.sleep()
    return


if __name__ == '__main__':
    power_m_pub = None
    sources_dict = {"is_update":0, "distance_cm":0}
    sources_obj = json.dumps(sources_dict, ensure_ascii=True)
    try:
        rospy.init_node('powerboard_m', anonymous=True)
        rospy.Subscriber("rx_unknown_node", vci_can, data_received)
        power_m_pub = rospy.Publisher('tx_power_m_node', vci_can, queue_size=10)
        main()
    except Exception: #rospy.ROSInterruptException:
        rospy.logerr(sys.exc_info())
        rospy.loginfo("lost connect")
        exit(1)

