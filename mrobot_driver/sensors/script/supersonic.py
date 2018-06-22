#!/usr/bin/env python
# coding=utf-8

import rospy
import struct
import json
import sys,time
from canMsg import canMsg
from std_msgs.msg import String
from mrobot_driver_msgs.msg import vci_can

global supersonic_id
supersonic_id = 0

def can_id_build(id):
    reserve = 0x0
    srcMacID = 0x01
    dstMacID = 0x60 + id
    ack = 0x00
    funcID = 0x02
    sourceID = 0x80
    canID = reserve << 28 | srcMacID << 21 | dstMacID << 13 | ack << 12 | funcID << 8 | sourceID
    return canID

def trigger_supersonic_detect():
    global supersonic_id
    supersonic_id += 1
    if supersonic_id > 8:
        supersonic_id = 1
    can_msg = vci_can()
    can_msg.ID = can_id_build(supersonic_id)
    can_msg.DataLen = 3
    can_msg.Data = [0x00,0x10,0xFF]
#rospy.loginfo(can_msg.Data)
    supersonic_pub.publish(can_msg)

def data_received(can_msg):
    a = canMsg(can_msg)
#a.print_info()
#print 'src_id: 0x%02x disatace: %dcm' %(a.get_srcMacID(), struct.unpack(">1B", can_msg.Data[1])[0])
#print 'distance: 0x%xcm'%(struct.unpack(">1B", can_msg.Data[1])[0])
#    rospy.loginfo("received data")
#    rospy.loginfo("ID:%08x"%data.ID)
#    rospy.loginfo("DataLen:%d"%data.DataLen)
    #rospy.loginfo("distance: %dcm"%struct.unpack("<I", can_msg.Data)[0])
#sources_dict['is_update'] = 1
    if a.get_srcMacID() == 0x61:
        sources_dict['src_id_61'] = struct.unpack(">1B", can_msg.Data[1])[0]
    elif a.get_srcMacID() == 0x62:
        sources_dict['src_id_62'] = struct.unpack(">1B", can_msg.Data[1])[0]
    elif a.get_srcMacID() == 0x63:
        sources_dict['src_id_63'] = struct.unpack(">1B", can_msg.Data[1])[0]
    elif a.get_srcMacID() == 0x64:
        sources_dict['src_id_64'] = struct.unpack(">1B", can_msg.Data[1])[0]
    elif a.get_srcMacID() == 0x65:
        sources_dict['src_id_65'] = struct.unpack(">1B", can_msg.Data[1])[0]
    elif a.get_srcMacID() == 0x66:
        sources_dict['src_id_66'] = struct.unpack(">1B", can_msg.Data[1])[0]
    elif a.get_srcMacID() == 0x67:
        sources_dict['src_id_67'] = struct.unpack(">1B", can_msg.Data[1])[0]
    elif a.get_srcMacID() == 0x68:
        sources_dict['src_id_68'] = struct.unpack(">1B", can_msg.Data[1])[0]
    sources_obj = json.dumps(sources_dict, sort_keys=True, indent=4, separators=(',',': '), ensure_ascii=True)
    print sources_obj 
   
    

def main():
    rate = rospy.Rate(10) #10hz
    while not rospy.is_shutdown():
        try:
            trigger_supersonic_detect()
            #rate.sleep()
        except Exception:
            rospy.logerr(sys.exc_info())

        rate.sleep()
    return


if __name__ == '__main__':
    supersonic_pub = None
    sources_dict = {"src_id_61":0, "src_id_62":0, "src_id_63":0, "src_id_64":0, "src_id_65":0, "src_id_66":0, "src_id_67":0, "src_id_68":0}
    sources_obj = json.dumps(sources_dict, ensure_ascii=True)
    try:
        rospy.init_node('supersonic', anonymous=True)
        rospy.Subscriber("can_to_ultrasonic", vci_can, data_received)
        supersonic_pub = rospy.Publisher('ultrasonic_to_can', vci_can, queue_size=10)
        time.sleep(1)
        main()
    except Exception: #rospy.ROSInterruptException:
        rospy.logerr(sys.exc_info())
        rospy.loginfo("lost connect")
        exit(1)

