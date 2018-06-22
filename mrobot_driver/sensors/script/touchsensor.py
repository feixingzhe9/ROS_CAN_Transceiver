#!/usr/bin/python
#coding=utf-8
import rospy
import struct
from canMsg import canMsg
from std_msgs.msg import String
from mrobot_driver_msgs.msg import vci_can

def can_id_build():
    reserve = 0x0
    srcMacID = 0xFF
    dstMacID = 0x70
    ack = 0x01
    funcID = 0x01
    sourceID = 0x80
    canID = reserve << 28 | srcMacID << 21 | dstMacID << 13 | ack << 12 | funcID << 8 | sourceID
    return canID

def trigger_supersonic_detect():
    can_msg = vci_can()
    can_msg.ID = can_id_build()
    can_msg.DataLen = 2
    can_msg.Data = [0x10,0xFF]
    #rospy.loginfo(can_msg.Data)
    supersonic_pub.publish(can_msg)

def data_received(can_msg):
    can_msgg = can_msg
    a = canMsg(can_msgg)
    a.print_info()
#    rospy.loginfo("received data")
#    rospy.loginfo("ID:%08x"%data.ID)
#    rospy.loginfo("DataLen:%d"%data.DataLen)


def main():
    rate = rospy.Rate(10) #10hz
    while not rospy.is_shutdown():
        try:
            trigger_supersonic_detect()
            rate.sleep()
        except Exception:
            rospy.logerr(sys.exc_info())

        rate.sleep()
    return

if __name__ == '__main__':
    touchsensor_pub = None
    try:
        rospy.init_node('touchsensor', anonymous=True)
        rospy.Subscriber("rx_touchsensor_node", vci_can, data_received)
        touchsensor_pub = rospy.Publisher('tx_touchsensor_node', vci_can, queue_size=10)
        main()
    except Exception: #rospy.ROSInterruptException:
        rospy.logerr(sys.exc_info())
        rospy.loginfo("lost connect")
        exit(1)

