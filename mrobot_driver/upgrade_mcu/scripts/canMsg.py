#!/usr/bin/env python
#coding=utf8

import struct
from mrobot_driver_msgs.msg import vci_can

class CanMsg(object):
    def __init__(self, can_msg):
        self.can_id = can_msg.ID
        self.src_mac_id = (self.can_id & (0x000000ff << 21)) >> 21
        self.dst_mac_id = (self.can_id & (0x000000ff << 13)) >> 13
        self.ack = (self.can_id & (0x00000001 << 12)) >> 12
        self.func_id = (self.can_id & (0x0000000f << 8)) >> 8
        self.source_id = (self.can_id & (0x000000ff << 0)) >> 0

        self.data_length = can_msg.DataLen
        self.data = [ord(c) for c in can_msg.Data]

    def get_can_id(self):
        return self.can_id

    def get_src_mac_id(self):
        return self.src_mac_id

    def get_dst_mac_id(self):
        return self.dst_mac_id

    def get_ack(self):
        return self.ack

    def get_func_id(self):
        return self.func_id

    def get_source_id(self):
        return self.source_id

    def get_data_length(self):
        return self.data_length

    def get_data(self):
        return self.data

    def print_info(self):
        print 'can_id = 0x%08x' % self.get_can_id()
        print 'srcMacID = 0x%02x, dstMacID = 0x%02x, ack = 0x%01x, funcID = 0x%01x, sourceID = 0x%02x' % (self.get_src_mac_id(),self.get_dst_mac_id(),self.get_ack(),self.get_func_id(),self.get_source_id())
#        print 'dstMacID = 0x%02x' % self.get_dstMacID()
#        print 'ack = 0x%01x' % self.get_ack()
#        print 'funcID = 0x%01x' % self.get_funcID()
#        print 'sourceID = 0x%02x' % self.get_sourceID()
        print 'data_length = %d' % self.get_data_length()
#        print 'data:',struct.unpack("<I", self.get_data())
        print 'data:',[hex(c) for c in self.get_data()]

if __name__ == '__main__':

    can_msg = vci_can()
    can_msg.ID = 0xa201281
    can_msg.DataLen = 3
    can_msg.Data = b'\x01\x10\xff\x91'# (0x01,0x10,0xff,0x01)#

    a = CanMsg(can_msg)
    a.print_info()

