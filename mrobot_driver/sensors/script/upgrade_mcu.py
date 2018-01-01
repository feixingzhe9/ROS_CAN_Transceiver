#!/usr/bin/env python
# encoding: utf-8

import hashlib
import os,sys
from time import sleep,time,ctime
import rospy
import struct
from canMsg import canMsg
from std_msgs.msg import String
from mrobot_driver_msgs.msg import vci_can

filename = 'test.bin'

def HexShow(argv):
        result = ''
        hLen = len(argv)
        for i in xrange(hLen):
            hvol = ord(argv[i])
            hhex = '%02x'%hvol
            result += hhex + ' '
        print 'Hex:',result
        return result[0:-1]

def md5hex(word):
    """ MD5加密算法，返回32位小写16进制符号
    """
    if isinstance(word, unicode):
        word = word.encode("utf-8")
    elif not isinstance(word, str):
        word = str(word)
    m = hashlib.md5()
    m.update(word)
    return m.hexdigest()

def md5sum(fname):
    """ 计算文件的MD5值
    """
    def read_chunks(fh):
        fh.seek(0)
        chunk = fh.read(8096)
        while chunk:
            yield chunk
            chunk = fh.read(8096)
        else: #最后要将游标放回文件开头
            fh.seek(0)
    m = hashlib.md5()
    if isinstance(fname, basestring) \
            and os.path.exists(fname):
        with open(fname, "rb") as fh:
            for chunk in read_chunks(fh):
                m.update(chunk)
    #上传的文件缓存 或 已打开的文件流
    elif fname.__class__.__name__ in ["StringIO", "StringO"] \
            or isinstance(fname, file):
        for chunk in read_chunks(fname):
            m.update(chunk)
    else:
        return ""
    return m.hexdigest()

def progress_bar(num, total):
	rate_num = int(num * 100 / total)
	r = '\r[%s%s]%d%%'%("="*rate_num, " "*(100-rate_num), rate_num)
	sys.stdout.write(r)
	if not rate_num == 100:
		sys.stdout.flush()
	else:
		sys.stdout.write('\n')

def can_id_build(update_step=0):
    reserve = 0x0
    srcMacID = 0x01
    dstMacID = 0x6c
    ack = 0x00
    funcID = 0x01
    sourceID = 0x10 + update_step
    canID = reserve << 28 | srcMacID << 21 | dstMacID << 13 | ack << 12 | funcID << 8 | sourceID
    return canID

def longFramePublish(msg):
	can_msg = vci_can()
	can_msg.ID = msg.ID
	seg_polo0 = int('0x40', 16)
	seg_polo1 = int('0x80', 16)
	seg_polo2 = int('0xc0', 16)
	data_index = 0
	loop_index = 0
	loop_total = msg.DataLen / 7
	if loop_total != 0:
		if loop_index == 0:
			can_msg.DataLen = 8
			print 'loop index:%d'%loop_index
			can_msg.Data = ('%02x'%seg_polo0).decode('hex') + msg.Data[data_index:data_index + 7]
			 #a = canMsg(can_msg)
			 #a.print_info()
			upgrade_pub.publish(can_msg)
			loop_index += 1
			data_index += 7
		if loop_index > 0:
			while loop_index < loop_total:
				print 'loop index:%d'%loop_index
				can_msg.DataLen = 8
				can_msg.Data = ('%02x'%(seg_polo1 + loop_index)).decode('hex') + msg.Data[data_index:data_index + 7]
				 #a = canMsg(can_msg)
				 #a.print_info()
				upgrade_pub.publish(can_msg)
				loop_index += 1
				data_index += 7
	if msg.DataLen % 7:
		can_msg.DataLen = msg.DataLen % 7 + 1
		can_msg.Data = ('%02x'%(seg_polo2 + loop_index)).decode('hex') + msg.Data[data_index:]
		 #a = canMsg(can_msg)
		 #a.print_info()
		upgrade_pub.publish(can_msg)
	
def upgradePrepare():
	firmwareSize = '%08x'%os.path.getsize(filename)
	firmwareMd5 = md5sum(filename)
	print 'firmwareMd5:'+ firmwareMd5
	print 'firmSize:' + firmwareSize
	can_msg = vci_can()
	can_msg.ID = can_id_build()
	can_msg.DataLen = 20
	can_msg.Data = firmwareMd5.decode('hex') + firmwareSize.decode('hex')
	longFramePublish(can_msg)
	HexShow(can_msg.Data)
	print 'can_data.len:%d' %(len(can_msg.Data))

def upgradeFirmwareSending():
	global is_upgrade_ack
	can_msg = vci_can()
	can_msg.ID = can_id_build(update_step=1)
	seg_polo0 = int('0x40', 16)
	seg_polo1 = int('0x80', 16)
	seg_polo2 = int('0xc0', 16)
	def sendPackage(msg):
		upgrade_pub.publish(msg)
		global is_upgrade_ack
		is_upgrade_ack = False
		 #a = canMsg(msg)
		 #a.print_info()
		return True

	firmware = open(filename,'rb')
	count = 0
	index = 0
	group = 0
	filesize = os.path.getsize(filename)
	if not filesize > 6:
		return False

	print 'start time: ' + ctime()
	startTime = time()
	while not rospy.is_shutdown():
		try:
			if is_upgrade_ack == True:
				strRead = firmware.read(6)
				if not strRead:
					endTime = time()
					used_time = endTime - startTime
					print 'average rate: %db/s'%(int(filesize/used_time))
					print 'endTime   = %f'%endTime
					print 'startTime = %f'%startTime
					print 'send file over, used time: %fs'%used_time
					print 'now time:' + ctime()
					break
				writeLen = len(strRead)
				can_msg.DataLen = 2 + writeLen
				if count == 0:
					can_msg.Data = ('%02x'%seg_polo0).decode('hex') + ('%02x'%group).decode('hex') + strRead
					sendPackage(can_msg)
				if count > 0 and writeLen == 6:
					can_msg.Data = ('%02x'%(seg_polo1 + index)).decode('hex') + ('%02x'%group).decode('hex') + strRead
					sendPackage(can_msg)
				if writeLen < 6 or (filesize - count * 6) < 6:
					can_msg.Data = ('%02x'%(seg_polo2 + index)).decode('hex') + ('%02x'%group).decode('hex') + strRead
					sendPackage(can_msg)
				count += 1
				index = count % 64
				group = count / 64
				progress_bar(count*6, filesize)
			else:
				sleep(0.015)
		except Exception:
			firmware.close()
			print 'error, close file'
			return False
	firmware.close()
	return True

def upgradeFinishCheck():
	can_msg = vci_can()
	can_msg.ID = can_id_build(update_step=2)
	can_msg.DataLen = 1
	can_msg.Data = '\x00'
	upgrade_pub.publish(can_msg)
	 #a = canMsg(can_msg)
	 #a.print_info()

def data_received(can_msg):
	 #seg_polo0 = int('0x40', 16)
	 #seg_polo1 = int('0x80', 16)
	 #seg_polo2 = int('0xc0', 16)
	global is_prepare_ok
	global is_upgrade_ack
	global receive_count
	a = canMsg(can_msg)
	 #a.print_info()
	if a.get_sourceID() == 0x10:
		if a.get_data()[1] == 0:
			is_prepare_ok = True
			print 'mcu prepare ok'
	elif a.get_sourceID() == 0x11:
		is_upgrade_ack = True
		receive_count += 1
		 #print 'rx_count:%d,frame id:%d'%(receive_count, (a.get_data()[2]*64 + a.get_data()[1] - seg_polo1) + 1)
	elif a.get_sourceID() == 0x12:
		if a.get_data()[1] == 0:
			print 'md5 check correct'
		else:
			print 'md5 check incorrect'

def main():
	global is_prepare_ok
	global is_upgrade_ack
	global receive_count
	rate = rospy.Rate(10)
	is_prepare_over = False
	is_upgrade_ack = True
	receive_count = 0
	while not rospy.is_shutdown():
		if is_prepare_over == False:
			upgradePrepare()
			is_prepare_over = True 
		try:
			if is_prepare_ok == False:
				print 'mcu not ok'
			else:
				if not upgradeFirmwareSending():
					break
				else:
					break
		except Exception:
			rospy.logerr(sys.exc_info())
		rate.sleep()
	return

if __name__ == "__main__":
	upgrade_pub = None
	global is_prepare_ok
	global is_upgarde_ack
	global upgrade_frame_id
	is_prepare_ok = False
	sub_topic = 'rx_test_node'
	pub_topic = 'tx_test_node'
	try:
		rospy.init_node('upgrade_node', anonymous=True)
		rospy.Subscriber(sub_topic, vci_can, data_received)
		upgrade_pub = rospy.Publisher(pub_topic, vci_can, queue_size=1000)
		sleep(1)
		main()
		upgradeFinishCheck()
	except Exception:
		rospy.logerr(sys.exc_info())
		rospy.loginfo("lost connect")
		exit(1)
	sleep(2)
	print 'program over'
