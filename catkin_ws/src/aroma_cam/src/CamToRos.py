#!/usr/bin/env python

from picamera import PiCamera
from time import sleep

import numpy as np
import sys, os
import argparse

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#parser = argparse.ArgumentParser()
#parser.add_argument("name_of_folder", help="specify the name of the experiment/path where to be saved!!!", type=str)

#args=parser.parse_args()
#bag = rosbag.Bag(args.input_bagpath)
#print "HELLO"

#dir = args.name_of_folder
#if os.path.exists(dir):
#	print "Directory already exists"
#	sys.exit(3) 
#
#os.makedirs(dir)
#print (dir + '%s.jpg' % i)

#print ("Photoshoot Starting ;)")
#
#camera = PiCamera()
#camera.resolution = (2592, 1952)
#camera.framerate = 15

#output = np.empty((2592*1952*3,), dtype=np.uint8)

#sleep(4)
#for i in range(10):
#	camera.start_preview()
#	sleep(1)s
#	camera.capture(dir + '/image%s.jpg' % i)
#	#camera.capture(output,'rgb')
#	print ('Pic Taken')
#	camera.stop_preview()

#print ("Photoshoot Finished")
#output = output.reshape((2592,1952,3))

def talker():
     pub = rospy.Publisher('chatter', String, queue_size=10)
     rospy.init_node('talker', anonymous=True)
     rate = rospy.Rate(10) # 10hz
     while not rospy.is_shutdown():
         hello_str = "hello world %s" % rospy.get_time()
         rospy.loginfo(hello_str)
         pub.publish(hello_str)
         rate.sleep()
 
if __name__ == '__main__':
    rospy.init_node('aroma_cam_node', anonymous=False)
    talker()
    #obst_detection_node = ObstDetectNode()
    #rospy.on_shutdown(obst_detection_node.onShutdown)
    #rospy.spin()