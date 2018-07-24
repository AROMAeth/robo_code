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

class AromaCam(object):
    def __init__(self):
        #pub = rospy.Publisher('chatter', String, queue_size=10)
        self.image_pub = rospy.Publisher("aroma_cam_img",Image)
        self.bridge = CvBridge()

        self.camera = PiCamera()
        self.camera.resolution = (2592, 1952)
        self.camera.framerate = 15

        self.output = np.empty((2592*1952*3,), dtype=np.uint8)
        self.pic = np.empty((1952,2592,3), dtype=np.uint8)
        
        sleep(4)
        print "EVERYTHING SETUP NOW"

        self.defineSettings()
        self.takePictures()


    def takePictures(self):
        rate = rospy.Rate(10) # 10hz
        self.camera.zoom = (0.25,0.25,0.75,0.75)
        while not rospy.is_shutdown():

            self.camera.capture(self.output,'rgb')
            #print ('Pic Taken')
            #camera.stop_preview()
            self.pic = self.output.reshape((1952,2592,3))            

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.pic, "bgr8"))
            except CvBridgeError as e:
                print(e)

            rate.sleep()


    def defineSettings(self):
        # Now fix the values
        self.camera.shutter_speed = self.camera.exposure_speed
        self.camera.exposure_mode = 'off'
        g = self.camera.awb_gains
        self.camera.awb_mode = 'off'
        self.camera.awb_gains = g
        # Finally, take several photos with the fixed settings



    def onShutdown(self):
        rospy.loginfo('Shutting down Aroma Cam Action, back to unsafe mode')


 
if __name__ == '__main__':
    rospy.init_node('aroma_cam_node', anonymous=False)
    aroma_cam_node = AromaCam()
    rospy.on_shutdown(aroma_cam_node.onShutdown)
    rospy.spin()
