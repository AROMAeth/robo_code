#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String

import RPi.GPIO as GPIO
import time




class AromaStepper(object):
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        control_pins = [7,11,13,15]

        for pin in control_pins:
          GPIO.setup(pin, GPIO.OUT)
          GPIO.output(pin, 0)

        halfstep_seq = [
          [1,0,0,0],
          [1,1,0,0],
          [0,1,0,0],
          [0,1,1,0],
          [0,0,1,0],
          [0,0,1,1],
          [0,0,0,1],
          [1,0,0,1]
        ]
        self.move_backward(500,0.5)
        self.move_forward(50,0.1)

    # speed from 0 to 1 (one being the fastest)
    # steps 50 steps = one rotation
    def move_backward(self,steps, speed):
      for i in range(steps):
        for halfstep in range(8):
          for pin in range(4):
            GPIO.output(control_pins[pin], halfstep_seq[halfstep][pin])
          time.sleep(max(0.001/speed,0.001))

    def move_forward(self,steps, speed):
      for i in range(steps):
        for halfstep in range(7,-1,-1):
          for pin in range(4):
            GPIO.output(control_pins[pin], halfstep_seq[halfstep][pin])
          time.sleep(max(0.001/speed,0.001))



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
        rospy.loginfo('Shutting down Aroma Stepper Action, back to unsafe mode')
        GPIO.cleanup()


 
if __name__ == '__main__':
    rospy.init_node('aroma_stepper_node', anonymous=False)
    aroma_stepper_node = AromaStepper()
    rospy.on_shutdown(aroma_stepper_node.onShutdown)
    rospy.spin()