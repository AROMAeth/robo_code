#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Int32

import RPi.GPIO as GPIO
import time
import numpy as np



class AromaDrive(object):
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)

        #INIT ALL PARAMETERS:
        read_string = rospy.get_param("~pins", "")
        a = read_string.split(",")
        self.pins = np.array(([int(a[0]),int(a[1])],[int(a[2]),int(a[3])],[int(a[4]),int(a[5])],[int(a[6]),int(a[7])]),dtype=int)

        self.straight_speed = rospy.get_param("~straight_speed", "")
        self.angle_speed = rospy.get_param("~angle_speed", "")

        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)

        #CREATE ALL THE SUBSCRIBERS:
        self.sub_topic1 = '/aroma_drive/control'
        self.subscriber1 = rospy.Subscriber(self.sub_topic1, String, self.callback_control,queue_size=1)


    # this is the callback function which processes the command from the outside and then executes
    # the needed or desired actions!!!
    def callback_control(self,msg):
        split_str = msg.data.split()
        if (len(split_str)==3):
            if (split_str[0]=="forward"):
                print (" Turning OF: " + str(float(split_str[1]))+ "[deg] and Forward of  " + str(float(split_str[2])) + "meter requested")
                if (float(split_str[1])>=0):
                    self.d_turnl(float(split_str[1]))
                else:
                    self.d_turnr(float(split_str[1]))
                self.d_forward(float(split_str[2]))
            elif (split_str[0]=="backward"):
                print (" Turning OF: " + str(float(split_str[1]))+ "[deg] and Backward of  " + str(float(split_str[2])) + "meter requested")
                if (float(split_str[1])>=0):
                    self.d_turnl(float(split_str[1]))
                else:
                    self.d_turnr(float(split_str[1]))
                self.d_backward(float(split_str[2]))
            else:
                print (self.pump_name + " INVALID READING STATEMENT") 
        else:
            print (self.pump_name + " INVALID READING STATEMENT")




    def d_forward(self,dist):
        t = dist/self.straight_speed
        GPIO.output(self.pins[0][0],1)
        GPIO.output(self.pins[1][0],1)
        GPIO.output(self.pins[2][0],1)
        GPIO.output(self.pins[3][0],1)
        time.sleep(t)
        for pin in self.pins:
            GPIO.output(pin, 0)

    def d_backward(self,dist):
        t = dist/self.straight_speed
        GPIO.output(self.pins[0][1],1)
        GPIO.output(self.pins[1][1],1)
        GPIO.output(self.pins[2][1],1)
        GPIO.output(self.pins[3][1],1)
        time.sleep(t)
        for pin in self.pins:
            GPIO.output(pin, 0)


    def d_turnr(self,angle):
        t = dist/self.angle_speed
        GPIO.output(self.pins[0][0],1)
        GPIO.output(self.pins[1][1],1)
        GPIO.output(self.pins[2][0],1)
        GPIO.output(self.pins[3][1],1)
        time.sleep(t)
        for pin in self.pins:
            GPIO.output(pin, 0)

    def d_turnl(self,angle):
        t = dist/self.angle_speed
        GPIO.output(self.pins[0][1],1)
        GPIO.output(self.pins[1][0],1)
        GPIO.output(self.pins[2][1],1)
        GPIO.output(self.pins[3][0],1)
        time.sleep(t)
        for pin in self.pins:
            GPIO.output(pin, 0)



    def onShutdown(self):
        rospy.loginfo('Shutting down Aroma Driving Action, back to unsafe mode')
        GPIO.cleanup()


 
if __name__ == '__main__':
    rospy.init_node('aroma_driving_node', anonymous=False)
    aroma_driving_node = AromaDrive()
    rospy.on_shutdown(aroma_driving_node.onShutdown)
    rospy.spin()
