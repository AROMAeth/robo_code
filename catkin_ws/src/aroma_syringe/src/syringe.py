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


class AromaStepper(object):
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)

        self.pump_name = rospy.get_param("~name", "")
        read_string = rospy.get_param("~pins", "")
        a = read_string.split(",")
        self.pins = np.array(([int(a[0]),int(a[1]),int(a[2]),int(a[3])]),dtype=int)
        #print self.pins
        step_size = rospy.get_param("~step_size", "")
        self.ul_per_step = (1/step_size)*1000
        print self.ul_per_step

        #CREATE ALL THE SUBSCRIBERS:
        self.sub_topic1 = '/{}/syringe_control'.format(self.pump_name)
        self.subscriber1 = rospy.Subscriber(self.sub_topic1, String, self.callback_control,queue_size=1)



        for pin in self.pins:
          GPIO.setup(pin, GPIO.OUT)
          GPIO.output(pin, 0)

        self.halfstep_seq = [
          [1,0,0,0],
          [1,1,0,0],
          [0,1,0,0],
          [0,1,1,0],
          [0,0,1,0],
          [0,0,1,1],
          [0,0,0,1],
          [1,0,0,1]
        ]
        #self.move_backward(500,0.5)
	# time.sleep(5)
        #self.move_forward(500,0.1)

    def callback_control(self,msg):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        split_str = msg.data.split()
        if (len(split_str)==3):
            if (split_str[0]=="push"):
                self.move_push(float(split_str[1]),float(split_str[2]))
            elif (split_str[0]=="pull"):
                self.move_pull(float(split_str[1]),float(split_str[2]))
            else:
                print ("INVALID READING STATEMENT") 
        else:
            print ("INVALID READING STATEMENT")




    # volume in ul
    # speed in ul per min
    def move_push(self,vol, speed):

        speed_stepspermin = speed/self.ul_per_step
        T_steps = 60/speed_stepspermin
        t_delay = T_steps/8

        print ("Delay between Steps =", T_steps)
        steps = int(vol / self.ul_per_step)

        print ("Number of Steps     =", steps)
        for i in range(steps):
            for halfstep in range(8):
                for pin in range(4):
                    GPIO.output(self.pins[pin], self.halfstep_seq[halfstep][pin])
                time.sleep(max(t_delay,0.001))

    # volume in ul
    # speed in ul per min
    def move_pull(self,vol, speed):

        speed_stepspermin = speed/self.ul_per_step
        T_steps = 60/speed_stepspermin
        t_delay = T_steps/8

        steps = int(vol / self.ul_per_step)

        for i in range(steps):
            for halfstep in range(7,-1,-1):
                for pin in range(4):
                    GPIO.output(self.pins[pin], self.halfstep_seq[halfstep][pin])
                time.sleep(max(t_delay,0.001))




    def onShutdown(self):
        rospy.loginfo('Shutting down Aroma Stepper Action, back to unsafe mode')
        GPIO.cleanup()


 
if __name__ == '__main__':
    pump_name = rospy.get_param("~name", "")
    rospy.init_node(pump_name+"_pump", anonymous=False)
    aroma_stepper_node = AromaStepper()
    rospy.on_shutdown(aroma_stepper_node.onShutdown)
    rospy.spin()
