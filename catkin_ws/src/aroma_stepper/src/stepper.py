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

        #INIT ALL PARAMETERS:
        self.want_move_backward = False
        self.want_move_forward = False
        self.speed_forward = 0.1
        self.steps_forward = 200
        self.speed_backward = 0.5
        self.steps_backward = 500

        #CREATE ALL THE SUBSCRIBERS:
        self.sub_topic1 = '/aroma_interface/stop'
        self.subscriber1 = rospy.Subscriber(self.sub_topic1, Bool, self.callback_stop,queue_size=1)



        self.sub_topic2 = '/aroma_interface/speed_for'
        self.subscriber2 = rospy.Subscriber(self.sub_topic2, Float32, self.callback_speed_for,queue_size=1)

        self.sub_topic3 = '/aroma_interface/steps_for'
        self.subscriber3 = rospy.Subscriber(self.sub_topic3, Int32, self.callback_steps_for,queue_size=1)

        self.sub_topic4 = '/aroma_interface/move_steps_for'
        self.subscriber4 = rospy.Subscriber(self.sub_topic4, Bool, self.callback_move_steps_for,queue_size=1)

        self.sub_topic5 = '/aroma_interface/move_inf_for'
        self.subscriber5 = rospy.Subscriber(self.sub_topic5, Bool, self.callback_move_inf_for,queue_size=1)



        self.sub_topic6 = '/aroma_interface/speed_back'
        self.subscriber6 = rospy.Subscriber(self.sub_topic6, Float32, self.callback_speed_back,queue_size=1)

        self.sub_topic7 = '/aroma_interface/steps_back'
        self.subscriber7 = rospy.Subscriber(self.sub_topic7, Int32, self.callback_steps_back,queue_size=1)

        self.sub_topic8 = '/aroma_interface/move_steps_back'
        self.subscriber8 = rospy.Subscriber(self.sub_topic8, Bool, self.callback_move_steps_back,queue_size=1)

        self.sub_topic9 = '/aroma_interface/move_inf_back'
        self.subscriber9 = rospy.Subscriber(self.sub_topic9, Bool, self.callback_move_inf_back,queue_size=1)



        self.control_pins = [7,11,13,15]

        for pin in self.control_pins:
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

    def callback_stop(self,msg):
        self.want_move_forward = False
        self.want_move_backward = False
        print "GOING TO STOP NOW!"


    
    def callback_speed_for(self,msg):
        print "revieved call to change forward speed to ..." + str(msg)
        self.speed_forward = msg.data

    def callback_steps_for(self,msg):
        print "revieved call to change forward number of steps to ..." + str(msg)
        self.steps_forward = msg.data

    def callback_move_steps_for(self,msg):
        print "revieved call to move forward..."
        self.move_forward(self.steps_forward,self.speed_forward) 

    def callback_move_inf_for(self,msg):
        print "revieved call to move INFINITELY forward..."
        self.move_inf_forward(self.steps_forward,self.speed_forward) 

    

    def callback_speed_back(self,msg):
        print "revieved call to change backward speed to ..." + str(msg)
        self.speed_backward = msg.data

    def callback_steps_back(self,msg):
        print "revieved call to change backward number of steps to ..." + str(msg)
        self.steps_backward = msg.data

    def callback_move_steps_back(self,msg):
        print "revieved call to move backward..."
        self.move_backward(self.steps_backward,self.speed_backward) 

    def callback_move_inf_back(self,msg):
        print "revieved call to move INFINITELY backward..."
        self.move_inf_backward(self.steps_backward,self.speed_backward) 


    # speed from 0 to 1 (one being the fastest)
    # steps 50 steps = one rotation
    def move_backward(self,steps, speed):
      if (self.want_move_forward!=True and self.want_move_backward!=True):
        self.want_move_backward = True
        for i in range(steps):
          if self.want_move_backward==False:
            break
          else:
            for halfstep in range(8):
              for pin in range(4):
                GPIO.output(self.control_pins[pin], self.halfstep_seq[halfstep][pin])
              time.sleep(max(0.001/speed,0.001))
        self.want_move_backward = False

    def move_forward(self,steps, speed):
      #to exclude multiple instances starting after each other!!!
      if (self.want_move_backward!=True and self.want_move_forward!=True):
        self.want_move_forward = True
        for i in range(steps):
            if self.want_move_forward==False:
                break
            else:
                for halfstep in range(7,-1,-1):
                    for pin in range(4):
                        GPIO.output(self.control_pins[pin], self.halfstep_seq[halfstep][pin])
                    time.sleep(max(0.001/speed,0.001))
	self.want_move_forward = False



    def move_inf_backward(self,steps, speed):
      if (self.want_move_forward!=True and self.want_move_backward!=True):
        self.want_move_backward = True
        while self.want_move_backward!=False:
          for halfstep in range(8):
            for pin in range(4):
              GPIO.output(self.control_pins[pin], self.halfstep_seq[halfstep][pin])
            time.sleep(max(0.001/speed,0.001))
        

    def move_inf_forward(self,steps, speed):
      #to exclude multiple instances starting after each other!!!
      if (self.want_move_backward!=True and self.want_move_forward!=True):
        self.want_move_forward = True
        while self.want_move_forward!=False:
          for halfstep in range(7,-1,-1):
            for pin in range(4):
              GPIO.output(self.control_pins[pin], self.halfstep_seq[halfstep][pin])
            time.sleep(max(0.001/speed,0.001))



    def onShutdown(self):
        rospy.loginfo('Shutting down Aroma Stepper Action, back to unsafe mode')
        GPIO.cleanup()


 
if __name__ == '__main__':
    rospy.init_node('aroma_stepper_node', anonymous=False)
    aroma_stepper_node = AromaStepper()
    rospy.on_shutdown(aroma_stepper_node.onShutdown)
    rospy.spin()
