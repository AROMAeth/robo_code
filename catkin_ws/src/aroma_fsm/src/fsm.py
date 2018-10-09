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



class AromaFSM(object):
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)

        #INIT ALL DYNAMIC PARAMETERS:
        medium_tank_name = rospy.get_param("~pump1", "")
        spill_tank_name = rospy.get_param("~pump2", "")
        microfluidic_name = rospy.get_param("~pump3", "")

        #NOW DEFINE THE PARAMETERS:
        #medium tank parameters: 1)direction 2)volume in ul 3)speed in ul per min
        self.medium_tank_dir = "push"
        self.medium_tank_vol = 100.0
        self.medium_tank_speed = 1000.0
        self.medium_tank_ended = False

        #spill tank parameters: 1)direction 2)volume in ul 3)speed in ul per min
        self.spill_tank_dir = "pull"
        self.spill_tank_vol = 100.0
        self.spill_tank_speed = 1000.0
        self.spill_tank_ended = False

        #microfluidic tank parameters: 1)direction 2)volume in ul 3)speed in ul per min
        self.microfluidic_tank_dir = "pull"
        self.microfluidic_tank_vol = 100.0
        self.microfluidic_tank_speed = 1000.0
        self.microfluidic_tank_ended = False

        #DEFINE THE BUBBLING PARAMETERS
        self.bubbling_time = 10.0
        self.bubbling_period = 5.0
        self.bubbling_percentage = 0.5
        self.bubbling_ended = False


        #PUMPING:
        self.sub_topic1 = '/{}/syringe_control_ended'.format(medium_tank_name)
        self.subscriber1 = rospy.Subscriber(self.sub_topic1, Bool, self.callback_medium_tank,queue_size=1)

        self.sub_topic2 = '/{}/syringe_control_ended'.format(spill_tank_name)
        self.subscriber2 = rospy.Subscriber(self.sub_topic2, Bool, self.callback_spill_tank,queue_size=1)

        self.sub_topic3 = '/{}/syringe_control_ended'.format(microfluidic_tank_name)
        self.subscriber3 = rospy.Subscriber(self.sub_topic3, Bool, self.callback_microfluidic_tank,queue_size=1)

        self.pub_topic1 = '/{}/syringe_control'.format(medium_tank_name)
        self.publisher_medium = rospy.Publisher(self.pub_topic1, String, queue_size=1)

        self.pub_topic2 = '/{}/syringe_control'.format(spill_tank_name)
        self.publisher_spill = rospy.Publisher(self.pub_topic2, String, queue_size=1)

        self.pub_topic3 = '/{}/syringe_control'.format(microfluidic_tank_name)
        self.publisher_microfluidic = rospy.Publisher(self.pub_topic3, String, queue_size=1)

        #BUBBLING:
        self.sub_topic4 = '/aroma_airpump/control_ended'
        self.subscriber4 = rospy.Subscriber(self.sub_topic4, Bool, self.callback_bubbling,queue_size=1)

        self.pub_topic4 = '/aroma_airpump/control'
        self.publisher_airpump = rospy.Publisher(self.pub_topic4, String, queue_size=1)

        #DRIVING:
        self.sub_topic5 = '/aroma_drive/control_ended'
        self.subscriber5 = rospy.Subscriber(self.sub_topic5, Bool, self.callback_drive,queue_size=1)

        self.pub_topic5 = '/aroma_airpump/control'
        self.publisher_drive = rospy.Publisher(self.pub_topic5, String, queue_size=1)


        #call the bubbling cycle for once!!! <-> init measurement!!!


        #NOW create the publishers and subscribers!!!



    #HERE NOW THE SIGNAL FLOW COMES IN: STEP 1 -> WE DRIVE!!!
    def callback_drive(self,msg):
        if(msg.data):
            create_str = (self.medium_tank_dir + " " + str(self.medium_tank_vol) + " " + str(self.medium_tank_vol))
            #now call the pump to move
            self.publisher_medium.publish(create_str)

    #AFTER MEDIUM WAS TAKEN IN -> NOW BUBBLE
    def callback_medium_tank(self,msg):
        if(msg.data):
            create_str = (str(self.bubbling_time) + " " + str(self.bubbling_period) + " " + str(self.bubbling_percentage))
            #now call the pump to move
            self.publisher_airpump.publish(create_str)

    #BUBBLING IS OVER -> SUCK THE SHIT INTO THE CHIP
    def callback_bubbling(self,msg):
        if(msg.data):
            create_str = (str(self.microfluidic_tank_dir) + " " + str(self.microfluidic_tank_vol) + " " + str(self.microfluidic_tank_speed))
            #now call the pump to move
            self.publisher_microfluidic.publish(create_str)

    #BUBBLING MEDIUM IN CHIP -> START IMAGING AND EMPTY THE CHAMBER!!!
    def callback_microfluidic_tank(self,msg):
        if(msg.data):
            print "ALSO NOW THEORETICALLY THE IMAGING CAN START"
            create_str = (str(self.spill_tank_dir) + " " + str(self.spill_tank_vol) + " " + str(self.spill_tank_speed))
            #now call the pump to move
            self.publisher_spill.publish(create_str)

    #IF THE SPILL PUMP IS FINISHED ONE COULD THEORETICALLY PUT IN NEW MEDIUM OR WE JUST LEAVE THE PIPELINE
    def callback_spill_tank(self,msg):
        if(msg.data):
            print "CHAMBER IS EMPTY NOW"


    def onShutdown(self):
        rospy.loginfo('Shutting down Aroma FSM Action, back to unsafe mode')


 
if __name__ == '__main__':
    rospy.init_node('aroma_FSM_node', anonymous=False)
    aroma_fsm_node = AromaFSM()
    rospy.on_shutdown(aroma_fsm_node.onShutdown)
    rospy.spin() #DRIVE BEFEHL STARTET ALLES!!!
