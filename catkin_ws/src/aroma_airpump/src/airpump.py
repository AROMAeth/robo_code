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



class AromaAirPump(object):
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)

        #INIT ALL PARAMETERS:
        read_string = rospy.get_param("~pins", "")
        self.pins = int(read_string)

        GPIO.setup(self.pins, GPIO.OUT)
        GPIO.output(self.pins, 0)

        #CREATE ALL THE SUBSCRIBERS:
        self.sub_topic1 = '/aroma_airpump/control'
        self.subscriber1 = rospy.Subscriber(self.sub_topic1, String, self.callback_control,queue_size=1)

        # FEEDBACK topic to show when being finished
        self.pub_topic = '/aroma_airpump/control_ended'
        self.publisher_end = rospy.Publisher(self.pub_topic, Bool, queue_size=1)


    # this is the callback function which processes the command from the outside and then executes
    # the needed or desired actions!!!
    def callback_control(self,msg):
        split_str = msg.data.split()
        if (len(split_str)==3):
            self.bubble(float(split_str[0]),float(split_str[1]),float(split_str[2]))
        else:
            print (self.pump_name + " INVALID READING STATEMENT")

        self.publisher_end.publish(True)




    def bubble(self,total_dur,period,percentage):
        #everything given in seconds
        num_cycles = int(total_dur/period)
        print ("Starting " + str(num_cycles) + " cycles with a period of " + str(period) + "seconds and " + str(percentage) + "percent on!")
        for i in range(0,num_cycles):
            GPIO.output(self.pins,1)
            time.sleep(period*percentage)
            #print "CIAO BELLO"
            GPIO.output(self.pins, 0)
            time.sleep(period*(1-percentage))
            #print "BELLO CIAO"
        print ("Airpump cycle has ended!")



    def onShutdown(self):
        rospy.loginfo('Shutting down Aroma Driving Action, back to unsafe mode')
        GPIO.cleanup()


 
if __name__ == '__main__':
    rospy.init_node('aroma_airpump_node', anonymous=False)
    aroma_airpump_node = AromaAirPump()
    rospy.on_shutdown(aroma_airpump_node.onShutdown)
    rospy.spin()
