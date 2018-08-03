#!/usr/bin/env python
 
import sys
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk

import rospy
import os
import rospkg
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Int32

class HellowWorldGTK:
	def __init__(self):
		#Set the path to Glade file, in the ros_glade ROS package
		#TODO: variabler Pfad!!!
		rospack = rospkg.RosPack()
		print rospack.get_path('aroma_interface')

		str= rospack.get_path('aroma_interface') + "/src/aroma_int.glade"
		self.gladefile = str 
		#Initiate the Builder and point it to the glade file
		self.builder = Gtk.Builder()
		self.builder.add_from_file(self.gladefile)
		#Connect event functions
		self.builder.connect_signals(self)
		window = self.builder.get_object("window1")
		window.show_all()
		
		rospy.init_node('aroma_interface', anonymous=False)
		self.pub1 = rospy.Publisher('aroma_interface/stop', Bool, queue_size=1)
		
		self.pub2 = rospy.Publisher('aroma_interface/speed_for', Float32, queue_size=1)
		self.pub3 = rospy.Publisher('aroma_interface/steps_for', Int32, queue_size=1)
		self.pub4 = rospy.Publisher('aroma_interface/move_steps_for', Bool, queue_size=1)
		self.pub5 = rospy.Publisher('aroma_interface/move_inf_for', Bool, queue_size=1)
		
		self.pub6 = rospy.Publisher('aroma_interface/speed_back', Float32, queue_size=1)
		self.pub7 = rospy.Publisher('aroma_interface/steps_back', Int32, queue_size=1)
		self.pub8 = rospy.Publisher('aroma_interface/move_steps_back', Bool, queue_size=1)
		self.pub9 = rospy.Publisher('aroma_interface/move_inf_back', Bool, queue_size=1)

		#rospy.on_shutdown(self.onShutdown)


	def stop_clicked(self, widget):
		#Simple button cliked event
		#self.talker() #Calls talker function which sends a ROS message
		print ("STOOOOOOOOOOOOOOP!!!!!!!!!!")
		self.pub1.publish(Bool(True))

	def scale_speed_for_val(self, widget):
		print "Speed forward" + str(widget.get_value ())
		self.pub2.publish(Float32(widget.get_value ()))

	def scale_steps_for_val(self, widget):
		print "Steps forward" + str(widget.get_value ())
		self.pub3.publish(Int32(widget.get_value ()))

	def move_steps_for_clicked_cb(self, widget):
		print ("move ... steps forward")
		self.pub4.publish(Bool(True))

	def move_inf_steps_for_clicked_cb(self, widget):
		#Simple button cliked event
		#self.talker() #Calls talker function which sends a ROS message
		print ("move inf steps forward")
		self.pub5.publish(Bool(True))

	def scale_speed_back_val(self, widget):
		print "Speed backward" + str(widget.get_value ())
		self.pub6.publish(Float32(widget.get_value ()))

	def scale_steps_back_val(self, widget):
		print "Steps backward" + str(widget.get_value ())
		self.pub7.publish(Int32(widget.get_value ()))

	def move_steps_back_clicked_cb(self, widget):
		#Simple button cliked event
		#self.talker() #Calls talker function which sends a ROS message
		print ("move ... steps backward")
		self.pub8.publish(Bool(True))

	def move_inf_steps_back_clicked_cb(self, widget):
		#Simple button cliked event
		#self.talker() #Calls talker function which sends a ROS message
		print ("move inf steps backward")
		self.pub9.publish(Bool(True))


	def MainWindow_destroy(self,widget):
		#MainWindow_destroy event
		sys.exit(0)

	#def onShutdown(self):
	#	rospy.loginfo('Shutting down Interface!')
    #    sys.exit(0)

if __name__ == "__main__":
	#s#tart the class
	hwg = HellowWorldGTK()
	#Gtk.timeout_add(1000, hwg.Timer1_timeout) #Adds a timer to the GUI, with hwg.Timer1_timeout as a 
	#callback function for the timer1
	Gtk.main()#Starts GTK