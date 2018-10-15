#!/usr/bin/env python

import sys
import os
import time
import cv2
import threading
import numpy as np
import signal
import json
import Image

import ArducamSDK

from matplotlib import pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

from PIL import Image
import matplotlib

import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from aroma_motility.ecoli import Ecoli_obj


class MotilityPipeline(object):
	"""
	Obstacle Detection Node
	"""
	def init_keypoint_Detection(self):
		# Setup SimpleBlobDetector parameters.
		params = cv2.SimpleBlobDetector_Params()
		# Change thresholds
		params.minThreshold = 10
		params.maxThreshold = 200


		# Filter by Area.
		params.filterByArea = True
		params.minArea = 200
		params.maxArea=600

		# Filter by Circularity
		params.filterByCircularity = False
		params.maxCircularity = 0.8
		params.minCircularity = 0

		# Filter by Convexity
		params.filterByConvexity = False
		newvariable305 = 1
		params.minConvexity = 0.1

		# Filter by Inertia
		params.filterByInertia = False
		params.minInertiaRatio = 0.01

		# Create a detector with the parameters
		ver = (cv2.__version__).split('.')
		if int(ver[0]) < 3 :
			detector = cv2.SimpleBlobDetector(params)
		else :
			detector = cv2.SimpleBlobDetector_create(params)
		return detector


	def __init__(self):
		self.node_name = "Motility Detection node"
		self.thread_lock = threading.Lock()

		self.state = 0 #0 means first frame must be evaluated!; 1 means initial; 2 means continouus
		self.good_points = []
		self.roi_size_refine = 60
		self.roi_size_corners = 35
		self.width = 0 #will be set when analysing the first image
		self.height = 0 #will be set when analysing the first image
		self.num_frame=0
		self.init_length = 20 #lenght over which initialization is done
		self.interm_length = 50

		self.save_plots = False
		self.storepath = "/home/niggi/Desktop/data/"

		#init detector
		self.detector = self.init_keypoint_Detection()


		self.bridge_ = CvBridge()

		#Create subscribers
		self.sub_topic = "/arducam_node/image"
		self.subscriber = rospy.Subscriber(self.sub_topic, Image, self.callback_img,queue_size=1, buff_size=2**24)

		#Create Publisher
		self.pub_topic_corr = "/motility_node/corrected/image"
		self.publisher_corr = rospy.Publisher(self.pub_topic_corr, Image, queue_size=1)

		self.pub_topic_draw = "/motility_node/draw/image"
		self.publisher_draw = rospy.Publisher(self.pub_topic_draw, Image, queue_size=1)



	def callback_img(self, image):
		thread = threading.Thread(target=self.callback,args=(image,))
		thread.setDaemon(True)
		thread.start()



	def callback(self, image):
		if not self.thread_lock.acquire(False):
			return

		img = self.bridge_.imgmsg_to_cv2(image,'mono8')
		# NOW PREPROCESS:

		img = cv2.equalizeHist(img)
		img = cv2.GaussianBlur(img,(5,5),cv2.BORDER_DEFAULT)

		#Case distinction now:
		if(self.state==0):
			self.init_keypoints(img)
		elif(self.state==1):
			self.refine_keypoints(img)
			self.num_frame += 1
		elif (self.state == 2):
			#normal processing
			self.cont_operation(img)
			self.num_frame += 1
			if(self.num_frame==50 or self.num_frame==100):
				self.filter_points(img)


		if (self.num_frame == 100000):
			self.logging(img)





		self.publisher_corr.publish(self.bridge_.cv2_to_imgmsg(img, 'mono8'))
		#draw the circles into the image:
		for point in self.good_points:
			cv2.circle(img, (np.int(point.x), np.int(point.y)), radius=10, color=(255))  # draw circle for each
		self.publisher_draw.publish(self.bridge_.cv2_to_imgmsg(img, 'mono8'))



		self.thread_lock.release()

	def onShutdown(self):
		rospy.loginfo('Shutting down Obstacle Detection, back to unsafe mode')



	def logging(self,im):
		blob_nr = 0


		for ecoli in self.good_points:
			im = cv2.circle(im, (np.int(ecoli.x), np.int(ecoli.y)),
							radius=10, color=(255))  # Show blobs
			# cv2.imshow("Keypoints", im)
			cv2.imwrite(self.storepath + 'blob' + str(blob_nr) + '.png', im)

			pp = PdfPages(self.storepath + 'blob' + str(blob_nr) + '.pdf')
			f1 = plt.figure()
			plt.plot(ecoli.history[:])
			plt.suptitle('Mean of rotational angle in degrees' + str(blob_nr), fontsize=15)
			plt.xlabel('Consecutive Measurements', fontsize=12)
			plt.ylabel('Mean of rotational angle', fontsize=12)
			pp.savefig(f1, bbox_inches='tight')

			f4 = plt.figure()
			plt.plot(ecoli.angles[:])
			plt.suptitle(' Rotational Angle of Blob' + str(blob_nr), fontsize=15)
			plt.xlabel('Time in frames', fontsize=12)
			plt.ylabel('Rotational Angle', fontsize=12)
			pp.savefig(f4, bbox_inches='tight')

			#f2 = plt.figure()
			#plt.plot(filtered_angle_diff[:])
			#plt.suptitle('Filtered rotation difference of Blob ' + str(blob_nr), fontsize=15)
			#plt.xlabel('Time in frames', fontsize=12)
			#plt.ylabel('Relative Rotational Angle', fontsize=12)
			#pp.savefig(f2, bbox_inches='tight')

			f3 = plt.figure()
			plt.plot(ecoli.angle_diff[:])
			plt.suptitle('Raw rotation difference of Blob ' + str(blob_nr), fontsize=15)
			plt.xlabel('Time in frames', fontsize=12)
			plt.ylabel('Relative Rotational Angle', fontsize=12)
			pp.savefig(f3, bbox_inches='tight')
			pp.close()
			blob_nr = blob_nr + 1

	def calc_filter(self,value, size):
		filter = []
		for i in range(len(value)):
			filter.append(np.median(value[max(0,i-size):min(i+size,len(value)-1)]))
		return filter


	# getting initial keypoits, checking if they stay in the area and returning their mean position as a list
	def refine_keypoints(self,im):

		del_points = []
		for ecoli in self.good_points:
			roi = im[max(0, int(round(ecoli.y - self.roi_size_refine / 2))):min(int(round(ecoli.y + self.roi_size_refine / 2)), self.width),
			  max(0, int(round(ecoli.x - self.roi_size_refine / 2))):min(int(round(ecoli.x + self.roi_size_refine / 2)), self.height)]
			all_blobs = self.detector.detect(roi)

			if len(all_blobs) == 1:
				ecoli.error = 0
				single_blob = all_blobs[0]
				ecoli.pointsx.append(single_blob.pt[0])
				ecoli.pointsy.append(single_blob.pt[1])
			elif len(all_blobs) > 0:
				if (np.size(ecoli.pointsx) > 0):
					dist = []
					a = np.size(ecoli.pointsx)
					for blub in all_blobs:
						dist.append(np.sqrt((ecoli.pointsx[a - 1] - blub.pt[0]) ** 2 + (ecoli.pointsy[a - 1] - blub.pt[1]) ** 2))
					single_blob = all_blobs[np.argmin(dist)]
					ecoli.pointsx.append(single_blob.pt[0])
					ecoli.pointsy.append(single_blob.pt[1])
			else:
				ecoli.error = ecoli.error + 1

			#SORT THE STUFF
			if ecoli.error > 5:
				del_points.append(ecoli)

		for point in del_points:
			if point in self.good_points:
				self.good_points.remove(point)

		if (self.num_frame == self.init_length):
			self.final_refinement()
			self.num_frame = 0
			self.state = 2

	# set new position after the initialisation
	def final_refinement(self):
		for ecoli in self.good_points:
				ecoli.set_pos_from_points(round(ecoli.x - self.roi_size_refine / 2), round(ecoli.y - self.roi_size_refine / 2))

	def cont_operation(self,im):
		weight = np.zeros((4))

		for ecoli in self.good_points:
			roi = []
			roi.append(im[max(0, int(ecoli.y - self.roi_size_corners / 2)):min(int(ecoli.y), self.width),
					   max(0, int(ecoli.x - self.roi_size_corners / 2)):min(int(ecoli.x), self.height)])
			roi.append(im[max(0, int(ecoli.y)):min(int(ecoli.y + self.roi_size_corners / 2), self.width),
					   max(0, int(ecoli.x - self.roi_size_corners / 2)):min(int(ecoli.x), self.height)])
			roi.append(im[max(0, int(ecoli.y)):min(int(ecoli.y + self.roi_size_corners / 2), self.width),
					   max(0, int(ecoli.x)):min(int(ecoli.x + self.roi_size_corners / 2), self.height)])
			roi.append(im[max(0, int(ecoli.y - self.roi_size_corners / 2)):min(int(ecoli.y), self.width),
					   max(0, int(ecoli.x)):min(int(ecoli.x + self.roi_size_corners / 2), self.height)])

			for i in range(4):
				weight[i] = (np.mean(roi[i]))

			x = weight[0] - weight[2]
			y = weight[1] - weight[3]
			ecoli.angles.append(np.arctan2(x, y))




	def init_keypoints(self, image):
		self.width, self.height = np.shape(image)
		keypoints = self.detector.detect(image)
		for point in keypoints:
			self.good_points.append(Ecoli_obj(point.pt[0],point.pt[1]))
		print "First Frame Analysed"
		self.state = self.state +1

	def filter_points(self,im):
		del_points = []
		for ecoli in self.good_points:
			# taking the smaller angle difference (in radian)
			ecoli.calc_angle_diff()
			#print(np.mean(ecoli.angle_diff))
			filtered_angle_diff = self.calc_filter(ecoli.angle_diff, 10)
			#if abs(np.mean(ecoli.angle_diff)) < 10:
			#ecoli.history.append(np.mean(filtered_angle_diff))
			#len_arr = len(ecoli.history)
			#if(len_arr>3):
				#if np.sum((np.abs(ecoli.history[len_arr-4:])<5))==4:
			if((np.mean(np.abs((filtered_angle_diff))))<5):
					#filtered_angle_diff = calc_filter(ecoli.angle_diff, 10)
					#print("Mean of Blob " + str(blob_nr) + "   equals to:  " + str(np.mean(ecoli.angle_diff)))
					# now save this shit!!!
				del_points.append(ecoli)
			else:
				ecoli.history.append(np.mean(filtered_angle_diff))
				print np.mean(filtered_angle_diff)
				#print ecoli.angles
				ecoli.angles = []
				a = len(ecoli.history)
				if (a>1):
					if(abs(ecoli.history[a-1])-abs(ecoli.history[a-2])>0):
						print "TURNING FASTER NOW"
					else:
						print "TURNING SLOWER"




		for point in del_points:
			if point in self.good_points:
				self.good_points.remove(point)

		#sacve this shiat
		blob_nr = 0
		if self.save_plots:
			for ecoli in self.good_points:
				im = cv2.circle(im, (np.int(ecoli.x), np.int(ecoli.y)),
								radius=10, color=(255))  # Show blobs
				# cv2.imshow("Keypoints", im)
				cv2.imwrite(self.storepath + 'blob' + str(blob_nr) + '.png', im)

				pp = PdfPages(self.storepath + 'blob' + str(blob_nr) + '.pdf')
				f1 = plt.figure()
				plt.plot(ecoli.angles[:])
				plt.suptitle(' Rotational Angle of Blob' + str(blob_nr), fontsize=15)
				plt.xlabel('Time in frames', fontsize=12)
				plt.ylabel('Rotational Angle', fontsize=12)
				pp.savefig(f1, bbox_inches='tight')

				f2 = plt.figure()
				plt.plot(filtered_angle_diff[:])
				plt.suptitle('Filtered rotation difference of Blob ' + str(blob_nr), fontsize=15)
				plt.xlabel('Time in frames', fontsize=12)
				plt.ylabel('Relative Rotational Angle', fontsize=12)
				pp.savefig(f2, bbox_inches='tight')

				f3 = plt.figure()
				plt.plot(ecoli.angle_diff[:])
				plt.suptitle('Raw rotation difference of Blob ' + str(blob_nr), fontsize=15)
				plt.xlabel('Time in frames', fontsize=12)
				plt.ylabel('Relative Rotational Angle', fontsize=12)
				pp.savefig(f3, bbox_inches='tight')
				pp.close()
				blob_nr = blob_nr + 1





# MEINER MEINUNG NACH HIER DANN WARSCH 2.NODE AUCH NOCH REIN WO DANN DIE OBST AVOIDANCE GEMACHT WIRD ODER SO

if __name__ == '__main__':
	rospy.init_node('aroma_motility_node', anonymous=False)
	aroma_motility_node = MotilityPipeline()
	rospy.on_shutdown(aroma_motility_node.onShutdown)
	rospy.spin()