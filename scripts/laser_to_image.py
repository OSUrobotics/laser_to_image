#!/usr/bin/env python

# Software License Agreement (BSD License)
#
#  Copyright (c) 2018, Benjamin Narin
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the OSU Personal Robotics Group. nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
# Author: Benjamin Narin

import roslib
roslib.load_manifest('laser_to_image')
import numpy as np, cv2
import math
import rospy
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge, CvBridgeError

# Discretization Size
disc_size = .08
# Discretization Factor
disc_factor = 1/disc_size
# Max Lidar Range
max_lidar_range = 10
# Create Image Size Using Range and Discretization Factor
image_size = int(max_lidar_range*2*disc_factor)

class laser_to_image:
	def __init__(self):
		# Laser Scan To Subscribe to
		self.joy_sub = rospy.Subscriber('/scan_multi',LaserScan,self.cloud_to_image_callback)
		# Publisher for Image
		self.pub = rospy.Publisher("scan_to_image",Image, queue_size = 10)
		# CvBridge Setup
		self.bridge = CvBridge()

	def cloud_to_image_callback(self,scan):
		# Store maxAngle of lidar
		maxAngle = scan.angle_max
		# Store minAngle of lidar
		minAngle = scan.angle_min
		# Store angleInc of lidar
		angleInc = scan.angle_increment
		# Store maxLength in lidar distances
		maxLength = scan.range_max
		# Store array of ranges
		ranges = scan.ranges
		# Calculate the number of points in array of ranges
		num_pts = len(ranges)
		# Create Array for extracting X,Y points of each data point
		xy_scan = np.zeros((num_pts,2))
		# Create 3 Channel Blank Image
		blank_image = np.zeros((image_size,image_size,3),dtype=np.uint8)
		# Loop through all points converting distance and angle to X,Y point
		for i in range(num_pts):
			# Check that distance is not longer than it should be
			if (ranges[i] > 10) or (math.isnan(ranges[i])):
				pass
			else:
				# Calculate angle of point and calculate X,Y position
				angle = minAngle + float(i)*angleInc
				xy_scan[i][0] = float(ranges[i]*math.cos(angle))
				xy_scan[i][1] = float(ranges[i]*math.sin(angle))

		# Loop through all points plot in blank_image
		for i in range(num_pts):
			pt_x = xy_scan[i,0]
			pt_y = xy_scan[i,1]
			if (pt_x < max_lidar_range) or (pt_x > -1 * (max_lidar_range-disc_size)) or (pt_y < max_lidar_range) or (pt_y > -1 * (max_lidar_range-disc_size)):
				pix_x = int(math.floor((pt_x + max_lidar_range) * disc_factor))
				pix_y = int(math.floor((max_lidar_range - pt_y) * disc_factor))
				if (pix_x > image_size) or (pix_y > image_size):
					print "Error"
				else:
					blank_image[pix_y,pix_x] = [0,0,255]

		# Convert CV2 Image to ROS Message
		img = self.bridge.cv2_to_imgmsg(blank_image, encoding="bgr8")
		# Publish image
		self.pub.publish(img)

		# Use CV to show image
		cv2.imshow('result', blank_image), cv2.waitKey(3)
		blank_image = np.zeros((image_size,image_size,3))


if __name__=='__main__':
	rospy.init_node('laser_to_image')
	laser_to_image = laser_to_image()
	rospy.spin()
