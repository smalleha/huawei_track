#!/usr/bin/env python

"""follower_ros.py: Robot will follow the Yellow Line in a track"""

__author__  = "Arjun S Kumar"

import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('/limo/color/image_raw', 
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                       Twist, queue_size=1)
    self.twist = Twist()
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
    # change below lines to map the color you wanted robot to follow
    lower_yellow = numpy.array([ 0,  0,  30])
    upper_yellow = numpy.array([90, 30, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    image2 = cv2.flip(image,-1)    

    h, w, d = image2.shape
    #print(h,w,d)
    search_top = 3*h/4
    search_bot = 3*h/4+20
    #print(search_top,search_bot)
    print(h,w)
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image2, (cx, cy), 20, (0,0,255), -1)
      # CONTROL starts
      
      err = w/2 - cx
      print(err)
      self.twist.linear.x = 0.1
      self.twist.angular.z = -float(err) / 100

      self.cmd_vel_pub.publish(self.twist)
      # CONTROL ends
    cv2.imshow("mask",mask)
    cv2.imshow("output", image)
    cv2.imshow("output2", image2)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
