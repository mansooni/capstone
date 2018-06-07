#! /usr/bin/env python

import roslib
import sys
import rospy
import cv2
import cv_bridge
import numpy as np
import socket

from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import String

class image_converter:

  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/rover/front/image_front_raw",Image,self.callback)
    self.blocking_pub = rospy.Publisher("rover/control/blocking", Bool, queue_size = 1)
    
    self.order_pub = rospy.Publisher("move",String,queue_size = 10) # publish goal  

    self.blocking = False
    self.pub = False

	#connect to socket
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.sock.connect(("127.0.0.1",8080))

  def callback(self,msg):
    try:
      image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
      #image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError as e:
      print(e)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    red = np.uint8([[[0, 0, 255]]])
    hsv_red = cv2.cvtColor(red, cv2.COLOR_BGR2HSV)
    print hsv_red
    
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    
    mask = cv2.inRange(hsv, lower_red, upper_red)
    
    # labeling
    kernel = np.ones((15, 15), np.uint8)
    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    labelnum, labelimg, contours, GoCs = cv2.connectedComponentsWithStats(closing)
    
    max_indices_len = 0
    max_label = 0
    
    for label in xrange(1, labelnum):
        label_indices = np.where(labelimg == label)
        label_indices_len = len(label_indices[0])
        
        if label_indices_len > max_indices_len:
            max_indices_len = label_indices_len
            max_label = label
            
    circle_x, circle_y = GoCs[max_label]
    x, y, w, h, size = contours[max_label]
    
    image = cv2.circle(image, (int(circle_x), int(circle_y)), 5, (0, 255, 0), -1)
    image = cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 255), 2)
    
    if float(w)/float(h) > 1.5 and h > 10:
        if 150 < max_indices_len:
            self.blocking = True
            
            if self.pub == False:
				self.blocking_pub.publish(self.blocking)
				self.pub = True
				rospy.loginfo('blocking detect')
				self.sock.sendall("arrive 1".encode()) # send data to control center
				rospy.loginfo('send data success')
				


    else:
        self.blocking = False
        self.pub = False

	self.blocking_pub.publish(self.blocking)
        
    rospy.logdebug('max pixel = %d, msg %s' % (max_indices_len, self.blocking))

    cv2.imshow("blocking", image)
    #cv2.imshow("closing", closing)
    
    cv2.waitKey(3)


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True, log_level=rospy.DEBUG)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    sock.close()
    print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

