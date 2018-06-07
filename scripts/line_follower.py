#! /usr/bin/env python
# -*- coding:utf-8

import roslib
import rospy
import sys 
import numpy as np
import cv_bridge
import mavros
import cv2

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
from time import sleep

class image_converter:
    
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size = 10)
        self.image_sub = rospy.Subscriber("/rover/front_test/image_front_raw", Image, self.callback)
        self.blocking_sub = rospy.Subscriber("rover/control/blocking", Bool, self.blocking_callback)

        self.parking_sub = rospy.Subscriber("rover/control/parking", Bool, self.parking_callback)
        

        self.order_sub = rospy.Subscriber("move",String,self.order_callback)       



        self.line_center = 250
        self.area = 6
        self.line_width = 50
        self.stabilizer = True
        self.blocking = False
        self.order = None
        self.parking = False
        




        self.yaw = 0
        self.throttle = 1700
                
    def blocking_callback(self, msg):
        self.blocking = msg.data


    def parking_callback(self, msg):
        self.parking = msg.data

    

    def order_callback(self,msg):
		self.order = msg.data


    def callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        rows, cols, ch = image.shape
        
        pts1 = np.float32([[55,120], [280,120], [0,160], [320,160]])
        pts2 = np.float32([[0,0], [320,0], [0,240], [320,240]])
                
        M = cv2.getPerspectiveTransform(pts1, pts2)
        #dst = cv2.warpPerspective(image, M, (300, 300))
        
        #hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                
        lower_yellow_line = np.array([20, 80, 80])      
        upper_yellow_line = np.array([40, 255, 255])
        
        lower_white_line = np.array([0, 0, 190])
        upper_white_line = np.array([255, 40, 255])
        
        mask_left = cv2.inRange(hsv, lower_yellow_line, upper_yellow_line)
        mask_right = cv2.inRange(hsv, lower_white_line, upper_white_line)
        
        
        #h, w, d = dst.shape
        '''
        search_top = h/10 * self.area
        search_bot = search_top + self.line_width
        search_across = w/2
        search_left = w/3
        search_right = w * 2/3
        '''
                
        search_top = rows/10 * self.area
        search_bot = search_top + self.line_width
        search_across = cols/2
        search_left = cols/3
        search_right = cols * 2/3
        
        # modify
        #mask_right[0:search_top, 0:w] = 0
        #mask_right[search_bot:h, 0:w] = 0
        #mask_right[0:h, 0:search_across] = 0
        
        mask_right[0:rows, 0:search_across] = 0
        mask_right[0:search_top, 0:cols] = 0
        mask_right[search_bot:rows, 0:cols] = 0
       
        #mask_left[0:search_top, 0:w] = 0
        #mask_left[search_bot:h, 0:w] = 0
        #mask_left[0:h, search_across:w] = 0
        
        mask_left[0:rows, search_across:cols] = 0       
        mask_left[0:search_top, 0:cols] = 0
        mask_left[search_bot:rows, 0:cols] = 0
        
        M_left = cv2.moments(mask_left)
        M_right = cv2.moments(mask_right)
        
        if self.blocking or self.parking:
            self.throttle = 1500
        else:
            self.throttle = 1700

        # left
        if M_left['m00'] > 0 and M_right['m00'] <= 0: # and not self.stablilizer:
            rospy.logdebug('Find Left')
            
            cx_left = int(M_left['m10']/M_left['m00'])
            cy_left = int(M_left['m01']/M_left['m00'])
           
            cx = cx_left + self.line_center
            cy = cy_left
            
            #self.pose_msg.xpose = cx
            #self.pose_msg.ypose = cy
            #self.pose_pub.publish(self.pose_msg)
            
            #dst = cv2.circle(dst, (cx_left, cy_left), 10, (0,0,255), -1)
            #dst = cv2.circle(dst, (cx, cy), 10, (0,255,0), -1)
            
            cv2.circle(image, (cx_left, cy_left), 10, (0,0,255), -1)
            cv2.circle(image, (cx, cy), 10, (0,255,0), -1)
            
            self.yaw = int(1500 + (cx - cols/2) * 1.5)
            
        
        # right
        elif M_left['m00'] <= 0 and M_right['m00'] > 0 : #and not self.stabilizer:
            rospy.logdebug('Find Right')
            
            cx_right = int(M_right['m10']/M_right['m00'])
            cy_right = int(M_right['m01']/M_right['m00'])
            
            cx = cx_right - self.line_center
            cy = cy_right
            
            #self.pose_msg.xpose = cx
            #self.pose_msg.ypose = cy
            #self.pose_pub.publish(self.pose_msg)
            
            #dst = cv2.circle(dst, (cx_right, cy_right), 10, (255, 0, 0), -1)
            #dst = cv2.circle(dst, (cx, cy), 10, (0,255,0), -1)
            
            cv2.circle(image, (cx_right, cy_right), 10, (255, 0, 0), -1)
            cv2.circle(image, (cx, cy), 10, (0, 255, 0), -1)
            
            self.yaw = int(1500 + (cx - cols/2) * 1.5)
            
        # both
        elif M_left['m00'] > 0 and M_right['m00'] > 0:
            rospy.logdebug('Find Both')
            
            self.stabilizer = False
            
            cx_left = int(M_left['m10']/M_left['m00'])
            cy_left = int(M_left['m01']/M_left['m00'])
            cx_right = int(M_right['m10']/M_right['m00'])
            cy_right = int(M_right['m01']/M_right['m00'])
            
            cx = (cx_left + cx_right) // 2
            cy = (cy_left + cy_right) // 2
            
            #self.pose_msg.xpose = cx
            #self.pose_msg.ypose = cy
            #self.pose_pub.publish(self.pose_msg)
            
            #dst = cv2.circle(dst, (cx_left, cy_left), 10, (0,0,255), -1)
            #dst = cv2.circle(dst, (cx_right, cy_right), 10, (255,0,0), -1)
            #dst = cv2.circle(dst, (cx, cy), 10, (0,255,0), -1)
            
            cv2.circle(image, (cx_left, cy_left), 10, (0,0,255), -1)
            cv2.circle(image, (cx_right, cy_right), 10, (255, 0, 0), -1)
            cv2.circle(image, (cx, cy), 10, (0,255,0), -1)
            
            self.yaw = int(1500 + (cx - cols/2) * 1.5)
            #self.roll = 0
           
        else:
            self.stabilizer = True
            #self.stabilizer_pub.publish(self.stailizer)
            
        cv2.imshow("Navigation", image)
        #cv2.imshow("Dst_Navigation", dst)
        #cv2.imshow("Left", mask_left)
        #cv2.imshow("Right", mask_right)
        
        cv2.waitKey(3)
        # deu_racer end
        
        
        if (self.yaw > 1900):
            self.yaw = 1900
        elif (self.yaw < 1100):
            self.yaw = 1100
                
        print 'yaw : ', self.yaw        
             
             
        msg = OverrideRCIn()
        
        msg.channels[0] = self.yaw              # roll
        msg.channels[1] = 0                     # pitch
        msg.channels[2] = self.throttle         # speed
        msg.channels[3] = 0                     # yaw
        msg.channels[4] = 0
        msg.channels[5] = 0
        msg.channels[6] = 0
        msg.channels[7] = 0
        
        self.pub.publish(msg)
       
        

def main(args):
    ic = image_converter()
    
    rospy.init_node('erle_rover_followline', anonymous=True, log_level=rospy.DEBUG)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    cv2.destroyAllWindows()
    
    
if __name__ == '__main__':
    main(sys.argv)
