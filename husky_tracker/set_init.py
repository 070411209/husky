import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 

import sys 
sys.path.append('/home/one/src/GAAS-Object-Tracking/KCF/build/devel/lib/python2.7/dist-packages')
from ros_kcf.srv import InitRect
from std_msgs.msg import Int32MultiArray
import car_config

class SetInit:
    def __init__(self, init_rect_img_topic='/camera/tracking/image'):
        print("Wait Service...")
        rospy.wait_for_service('init_rect')
        print("Done.")
        self.init_rect_service = rospy.ServiceProxy('init_rect', InitRect)
        self.x1 = None
        self.y1 = None
        self.x2 = None
        self.y2 = None
        self.pressed = False
        self.sub = rospy.Subscriber(init_rect_img_topic, Image, self.callback)
        self.bridge = CvBridge()
        self.frame = None
        rospy.spin()
    
    def callback(self, data):
        cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        if self.x1==None:
            cv2.imshow('image', cv_img)
        else:
            cv_img = cv2.rectangle(cv_img, (self.x1, self.y1), (self.x2, self.y2), (255,0,0),2)
            cv2.imshow('image', cv_img)
        cv2.setMouseCallback('image', self.draw_rect)
        if cv2.waitKey(10) & 0xFF == ord('s'):
            if self.x2 is not None:
                self.init_rect_service(self.x1, self.y1, self.x2, self.y2)
                print("Init rect :[%f, %f], [%f, %f] ", self.x1, self.y1, self.x2, self.y2)
                

    def draw_rect(self,event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.x1 = x
            self.y1 = y
            self.x2 = x+1
            self.y2 = y+1
            self.pressed = True
        elif event == cv2.EVENT_LBUTTONUP:
            self.x2 = x
            self.y2 = y
            self.pressed = False
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.pressed == True:
                self.x2 = x 
                self.y2 = y
            


if __name__=='__main__':
    rospy.init_node('camera_display', anonymous=True)
    
    init_rect_img_topic = "/realsense/color/image_raw"   # car_config.init_rect_img_topic

    SetInit(init_rect_img_topic)

    print("Test...")
