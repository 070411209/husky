#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

from items import MessageItem
import time

class SetInit:
    def __init__(self, init_rect_img_topic):
        (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
        tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
        self.tracker_type = "MEDIANFLOW"
        self.x1 = None
        self.y1 = None
        self.x2 = None
        self.y2 = None
        self.box = None
        self.coord = None
        self.pressed = False
        self.sub = rospy.Subscriber(init_rect_img_topic, Image, self.callback)
        self.bridge = CvBridge()
        # self.frame = None
        self.isWorking = False
        self.draw_coord = True
        rospy.spin()
    
    def callback(self, data):
        cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow('image', cv_img)
        if cv2.waitKey(10) & 0xFF == ord('s'):
            print("Tracker Type : ", self.tracker_type)
            bbox = cv2.selectROI(cv_img, False)
            self.initWorking(cv_img, bbox)
        item = self.track(cv_img)
        item.getMessage()
        cv2.imshow("track", item.getFrame())         
        

    def initWorking(self,frame,box):
        if self.tracker_type == 'BOOSTING':
            self.tracker = cv2.TrackerBoosting_create()
        if self.tracker_type == 'MIL':
            self.tracker = cv2.TrackerMIL_create()
        if self.tracker_type == 'KCF':
            self.tracker = cv2.TrackerKCF_create()
        if self.tracker_type == 'TLD':
            self.tracker = cv2.TrackerTLD_create()
        if self.tracker_type == 'MEDIANFLOW':
            self.tracker = cv2.TrackerMedianFlow_create()
        if self.tracker_type == 'GOTURN':
            self.tracker = cv2.TrackerGOTURN_create()

        '''
        追踪器工作初始化
        frame:初始化追踪画面
        box:追踪的区域
        '''
        if not self.tracker:
            raise Exception("追踪器未初始化")
        status = self.tracker.init(frame,box)
        if not status:
            raise Exception("追踪器工作初始化失败")
        
        self.coord = box
        self.isWorking = True
        print("ROI: ", self.coord)

    def track(self,frame):
        message = None
        if self.isWorking:
            status, self.coord = self.tracker.update(frame)
            if status:
                message = {"coord":[((int(self.coord[0]), int(self.coord[1])),(int(self.coord[0] + self.coord[2]), int(self.coord[1] + self.coord[3])))]}
                if self.draw_coord:
                    p1 = (int(self.coord[0]), int(self.coord[1]))
                    p2 = (int(self.coord[0] + self.coord[2]), int(self.coord[1] + self.coord[3]))
                    cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
                    message['msg'] = "is tracking"
        return MessageItem(frame,message)

if __name__=='__main__':
    rospy.init_node('init_tracker', anonymous=True)
    
    init_img_topic = car_config.init_rect_img_topic

    SetInit(init_img_topic)

    print("test")
