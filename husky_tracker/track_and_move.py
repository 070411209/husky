#encoding=utf-8
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
import message_filters

from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
from pyquaternion import Quaternion


import sys 
sys.path.append('/home/one/src/GAAS-Object-Tracking/KCF/build/devel/lib/python2.7/dist-packages')
from ros_kcf.srv import InitRect
from std_msgs.msg import Int32MultiArray
# from commander import Commander
import time
import car_config


#Keep the object in the center.
class SimpleTrackAndMove:
    def __init__(self, resolution,  K, left_topic='/gi/simulation/left/image_raw', right_topic='/gi/simulation/right/image_raw', 
                        object_position_topic='/track_rect_pub' ,move_threshold=0.2, altitude=3000, stereo=None, baseline=None, ):
        rospy.init_node("tracker_node")
        rate = rospy.Rate(20)
        self.takeoff_height = 3.2
        self.fx = float(K[0][2])
        self.fy = float(K[1][2])

        self.idx = 0
        # self.con = Commander()
        time.sleep(0.1)
        '''
        ros publishers
        '''
        self.local_target_pub = rospy.Publisher('/husky_beta/husky_velocity_controller/cmd_vel', Twist, queue_size=10)        
        '''
        ros subscribers
        '''
        self.left_sub = message_filters.Subscriber(left_topic, Image)
        self.right_sub = message_filters.Subscriber(right_topic, Image)
        self.object_position_sub = message_filters.Subscriber(object_position_topic, Int32MultiArray)
        ts = message_filters.ApproximateTimeSynchronizer([self.left_sub, self.right_sub, self.object_position_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.update_new_position)


        self.prev_position = None
        self.cur_position = None
        self.width = resolution[0]
        self.hight = resolution[1]
        self.cur_target_pose = None
 
        self.camera_center = self.get_center((0,0, self.width, self.hight))
        self.move_threshold = move_threshold
        self.altitude = altitude

        self.stereo = stereo
        if stereo is not None:
            assert baseline is not None
        self.baseline = baseline

        self.bridge = CvBridge()

        rospy.spin()


    def check_border(self, box):
        x1 = max(box[0],0) if box[0] <= self.width else self.width
        y1 = max(box[1],0) if box[1] <= self.hight else self.hight
        x2 = max(box[2],0) if box[2] <= self.width else self.width
        y2 = max(box[3],0) if box[3] <= self.hight else self.hight
        return (x1, y1, x2, y2)

    def get_center(self, box):
        x1 = box[0]
        y1 = box[1]
        x2 = box[2]
        y2 = box[3]
        return (abs(x2+x1)/2., abs(y2+y1)/2.)
    
    def update_new_position(self, left, right, track_result):
        left_img = self.bridge.imgmsg_to_cv2(left, "mono8")
        right_img = self.bridge.imgmsg_to_cv2(right, "mono8")

        box = track_result.data
        self.cur_position = self.check_border(box)
        if self.idx%5 == 0:
            left_img_rgb = self.bridge.imgmsg_to_cv2(left, "bgr8")
            left_img_rgb = cv2.rectangle(left_img_rgb, (self.cur_position[0],self.cur_position[1]), (self.cur_position[2],self.cur_position[3]), (255,0,0))
            cv2.imwrite('a.png', left_img_rgb)
            # cv2.imshow('rect_image', left_img_rgb)
            # cv2.waitKey(2)
            # cv2.destroyAllWindows()
        self.idx+=1

        # self.update_altitude(left_img, left_img, track_result)
        new_move = self.get_next_move()
        if new_move is not None:
            print('Move:',new_move)
            self.local_target_pub.publish(self.cur_target_pose)
            time.sleep(0.5)

    def update_altitude(self, left, right, track_result):
        if self.stereo is not None:
            disparity = stereo.compute(imgL,imgR)
            disparity = cv2.convertScaleAbs(disparity, disparity, alpha = 1./16)
            mean_disparity = np.mean(disparity[track_result[1]:track_result[3], track_result[0]:track_result[2]])
            mean_depth = self.baseline* self.fx / mean_disparity 
            self.altitude = mean_depth
        else:
            pass
    
    def get_point_dist(self, p1, p2):
        return np.sqrt(np.sum(np.square(np.asarray(p1)-np.asarray(p2))))
    
    def get_dist(self, a, b):
        return np.sqrt(np.sum(np.square(a)+np.square(b)))
    
    def get_next_move(self):
        cur_center = self.get_center(self.cur_position)
        # print("cur_center : ", cur_center)
        # print("-- ", cur_center[0]-self.fx, cur_center[1]-self.fy)
        #|----^ X---|
        #|----|-----|
        #|Y---|-----|
        #<————+-----|
        #|----------|
        #|----------|
        x_yaw = -(cur_center[0]- self.camera_center[0])/200.0
        y_x = -(cur_center[1] - self.camera_center[1])/200.0
        print("x_yaw = ", x_yaw, " y_x = ", y_x)

        self.cur_target_pose = self.construct_target(y_x, x_yaw)

        if abs(y_x) > self.move_threshold:
            return True
        else:
            return None

    def construct_target(self, x, yaw, yaw_rate = 1):
        target_raw_pose=Twist()
        target_raw_pose.linear.x = x
        target_raw_pose.angular.z = 0
        return target_raw_pose

if __name__ == '__main__':
    K = car_config.K 
    resolution = car_config.resolution
    left_topic = car_config.left_topic
    right_topic = car_config.right_topic
    object_position_topic = car_config.object_position_topic

    SimpleTrackAndMove(resolution, K, left_topic=left_topic, right_topic=right_topic, object_position_topic=object_position_topic)
