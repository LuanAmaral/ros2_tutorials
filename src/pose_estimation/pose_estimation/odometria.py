from re import I
import cv2
import numpy as np
from .camera_params import *
import os
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

#Defines
FRAME_RATE = 20
    # params for corner detection
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )
    # Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15, 15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=FRAME_RATE,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )
#Global Variables

cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
fast = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)

def convert_matrix2quaternions(R):
    w = np.sqrt(float(1)+ R[0,0] + R[1,1] + R[2,2])*0.5
    x = (R[2,1]-R[1,2])/(4*w)
    y = (R[0,2]-R[2,0])/(4*w)
    z = (R[1,0]-R[0,1])/(4*w)
    return np.array([w,x,y,z])


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('Visual_Odometry')
        self.publisher_ = self.create_publisher(PoseArray, 'Pose', 10)
        timer_period = 1/FRAME_RATE  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.R = np.identity(3)
        self.t = np.array([[0],[0],[0]])
        self.q = np.zeros(4)
        self.pose = PoseArray()
        self.pose.header.frame_id = "map"
        if cv2.getWindowProperty("odometry", 0) >= 0:
            ret, imgk_1 = cap.read()
            self.grayk_1 = cv2.cvtColor(imgk_1, cv2.COLOR_BGR2GRAY)
            self.grayk_1 = cv2.GaussianBlur(self.grayk_1, (5,5), 0)
            # self.kpk_1 = fast.detect(self.grayk_1)
            self.kpk_1 = cv2.goodFeaturesToTrack(self.grayk_1, mask = None,  **feature_params)

    def timer_callback(self):
        self.odometria()

        pose = Pose()
        pose.position.x = float(self.t[0])
        pose.position.y = float(self.t[1])
        pose.position.z = float(self.t[2]) 
        pose.orientation.w = float(self.q[0])
        pose.orientation.x = float(self.q[1])
        pose.orientation.y = float(self.q[2])
        pose.orientation.z = float(self.q[3])

        self.pose.header.stamp= self.get_clock().now().to_msg()
        self.pose.poses.append(pose)

        self.publisher_.publish(self.pose)
        # self.get_logger().info('Number of points: "%d"', self.kpk)
        self.i += 1

    def odometria(self):
        if cv2.getWindowProperty("odometry", 0) >= 0:
            ret, imgk = cap.read()
            grayk = cv2.cvtColor(imgk, cv2.COLOR_BGR2GRAY)
            grayk = cv2.GaussianBlur(grayk, (5,5), 0)
            # kpk_1 = fast.detect(grayk_1)
            # kpk = fast.detect(grayk)
        
            kpk, st, err = cv2.calcOpticalFlowPyrLK(self.grayk_1, grayk, self.kpk_1, None, **lk_params)
            E, mask = cv2.findEssentialMat(self.kpk_1, kpk, K, cv2.FM_RANSAC, 0.90, 1)
            ret, R_, t_, mask = cv2.recoverPose(E, self.kpk_1, kpk, K)
            
            self.t = self.R.dot(t_) + self.t
            self.R = self.R.dot(R_)
            self.q = convert_matrix2quaternions(self.R)

            # imgk = cv2.drawKeypoints(imgk, np.float32(self.kpk_1), None, color=(255,0,0))
            cv2.imshow('odometry', imgk)

            # self.kpk_1 = fast.detect(grayk)
            self.kpk_1 = cv2.goodFeaturesToTrack(grayk, mask = None,  **feature_params)
            self.grayk_1 = grayk

            keyCode = cv2.waitKey(30) & 0xFF
            if keyCode == 27:
                print("ERROR")
                cap.release()
                cv2.destroyAllWindows()
            


def main(args=None):
    if cap.isOpened():
        cv2.namedWindow("odometry", cv2.WINDOW_AUTOSIZE)
        print('CAP IS OPENED\n')
        rclpy.init(args=args)
        pose_publisher = MinimalPublisher()
        rclpy.spin(pose_publisher)
        pose_publisher.destroy_node()
        rclpy.shutdown()
    else:
        print("Unable to open camera")
