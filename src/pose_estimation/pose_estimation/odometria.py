import cv2
import numpy as np
from camera_params import *
import os
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

#Defines
FRAME_RATE = 20

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
p = np.zeros(3)
q = np.zeros(4)

cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
fast = cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True)


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('Visual_Odometry')
        self.publisher_ = self.create_publisher(Pose, 'Pose', 10)
        timer_period = 1/FRAME_RATE  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        pose = Pose()
        pose.position.x = p[0]
        pose.position.y = p[1]
        pose.position.z = p[3] 
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        self.publisher_.publish(pose)
        self.get_logger().info('.')
        self.i += 1



search_params = dict(checks=50)

feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )

lk_params=dict(winSize  = (21,21), criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
# lk_params = dict( winSize  = (15, 15),
                #   maxLevel = 2,
                #   criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

def odometria():
    if cap.isOpened():
        cv2.namedWindow("odometry", cv2.WINDOW_AUTOSIZE)
        ret, imgk_1 = cap.read() 
        grayk_1 = cv2.cvtColor(imgk_1, cv2.COLOR_BGR2GRAY)
        grayk_1 = cv2.GaussianBlur(grayk_1, (5,5), 0)
        # kpk_1 = fast.detect(gray,None)
        R = np.zeros(shape=(3, 3))
        t = np.zeros(shape=(3, 3))
        frame_count=0

        while cv2.getWindowProperty("odometry", 0) >= 0:
            print('-----------START-----------')
            ret, imgk = cap.read()
            grayk = cv2.cvtColor(imgk, cv2.COLOR_BGR2GRAY)
            grayk = cv2.GaussianBlur(grayk, (5,5), 0)
            # kpk_1 = fast.detect(grayk_1)
            kpk_1 = cv2.goodFeaturesToTrack(grayk_1, mask=None, **feature_params)
            
            
            kpk, st, err = cv2.calcOpticalFlowPyrLK(grayk_1, grayk, kpk_1, None, **lk_params)
            if kpk is not None:
                good_new = kpk[st==1]
                good_old = kpk_1[st==1]

            E, _ = cv2.findEssentialMat(good_new, good_old, K, cv2.RANSAC, 0.999, 1.0, None)
            _, R, t, _ = cv2.recoverPose(E, good_old, good_new, K, R, t, None)

            if frame_count < 2:
                E, _ = cv2.findEssentialMat(good_new, good_old, K, cv2.RANSAC, 0.999, 1.0, None)
                _, R, t, _ = cv2.recoverPose(E, good_old, good_new, K, R, t, None)
                frame_count = frame_count+1
            else:
                E, _ = cv2.findEssentialMat(good_new, good_old, K, cv2.RANSAC, 0.999, 1.0, None)
                _, R, t, _ = cv2.recoverPose(E, good_old, good_new, K, R.copy(), t.copy(), None)

            print("\n-------------------------------------------------------\n\t Resultados")
            print(R)
            print(t)
            print('')

            print(type(imgk))


            img = cv2.drawKeypoints(imgk, kpk, None, color=(255,0,0))

            cv2.imshow('odometry', imgk)

            keyCode = cv2.waitKey(30) & 0xFF
            if keyCode == 27:
                break

        cap.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")

def main(args=None):
    rclpy.init(args=args)
    pose_publisher = MinimalPublisher()
    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    odometria()