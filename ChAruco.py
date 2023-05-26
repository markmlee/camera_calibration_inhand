 ###################################################################
 #
 # This helper class is for generating sets of [cam pose, end-effector pose]
 # for robot hand-eye calibration using the Franka Panda Robot.
 # ChAruco generated using the following link:
 # https://calib.io/pages/camera-calibration-pattern-generator
 #
 # Output : 
 # 
 # Input  : /
 #          /
 
 # E-mail : MoonRobotics@cmu.edu    (Lee Moonyoung)
 
 #
 # Versions :
 # v1.0
 ###################################################################

import numpy as np
import random
import cv2

import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError


class ChAruco:
    def __init__(self):
 
        rospy.init_node('detector')

        self.bridge = CvBridge()
        self.image_cv = None
        self.recevied_image = False

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)


        #(num COL, num ROW, size of checker, size of aruco tag, aruco definition) ***please verify COL x ROW ordering
        self.board = cv2.aruco.CharucoBoard_create(5, 3, 0.048, 0.037, self.aruco_dict)

        sub_image = rospy.Subscriber("/camera/color/image_raw", Image, self.get_pose_rb0)


    def get_pose_rb0(self,image_in):
        # rospy.loginfo('Got image')
        # rospy.logdebug('Got image')
        self.image_ros = image_in

        # Try to convert the ROS Image message to a CV2 Image
        try:
            self.image_cv = self.bridge.imgmsg_to_cv2(image_in, "bgr8")
        except CvBridgeError:
            rospy.logerr("CvBridge Error")
            rospy.loginfo("CvBridge Error")
            rospy.logdebug("CvBridge Error")

        # self.image_cv = self.bridge.imgmsg_to_cv2(image_in, "bgr8")
        self.recevied_image = True

    def get_image(self):
        return self.image_cv

    def get_image_shape(self):
        return self.image_cv.shape



    def get_offset(self, camera_matrix, dist_coeff, debug: bool = False):
        frame = self.get_image()
        corners, ids, rejected_points = cv2.aruco.detectMarkers(frame, self.aruco_dict)

        if corners is None or ids is None:
            return None
        if len(corners) != len(ids) or len(corners) == 0:
            return None

        ret, c_corners, c_ids = cv2.aruco.interpolateCornersCharuco(corners,ids, frame, self.board)
        if debug:
            image_copy = cv2.aruco.drawDetectedCornersCharuco(frame, c_corners, c_ids)

        gotpose, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(c_corners, c_ids, self.board, 
                                                              camera_matrix, dist_coeff, np.empty(1), np.empty(1))
        if gotpose:
            dist_coeff_copy = np.array(dist_coeff, dtype=np.float32).reshape(-1, 1)
            image_disp = cv2.drawFrameAxes(image_copy, camera_matrix, dist_coeff_copy, rvec, tvec, 0.1)
            cv2.imshow("Image", image_disp)
            cv2.waitKey(0)

        return gotpose, rvec, tvec
    
