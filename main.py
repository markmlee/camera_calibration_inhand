import numpy as np
import rospy

import time
import ChAruco
import cv2
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray
import csv
import sys


import tf
import yaml

 ###################################################################
 #
 # This python script is for generating sets of [cam pose, end-effector pose]
 # for robot hand-eye calibration using the Franka Panda Robot.
 #
 # Usage : Move robot to desired positions then press [enter] to record...repeat. 
 #
 # Output : cam_transform.csv, ee_transform.csv
 # 
 # Input  : /camera/color/image_raw (published through realsense2_camera package)
 #          /tf (published through Frankapy API)
 #          ChAruco board 
 
 # E-mail : MoonRobotics@cmu.edu    (Lee Moonyoung)
 
 #
 # Versions :
 # v1.0
 ###################################################################
 

#get multiple frames
desired_num_points_for_calib = 20

# filenames to store saved csv files
f_cam = open('./data/cam_transform.csv', 'w')
f_ee = open('./data/ee_transform.csv', 'w')

# Use guide mode for collecting samples
use_guide_mode = False


def load_camera_matrix():

    with open('camera_matrix.yaml', 'r') as file:
        yaml_load = yaml.safe_load(file)

    #cam matrix, dist coeff from RealsenseD435 factory settings
    #read in from /camera/color/camera_info ROS topic
    camera_matrix = np.array([
        [yaml_load['cam_k_matrix'][0], yaml_load['cam_k_matrix'][1], yaml_load['cam_k_matrix'][2]], 
        [yaml_load['cam_k_matrix'][3], yaml_load['cam_k_matrix'][4], yaml_load['cam_k_matrix'][5]],
        [yaml_load['cam_k_matrix'][6], yaml_load['cam_k_matrix'][7], yaml_load['cam_k_matrix'][8]]
    ])

    dist_coeff = np.array([0,0,0,0,0])

    return camera_matrix, dist_coeff

def wait_for_first_image(ChAruco_instance):

    #wait for image callback thread before proceeding
    while ChAruco_instance.recevied_image == False:
        time.sleep(1)

    #display received image for debugging
    img = ChAruco_instance.get_image()
    img_size = ChAruco_instance.get_image_shape
    # print(f"press ENTER to close image viewer")
    # cv2.imshow("image", img) 
    # cv2.waitKey(0)

    return img


# ====================================================================================================
if __name__ == "__main__":
    
    print(" ----------- start  of script ----------- ")

    # load camera matrix from yaml file
    camera_matrix, dist_coeff = load_camera_matrix()
    
    # wait for camera to be ready
    ChAruco_instance = ChAruco.ChAruco()
    img = wait_for_first_image(ChAruco_instance)

    # create the csv writer
    writer = csv.writer(f_cam)
    writer_ee = csv.writer(f_ee)

    #For desired number of calib pts, move robot and collect [cam pose, EE pose]
    for i in range (desired_num_points_for_calib):


        # Go to some position and then record stuff
        print(f"this is {i} index")
        # img = ChAruco_instance.get_image()
        # cv2.imshow("image", img)
        # cv2.waitKey(0) #<------ stopper until moving robot to desired position


        # #get EE
        tf_listener = tf.TransformListener()
        rospy.sleep(1)
        (ee_trans,ee_quat) = tf_listener.lookupTransform('/link_base', '/link_eef', rospy.Time(0))
        
        
        print(f" ******* RELOCATE THE ROBOT BEFORE PRESSING ENTER ********* ")
        # get camera pose from camera
        gotpose, rot, trans = ChAruco_instance.get_offset(camera_matrix, dist_coeff, debug=True)
        if trans.ndim > 1:
            assert trans.squeeze().ndim == 1, 'Should have one dimension for (x, y, z)'
            trans = trans.squeeze()

        # print(f"pose, rot,trans {gotpose,rot,trans}")
        R_np,_ = cv2.Rodrigues(rot)
        R = R.from_matrix(R_np)
        quat = R.as_quat()


        

        cam_row = [trans[0], trans[1], trans[2], quat[0], quat[1], quat[2], quat[3] ]
        # write a row to the csv file
        writer.writerow(cam_row)

        ee_row = [ee_trans[0], ee_trans[1], ee_trans[2], ee_quat[0], ee_quat[1], ee_quat[2], ee_quat[3] ]
        # write a row to the csv file
        writer_ee.writerow(ee_row)

        #print out ee, cam
        print(f" ee pose : {ee_row}, cam pose : {cam_row}")


    # close the file
    f_cam.close()
    f_ee.close()
    print(" ----------- end of script ----------- ")
