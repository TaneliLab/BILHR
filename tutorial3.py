#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed,Bumper,HeadTouch
from sensor_msgs.msg import Image,JointState
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import csv

import HSV_Nao_blob_DET_v2 as NBD

class tutorial3:

    def __init__(self):
        self.blobX = 0
        self.blobY = 0
        self.blobSize = 0
        self.shoulderRoll = 0
        self.shoulderPitch = 0
        # For setting the stiffnes of single joints
        self.stiffnesPub = 0

    def image_cb(self,data):
        bridge_instance = CvBridge()
        try:
<<<<<<< HEAD
            # br = CvBridge()

            # Output debugging information to the terminal
            rospy.loginfo("receiving video frame__B_DET")
=======
            br = CvBridge()
 
            # Output debugging information to the terminal
            rospy.loginfo("receiving video frame")
            
            # Convert ROS Image message to OpenCV image
            current_frame = br.imgmsg_to_cv2(data)
            current_frame = cv2.cvtColor(current_frame,cv2.COLOR_BGR2HSV)

            params = cv2.SimpleBlobDetector_Params()

            # Change thresholds
            params.minThreshold = 1
            params.maxThreshold = 255


            # Filter by Area.
            params.filterByArea = True
            params.minArea = 30

            # Filter by Circularity
            params.filterByCircularity = True
            params.minCircularity = 0.1

            params.filterByColor = True
            params.blobColor = 255

            
            lower_green = np.array([160,100,20])
            upper_green = np.array([179,255,255])
            # Threshold the HSV image to get only blue colors
            mask = cv2.inRange(current_frame, lower_green, upper_green)
            
            erode_kernel = np.ones((3,3),np.uint8)
            eroded_img = cv2.erode(mask,erode_kernel,iterations = 1)
        
            # dilate
            dilate_kernel = np.ones((10,10),np.uint8)
            dilate_img = cv2.dilate(eroded_img,dilate_kernel,iterations = 1)
            detector = cv2.SimpleBlobDetector_create(params)
            # Detect blobs.
            
            # Create a detector with the parameters
            # OLD: detector = cv2.SimpleBlobDetector(params)
            detector = cv2.SimpleBlobDetector_create(params)


            # Detect blobs.
            #keypoints = detector.detect(image_hsv)
            keypoints = detector.detect(dilate_img)
            max = 0
            if(len(keypoints) >= 0):
                max = 0
                xCoord = 0
                yCoord = 0
                maxObject = None
                for blob in keypoints:
                    if(blob.size>max):
                        max = blob.size
                        xCoord = blob.pt[0]
                        yCoord = blob.pt[1]
                        maxObject = blob

            # Round the coordinates to get pixel coordinates:
            xPixel = round(xCoord)
            yPixel = round(yCoord)
>>>>>>> cmac

            # Convert ROS Image message to OpenCV image
            src = bridge_instance.imgmsg_to_cv2(data,"bgr8") #rgb 8bit
            # current_frame = br.imgmsg_to_cv2(data)

            #[hkh]Getting x,y coordinate
            # cv_image = CVA.blob_detection(src)
            blob_detection_ret = NBD.blob_detection(src) #[hkh]
            if not isinstance(blob_detection_ret, type(None)):
                self.blobX, self.blobY = blob_detection_ret
            
<<<<<<< HEAD
            # cv2.imshow("Keypoints", cv_image) #Keypoints are already implemented above
=======
            self.blobX = xPixel
            self.blobY = yPixel
            self.blobSize = max

            # For better readability, round size to 3 decimal indices
            blobSize = round(max, 3)

            rospy.loginfo("Biggest blob: x Coord: " + str(xPixel) + " y Coord: " + str(yPixel) + " Size: " + str(blobSize))

            im_with_keypoints = cv2.drawKeypoints(current_frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            #cv2.circle(frame,(int(kp_max.pt[0]),int(kp_max.pt[1])),int(kp_max.size),(0,255,0),2)
>>>>>>> cmac

            cv2.waitKey(3)

        except CvBridgeError as e:
            rospy.logerr(e)

    # For reading in touch on TB1
    # also collect training data here(?)
    def touch_cb(self,data):
        rospy.loginfo("touch button: "+str(data.button)+" state: "+str(data.state))
        if data.button == 1 and data.state == 1:
            # save those values to a file
            # Check if blob has a size greater 0:
            # Save current variable value to not have it change again due to callback
            saveX = self.blobX
            saveY = self.blobY
            savePitch = self.shoulderPitch
            saveRoll = self.shoulderRoll
            size = self.blobSize
<<<<<<< HEAD
            #if saveX and saveY != 0:
            rospy.loginfo("----- wrote to file -------")
            # open the file in the write mode
            f = open("/home/bio/Desktop/BIHR_Nao2022/src/tutorial_3/scripts/samples.csv", "a")
            writer = csv.writer(f)
            row = [saveX, saveY, savePitch, saveRoll]
            rospy.loginfo(row)
            writer.writerow(row)
            f.flush()
            f.close()
=======
            if size > 0:
                # open the file in the write mode
                f = open('samples.csv', 'w')
                writer = csv.writer(f)
                row = [saveX, saveY, savePitch, saveRoll]
                writer.writerow(row)
                f.close()
>>>>>>> cmac



    def joints_cb(self,data):
        for index, jointNames in enumerate(data.joint_names):
            if jointNames == "RShoulderPitch":
                self.shoulderPitch = data.joint_angles[index]
                continue
                #rospy.loginfo("save")
            elif jointNames.name == "RShoulderRoll":
                self.shoulderRoll = data.joint_angles[index]
            


    def tutorial3_execute(self):
        rospy.init_node('tutorial3_node',anonymous=True) #initilizes node, sets name

        # create several topic subscribers
        rospy.Subscriber("joint_states",JointAnglesWithSpeed,self.joints_cb)
        rospy.Subscriber("tactile_touch",HeadTouch,self.touch_cb)
        rospy.Subscriber("/nao_robot/camera/top/camera/image_raw",Image,self.image_cb)

        self.stiffnesPub = rospy.Publisher("joint_stiffness", JointState, queue_size=10)

        # Set the initial stiffnesses
        stiffnessState = JointState()
        stiffnessState.header = Header()
        stiffnessState.position = []
        stiffnessState.velocity = []

        stiffnessState.name = ['HeadYaw', 'HeadPitch', 'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw',
  'LHand', 'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll', 'RHipYawPitch',
  'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll', 'RShoulderPitch', 'RShoulderRoll',
  'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand']


        stiffnessState.effort = [0.9, 0.9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        print(stiffnessState)
        self.stiffnesPub.publish(stiffnessState)

       

        rospy.spin()
    




if __name__=='__main__':
    # instantiate class and start loop function
    node_instance = tutorial3()
    node_instance.tutorial3_execute()

