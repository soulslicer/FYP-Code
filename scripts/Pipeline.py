#!/usr/bin/python 

# Base libraries
import cv2
import time
import string
import json
import os
import numpy as np

# ROS
import roslib
import rospy
import tf
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import *
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from sonar.cfg import PipelineConfig

# Others
from Ros import *
from depth import *
from Algorithms import *
from sonar.msg import *

class Pipeline():

    def __init__(self):

        # Algorithms
        self.ros = ROS(ros_callback=self.ros_callback, ui_callback=self.ui_callback, mode="Pipeline")
        self.cvAverage = CvAverage(width=394, height=480, maxSize=2)
        self.lkSonar = LK(sonar=True, points_wanted=5)
        self.lkCamera = LK(sonar=False, points_wanted=2)
        self.featureExtractor = FeatureExtractor()

        # Other variables
        self.currTime = 0
        self.sonarImageMouse = (0,0)
        self.cameraImageMouse = (0,0)
        self.mode = 2

        # Data capture
        self.data_list = []
        self.currDirectory = os.getcwd()
        self.writeDirectory = "/"

        # FYP Algorithms
        self.mapper = Mapper()
        self.pf = ParticleFilter(Sonar_resolution=(394,480), Camera_resolution=(780,580), Npop_particles=2000)

        srv = Server(PipelineConfig, self.dynamic_callback)
        self.client = Client("Pipeline", timeout=30, config_callback=None)
        # self.client.update_configuration({"TrackingMode": 0})
        # self.pf.set_position(pos=(-1.5,-0.2,5.0), variance=(0.1,0.1,0.1))

    """""""""""""""""""""""""""""""""
    Start/Stop
    """""""""""""""""""""""""""""""""

    def start(self):
        self.ros.start()

    def stop(self):
        self.ros.stop()

    """""""""""""""""""""""""""""""""
    Callbacks
    """""""""""""""""""""""""""""""""

    def write_data(self, filename):
        filename = self.currDirectory + self.writeDirectory + filename
        f = open(filename,'w')
        for dataset in self.data_list:
            for item in dataset:
                f.write("%s " % str(round(item,6)))
            f.write("\n")
        f.close()
        print filename

    def ui_callback(self, ui_data):
        for key in ui_data.keys():
            value = ui_data[key]
            if key == "Threshold":
                self.threshold = value
            if key == "SonarImageMouse":
                self.sonarImageMouse = value
            if key == "CameraImageMouse":
                self.cameraImageMouse = value
            if key == "SaveData":
                self.save_data(self.sonarImageMouse, self.cameraImageMouse)
            if key == "WriteData":
                self.write_data(value)

    """""""""""""""""""""""""""""""""
    Publishing and data processing
    """""""""""""""""""""""""""""""""

    def publish_mapping(self, sonarArray):
        sonarObjects = sonar_objects()
        for sonarItem in sonarArray:
            sonarObject = sonar_object() 
            if sonarItem["SonarPosition"] is not None:
                sonarObject.sonarPosition = Point32(sonarItem["SonarPosition"][0], sonarItem["SonarPosition"][1], sonarItem["SonarPosition"][2])
            if sonarItem["VehiclePosition"] is not None:
                sonarObject.vehiclePosition = Point32(sonarItem["VehiclePosition"][0], sonarItem["VehiclePosition"][1], sonarItem["VehiclePosition"][2])
            for cnt in sonarItem["Contour"]:
                sonarObject.contour.points.append(Point32(cnt[0][0],cnt[0][1],0))
            sonarObject.centroid = Point32(sonarItem["Centroid"][0], sonarItem["Centroid"][1], 0)
            if sonarItem["PixelVector"] is not None:
                sonarObject.pixelVector = Point32(sonarItem["PixelVector"][0], sonarItem["PixelVector"][1], 0)
            sonarObject.majorLength = sonarItem["MA"]
            sonarObject.minorLength = sonarItem["ma"]
            sonarObject.cornerPoints.points = (Point32(sonarItem["Points"][0][0], sonarItem["Points"][0][1], 0), Point32(sonarItem["Points"][1][0], sonarItem["Points"][1][1], 0), Point32(sonarItem["Points"][2][0], sonarItem["Points"][2][1], 0), Point32(sonarItem["Points"][3][0], sonarItem["Points"][3][1], 0))
            sonarObject.polarCentre = Point32(sonarItem["RTheta"][0], sonarItem["RTheta"][1], 0)
            sonarObject.polarLeft = Point32(sonarItem["RThetaLeftSide"][0], sonarItem["RThetaLeftSide"][1], 0)
            sonarObject.polarRight = Point32(sonarItem["RThetaRightSide"][0], sonarItem["RThetaRightSide"][1], 0)
            sonarObject.trackValid = sonarItem["TrackValid"]
            if sonarItem["Mapping"] is not None:
                sonarObject.mapping = camera_mapping(
                    leftLine=[Point32(sonarItem["Mapping"]["LeftLine"][0][0], sonarItem["Mapping"]["LeftLine"][0][1], 0),Point32(sonarItem["Mapping"]["LeftLine"][1][0], sonarItem["Mapping"]["LeftLine"][1][1], 0)], 
                    rightLine=[Point32(sonarItem["Mapping"]["RightLine"][0][0], sonarItem["Mapping"]["RightLine"][0][1], 0),Point32(sonarItem["Mapping"]["RightLine"][1][0], sonarItem["Mapping"]["RightLine"][1][1], 0)], 
                    centreLine=[Point32(sonarItem["Mapping"]["CentreLine"][0][0], sonarItem["Mapping"]["CentreLine"][0][1], 0),Point32(sonarItem["Mapping"]["CentreLine"][1][0], sonarItem["Mapping"]["CentreLine"][1][1], 0)], 
                )
            sonarObjects.sonar_objects.append(sonarObject)
        self.ros.sonar_objects_publish(sonarObjects)

    def add_calibration_data(sonarItem, buoyItem):
        sonarRTheta = Img.sonar_pixel(sonarItem["Centroid"][0],sonarItem["Centroid"][1])
        self.data_list.append([
            sonarItem["Centroid"][0],                   # Sonar Mouse X
            sonarItem["Centroid"][1],                   # Sonar Mouse Y
            sonarRTheta[0],                             # Sonar R
            sonarRTheta[1],                             # Sonar Theta
            buoyItem["Centroid"][0],                    # Camera Mouse X
            buoyItem["Centroid"][1],                    # Camera Mouse Y
            self.data["Vehicle_Position"][0],           # Pose X
            self.data["Vehicle_Position"][1],           # Pose Y
            self.data["Vehicle_Position"][2],           # Pose Z
            self.data["IMU_RPY"][0],                    # Pose R
            self.data["IMU_RPY"][1],                    # Pose P
            self.data["IMU_RPY"][2],                    # Pose Y
            0,                                          # Pose XQ
            0,                                          # Pose YQ
            0,                                          # Pose ZQ
            0,                                          # Pose WQ
            self.data["Vehicle_Position"][1],           # Depth
            0,                                          # Brightness,
            self.data["Vehicle_Velocity"][0],           # Vehicle Vector X
            self.data["Vehicle_Velocity"][1],           # Vehicle Vector Y
            0,                                          # Yaw Vector X
            0,                                          # Yaw Vector Y
            0,                                          # Pixel Vector X
            0,                                          # Pixel Vector Y
            self.data["IMU_Velocity"][0],               # Pose R Velocity
            self.data["IMU_Velocity"][1],               # Pose P Velocity 
            self.data["IMU_Velocity"][2],               # Pose Y Velocity
        ])
        print "captured"

    def ros_callback(self, data):

        """""""""""""""""""""""""""""""""
        Data capture
        """""""""""""""""""""""""""""""""

        # Timing
        millis = int(round(time.time() * 1000))
        elapsed = millis - self.currTime        
        self.currTime = millis
        self.fpsRate = 1./((elapsed+0.000000001)/1000.)
        # print self.fpsRate

        # Get Data
        self.data = data
        try:
            sonarImage = self.data["SonarImage"].copy()
            cameraImage = self.data["CameraImage"].copy()
        except Exception as ex:
            print str(ex)
            return None, None, None, None

        """""""""""""""""""""""""""""""""
        Image Processing
        """""""""""""""""""""""""""""""""

        # Camera Processing
        cameraImage = Img.increase_contrast(img=cameraImage, mult=1.0)
        cameraImage = Img.undistort(cameraImage)

        # Power Law
        sonarImage = Img.power_law(img=sonarImage, power=2)

        # Remove water noise through running average
        self.cvAverage.add_image(sonarImage)
        sonarImage = self.cvAverage.process_image()

        # Remove bottom part from sonar
        sonarImage[460:480, 0:394] = 0

        # Make copy of images to write on
        sonarProcessed = sonarImage.copy()
        cameraProcessed = cameraImage.copy()
        cv2.putText(sonarProcessed, str(round(self.fpsRate,2)), (20,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0))
        cv2.putText(cameraProcessed, str(round(self.data["Vehicle_Position"][2]-2.2180,2)), (20,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0))
        cv2.putText(cameraProcessed, str(round(self.data["IMU_RPY"][1],2)), (20,60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0))

        # Convert to correct color types
        sonarImage = Img.color_to_gray(sonarImage)

        # Extract features from sonar
        sonarArray = self.featureExtractor.extract_features_sonar(sonarImage, sonarProcessed)

        # Extract features from camera
        cameraArray = self.featureExtractor.extract_features_camera(cameraImage, cameraProcessed)

        """""""""""""""""""""""""""""""""
        Saving of data
        """""""""""""""""""""""""""""""""

        if self.mode == 0:

            # Get buoy first
            buoyItem = Img.extract_buoy(cameraImage, cameraProcessed)
            if buoyItem is None:
                return None, None , sonarProcessed, cameraProcessed
            buoyROI = Img.square_roi_extract(image=cameraImage, mouse=buoyItem["Centroid"], maxDist=50)

            # Get sonar closest blob
            sonarItem = Math.closest_point(sonarArray, self.sonarImageMouse)
            if sonarItem is None:
                return None, None , sonarProcessed, cameraProcessed
            if sonarItem["Cost"] > 50:
                return None, None , sonarProcessed, cameraProcessed
            sonarROI = Img.square_roi_extract(image=sonarImage, mouse=sonarItem["Centroid"], maxDist=50)

            add_calibration_data(sonarItem, buoyItem)
            return buoyROI, sonarROI , sonarProcessed, cameraProcessed

        """""""""""""""""""""""""""""""""
        LK Tracking
        """""""""""""""""""""""""""""""""

        # LK
        output = self.lkSonar.process(sonarImage, sonarProcessed, 1/self.fpsRate, sonarArray)
        cameraImageForLK = Img.color_to_gray(cv2.GaussianBlur(cameraImage.copy(),(5,5),0))
        output = self.lkCamera.process(cameraImageForLK, cameraProcessed, 1/self.fpsRate, cameraArray)

        # Draw Valid Sonar Items
        for sonarItem in sonarArray:
            if sonarItem.has_key("PixelVector") and sonarItem["TrackValid"]:
                Img.draw_arrow_vector(sonarProcessed, sonarItem["Centroid"], sonarItem["PixelVector"], (0,0,255))
                cv2.drawContours(sonarProcessed,[sonarItem["Box"]],0,(0,255,0),1)
                sonarText = str(sonarItem["RTheta"]) + "\n" + str((round(sonarItem["MA"],3),round(sonarItem["ma"],3)))
                Img.draw_text(sonarProcessed, sonarText, sonarItem["Centroid"], 0.3, (255,255,255), 1, 10)

        """""""""""""""""""""""""""""""""
        Sonar Mapping
        """""""""""""""""""""""""""""""""               

        # Compute mapping
        self.mapper.sonarCameraMapping(cameraArray, sonarArray, cameraProcessed, sonarProcessed, self.data["IMU_RPY"], mapOnly=False)
        self.publish_mapping(sonarArray)

        # PF
        if self.mode == 2:
            self.pf.update_particles(vehicleVelocity=self.data["Vehicle_Velocity"], imuVelocity=self.data["IMU_Velocity"], fpsRate=self.fpsRate, mapper=self.mapper)
            self.pf.calc_log_likelihood(sonarImage, cameraImage, sonarProcessed, cameraProcessed, sonarArray, cameraArray, self.ros)
            self.pf.resample_particles()
            self.pf.draw_particles(cameraProcessed, sonarProcessed, self.ros)
            mean_solution = self.pf.compute_mean()
            #gmm_solutions = self.pf.compute_gmm()

            # Transform solutions?

            # self.ros.particle_cloud_publish(self.pf.State)
            # self.ros.gmm_cloud_publish(self.pf.Solutions)


        return None, None, sonarProcessed, cameraProcessed

    def dynamic_callback(self, config, level):
        rospy.loginfo("TrackingMode: " + str(config["TrackingMode"]))
        self.mode = config["TrackingMode"]

        if config["ParticleSetStart"]:
            rospy.loginfo("Resetting particles state")
            try:
                pfStartPos = map(float,config["ParticleStartingPos"].replace(" ","").split(','))
                pfStartVar = map(float,config["ParticleStartingVar"].replace(" ","").split(','))
                self.pf.set_position(position=pfStartPos, variance=pfStartVar)
            except Exception,e: 
                rospy.logerr("Invalid position input")
        if config["ParticleReset"]:
            self.pf.create_particles()

        self.pf.sonar_dimensions = (config["ParticleSonarMajor"],config["ParticleSonarMinor"])
        self.pf.sonar_tolerance = config["ParticleSonarTolerance"]
        self.pf.camera_ratio = config["ParticleCameraRatio"]
        self.pf.camera_tolerance = config["ParticleCameraTolerance"]
        self.pf.Camera_color = [float(config["ParticleCameraColor"])]
        self.pf.Randomize_percentage = config["ParticleRandomize"]
        self.pf.Weightage = config["ParticleWeightage"]
        self.featureExtractor.cameraStartRange = config["CameraStartRange"]
        self.featureExtractor.cameraEndRange = config["CameraEndRange"]
        self.featureExtractor.cameraStartRatio = config["CameraStartRatio"]
        self.featureExtractor.cameraEndRatio = config["CameraEndRatio"]
        return config
       
if __name__ == '__main__':


    rospy.init_node('Pipeline')
    pipeline = Pipeline()
    pipeline.start()

    



    rospy.spin()
