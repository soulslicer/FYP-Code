# Base libraries
import cv2
import time
import string
import json
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
import std_msgs.msg
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
from sonar.msg import *

# Others
from depth import *

class ROS():

    def __init__(self, ros_callback, ui_callback, mode):
        self.bridge = CvBridge()
        self.ros_callback = ros_callback
        self.ui_callback = ui_callback
        self.mode = mode

        self.sonarImage = None
        self.cameraImage = None
        self.sonarROI = np.zeros((100,100,3), np.uint8)
        self.cameraROI = np.zeros((100,100,3), np.uint8)
        self.dvlData = None
        self.depthData = None
        self.orientationData = None
        self.prevVehicleVector = (0,0)

    """""""""""""""""""""""""""""""""
    Register/Unregister nodes
    """""""""""""""""""""""""""""""""

    def start(self):
        self.register_pipeline()

    def stop(self):
        self.unregister_pipeline()

    def unregister_pipeline(self):
        rospy.loginfo("Unregister pipeline")
        self.sonarSub.unregister()
        self.cameraSub.unregister()
        self.depthSub.unregister()
        self.imuSub.unregister()
        self.dvlSub.unregister()
        self.uiSub.unregister()

        self.cameraPub.unregister()
        self.sonarPub.unregister()
        self.sonarROIPub.unregister() 
        self.cameraROIPub.unregister()
        rospy.sleep(rospy.Duration(0.05))

    def set_ahrs_hack(self):
        MyObject = type('MyObject', (object,), {})
        self.angularData = MyObject()
        self.angularData.x = 0
        self.angularData.y = 0
        self.angularData.z = 0

    def register_pipeline(self):
        if self.mode == "Pipeline":
            queue_size = 1
            self.sonarSub = rospy.Subscriber("/sonar_image/compressed", CompressedImage, self.sonar_callback, queue_size=queue_size)
            self.cameraSub = rospy.Subscriber("/frontcam/camera/image_color/compressed", CompressedImage, self.camera_callback, queue_size=queue_size)
            self.depthSub = rospy.Subscriber("/depth", depth, self.depth_callback, queue_size=queue_size)
            self.imuSub = rospy.Subscriber("/navigation/RPY", Vector3Stamped, self.imu_callback, queue_size=queue_size)
            self.uiSub = rospy.Subscriber("/ui", String, self.internal_ui_callback, queue_size=queue_size)

            self.set_ahrs_hack()
            self.ahrsSub = rospy.Subscriber("/AHRS8/data", Imu, self.ahrs_callback, queue_size=queue_size)
            self.dvlSub = rospy.Subscriber("/navigation/odom/relative", Odometry, self.dvl_callback, queue_size=queue_size)

            self.cameraPub = rospy.Publisher("/camera_processed2/compressed", CompressedImage, queue_size=10)
            self.sonarPub = rospy.Publisher("/sonar_processed2/compressed", CompressedImage, queue_size=10)
            self.debugPub = rospy.Publisher("/debugPub/compressed", CompressedImage, queue_size=10)
            self.sonarROIPub = rospy.Publisher("/sonar_roi/compressed", CompressedImage, queue_size=10)
            self.cameraROIPub = rospy.Publisher("/camera_roi/compressed", CompressedImage, queue_size=10)
            self.sonarObjectsPub = rospy.Publisher("/sonar_objects", sonar_objects, queue_size=10)

            self.particlePub = rospy.Publisher('/ParticleCloud', PointCloud, queue_size=10)
            self.gmmPub = rospy.Publisher('/GMMCloud', PointCloud, queue_size=10)

            rospy.sleep(rospy.Duration(0.05))
            rospy.loginfo("Register pipeline")

        elif self.mode == "UI":
            self.sonarSub = rospy.Subscriber("/camera_processed2/compressed", CompressedImage, self.sonar_callback, queue_size=2)
            self.cameraSub = rospy.Subscriber("/sonar_processed2/compressed", CompressedImage, self.camera_callback, queue_size=2)
            self.sonarROISub = rospy.Subscriber("/sonar_roi/compressed", CompressedImage, self.sonarroi_callback, queue_size=2)
            self.cameraROISub = rospy.Subscriber("/camera_roi/compressed", CompressedImage, self.cameraroi_callback, queue_size=2)
            self.depthSub = rospy.Subscriber("/depth", depth, self.depth_callback, queue_size=2)
            self.imuSub = rospy.Subscriber("/navigation/RPY", Vector3Stamped, self.imu_callback, queue_size=2)

            self.set_ahrs_hack()
            self.ahrsSub = rospy.Subscriber("/AHRS8/data", Imu, self.ahrs_callback, queue_size=2)
            self.dvlSub = rospy.Subscriber("/navigation/odom/relative", Odometry, self.dvl_callback, queue_size=2)

            self.uiPub = rospy.Publisher('/ui', String)

            rospy.sleep(rospy.Duration(0.05))
            rospy.loginfo("Register UI")            

    """""""""""""""""""""""""""""""""
    Callbacks
    """""""""""""""""""""""""""""""""

    def debug(self, img):
        self.debugPub.publish(self.cv2_to_compressed_ros(img))

    def get_data(self):
        syncErr = False

        # Data
        if self.sonarImage == None:
            rospy.logerr("No Data from Sonar")
            syncErr = True
        if self.cameraImage == None:
            rospy.logerr("No Data from Sonar")
            syncErr = True
        if self.dvlData == None:
            rospy.logerr("No Data from DVL")
            syncErr = True
        if self.orientationData == None:
            rospy.logerr("No Data from IMU")
            syncErr = True
        if self.depthData == None:
            rospy.logerr("No Data from Depth")
            syncErr = True
        if syncErr:
            return None

        # Timestamping
        sonarTS = self.sonarImage.header.stamp.secs
        cameraTS = self.cameraImage.header.stamp.secs
        orientationTS = self.orientationData.header.stamp.secs
        depthTS = 1
        dvlTS = 1
        if sonarTS == 0 or cameraTS == 0 or dvlTS == 0 or orientationTS == 0 or depthTS == 0:
            syncErr = True
        if syncErr:
            rospy.logerr("Timing error")
            return None      
        waitingTime = 1
        if cameraTS-sonarTS > waitingTime:
            rospy.logerr("Camera out of sync")
            syncErr = True
        if sonarTS-cameraTS > waitingTime:
            rospy.logerr("Sonar out of sync")
            syncErr = True
        if dvlTS-sonarTS > waitingTime:
            rospy.logerr("DVL out of sync")
            syncErr = True
        if orientationTS-sonarTS > waitingTime:
            rospy.logerr("IMU out of sync")
            syncErr = True
        if syncErr:
            return None

        # Check if Vehicle velocity data is new
        velocity_new = False
        self.vehicleVector = (self.dvlData.pose.pose.position.y, self.dvlData.pose.pose.position.x)
        subt = np.subtract(self.prevVehicleVector, self.vehicleVector)
        if not (subt[0] == 0 and subt[1] == 0):
            velocity_new = True
        self.prevVehicleVector = self.vehicleVector

        return {
            "SonarImage": self.compressed_ros_to_cv2(self.sonarImage),
            "CameraImage": self.compressed_ros_to_cv2(self.cameraImage),
            "SonarROI": self.compressed_ros_to_cv2(self.sonarROI),
            "CameraROI": self.compressed_ros_to_cv2(self.cameraROI),
            "Vehicle_Position": (
                round(self.dvlData.pose.pose.position.y, 3),
                self.depthData.depth,
                round(self.dvlData.pose.pose.position.x, 3),
                ),
            "Vehicle_Velocity": (
                round(self.dvlData.twist.twist.linear.y, 3),
                round(self.dvlData.twist.twist.linear.z, 3),
                round(self.dvlData.twist.twist.linear.x, 3),
                ),
            "IMU_Velocity": (
                round(self.angularData.x, 3),
                round(self.angularData.y, 3),
                round(self.angularData.z, 3)
                ),
            "IMU_RPY": (
                round(self.orientationData.vector.x, 2),
                round(self.orientationData.vector.y, 2),
                round(self.orientationData.vector.z, 2)
                ),
            "Velocity_New": velocity_new
        }

    def sonar_callback(self, rosImg):
        self.sonarImage = rosImg

        data = self.get_data()
        if self.mode == "Pipeline":
            cameraROI, sonarROI, sonarProcessed, cameraProcessed = self.ros_callback(data=data)
            if (sonarProcessed is not None) and (cameraProcessed is not None):
                try:
                    self.sonarPub.publish(self.cv2_to_compressed_ros(sonarProcessed))
                    self.cameraPub.publish(self.cv2_to_compressed_ros(cameraProcessed))
                    pass
                except Exception, e:
                    print str(e)
                    pass
            if (sonarROI is not None) and (cameraROI is not None):
                try:
                    self.sonarROIPub.publish(self.cv2_to_compressed_ros(sonarROI))
                    self.cameraROIPub.publish(self.cv2_to_compressed_ros(cameraROI))
                except Exception, e:
                    pass
        elif self.mode == "UI":
            self.ros_callback(data=data)

    def sonar_objects_publish(self, data):
        self.sonarObjectsPub.publish(data)

    def particle_cloud_publish(self, data):
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = "map"
        pcl = PointCloud()
        pcl.header = h
        pcl.points = []
        pcl.channels = []

        for i in range(0, data.shape[1]):
            point = Point32()
            point.x = data[0,i]
            point.y = data[1,i]
            point.z = data[2,i]
            pcl.points.append(point)

        self.particlePub.publish(pcl)

    def gmm_cloud_publish(self, data):
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = "map"
        pcl = PointCloud()
        pcl.header = h
        pcl.points = []
        pcl.channels = []

        for i in range(0, data.shape[1]):
            point = Point32()
            point.x = data[0,i]
            point.y = data[1,i]
            point.z = data[2,i]
            pcl.points.append(point)

        self.gmmPub.publish(pcl)

    def internal_ui_callback(self, ui_data):
        ui_data = ui_data.data
        ui_data = json.loads(ui_data)
        self.ui_callback(ui_data)

    def ahrs_callback(self, data):
        self.angularData = data.angular_velocity

    def cameraroi_callback(self, rosImg):
        self.cameraROI = rosImg

    def sonarroi_callback(self, rosImg):
        self.sonarROI = rosImg

    def camera_callback(self, rosImg):
        self.cameraImage = rosImg

    def dvl_callback(self, data):
        self.dvlData = data

    def depth_callback(self, data):
        self.depthData = data

    def imu_callback(self, data):
        self.orientationData = data

    def send_settings(self, settings):
        settings = json.dumps(settings, ensure_ascii=False)
        self.uiPub.publish(settings)

    """""""""""""""""""""""""""""""""
    Conversion
    """""""""""""""""""""""""""""""""

    def compressed_ros_to_cv2(self, img):
        try:
            np_arr = np.fromstring(img.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        except CvBridgeError as e:
            rospy.logerr(e)
        return frame 

    def image_ros_to_cv2(self, img):
        try:
            frame = self.bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        return frame

    def cv2_to_ros(self, img):
        try:
            frame = self.bridge.cv2_to_imgmsg(img, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        return frame
        
    def cv2_to_compressed_ros(self, img):
        try:
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()
            return msg
        except CvBridgeError as e:
            rospy.logerr(e)
