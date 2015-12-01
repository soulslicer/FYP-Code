# Base libraries
import cv2
import time
import string
import json
import numpy as np
import random
from sklearn import mixture
from scipy import linalg
import itertools

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
# from geometry_msgs.msg import *
import std_msgs.msg
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud

# Others
from depth import *
from sonar_mapping import *
from common import anorm2, draw_str

def cosd(angle):
    return np.cos(np.radians(angle))

def sind(angle):
    return np.sin(np.radians(angle))

def acosd(yx):
    return np.degrees(np.arccos(yx))

def asind(yx):
    return np.degrees(np.arcsin(yx))

def sqrt(x):
    return np.sqrt(x)

class Math():

    @staticmethod
    def compute_tangential(angularVel, R, Theta):
        tangetialVelocity = R * angularVel
        tangentialVector = (tangetialVelocity*cosd(Theta), -tangetialVelocity*sind(Theta))
        return tangentialVector

    @staticmethod
    def add_2d_vector(v1, v2):
        return (v1[0]+v2[0],v1[1]+v2[1])

    @staticmethod
    def l2_norm_2d(v1, v2):
        return np.sqrt( (v2[0]-v1[0])**2 + (v2[1]-v1[1])**2 )

    @staticmethod
    def distance_to_line_2d(line, p3):
        p1 = line[0]
        p2 = line[1]
        return abs( (p2[1]-p1[1])*p3[0] - (p2[0]-p1[0])*p3[1] + p2[0]*p1[1] - p2[1]*p1[0])/sqrt( (p2[1]-p1[1])**2 + (p2[0]-p1[0])**2 )

    @staticmethod
    def invert_vector(v):
        return (-v[0],-v[1],-v[2])

    @staticmethod
    def sort(sorttype):
        def compare(x,y):
            if x[sorttype] > y[sorttype]:
                return 1
            elif x[sorttype] < y[sorttype]:
                return -1
            else:
                return 0
        return compare

    @staticmethod
    def iterative_solve(arr):
        arr.sort(Math.sort("Cost"))
        if len(arr) > 0:
            return arr[0]
        else:
            return None

    @staticmethod
    def closest_point(arr, point):
        for item in arr:
            p1 = item["Centroid"]
            p2 = point
            item["Cost"] = Math.l2_norm_2d(p1,p2)
        return Math.iterative_solve(arr)

class Img():

    @staticmethod 
    def finlaynorm(img,cycle=2):
        b,g,r = cv2.split(img)
        b = np.float32(b)
        g = np.float32(g)
        r = np.float32(r)
        b = b/(b+g+r)*255
        g = g/(b+g+r)*255
        r = r/(b+g+r)*255
        bmean = np.mean(b)
        gmean = np.mean(g)
        rmean = np.mean(r)
        b = b/bmean
        g = g/gmean
        r = r/rmean
        b = cv2.normalize(b,0,255,cv2.NORM_MINMAX)*255
        g = cv2.normalize(g,0,255,cv2.NORM_MINMAX)*255
        r = cv2.normalize(r,0,255,cv2.NORM_MINMAX)*255
        b = b.clip(max=255)
        g = g.clip(max=255)
        r = r.clip(max=255)
        out = cv2.merge((np.uint8(b),np.uint8(g),np.uint8(r)))
        return out

    @staticmethod
    def draw_text(img, text, pos, size, color, thickness, spacing):
        y0 = pos[1]
        dy = spacing
        for i, line in enumerate(text.split('\n')):
            y = y0 + i*dy
            cv2.putText(img, line, (pos[0], y ), cv2.FONT_HERSHEY_SIMPLEX, size, color, thickness)

    @staticmethod
    def undistort(img):
        F = np.matrix([
            [735.4809,  0.,         388.9476],
            [0.,        733.6047,   292.0895],
            [0.,        0.,         1.0000  ]
        ])
        return cv2.undistort(img,F,np.array([0.0369, -0.3870, 0, 0, 0]))

    @staticmethod
    def square_roi_extract(image, mouse, maxDist):
        try:
            tl = {"X": mouse[0] - maxDist, "Y": mouse[1] - maxDist}
            tr = {"X": mouse[0] + maxDist, "Y": mouse[1] - maxDist}
            br = {"X": mouse[0] + maxDist, "Y": mouse[1] + maxDist}
            bl = {"X": mouse[0] - maxDist, "Y": mouse[1] + maxDist}
            return image[tr["Y"]:br["Y"], tl["X"]:tr["X"]]
        except:
            return None

    @staticmethod
    def increase_contrast(img, mult):
        def mult_channel(mult, channel):
            channel = channel*mult
            channel = np.clip(channel, 0, 255)
            channel = channel.astype(np.uint8)
            return channel
        b,g,r = cv2.split(img)
        b = mult_channel(mult, b)
        g = mult_channel(mult, g)
        r = mult_channel(mult, r)
        img = cv2.merge((b,g,r))
        return img

    @staticmethod
    def hsv_normalize(img):
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        h,s,v = cv2.split(hsv)
        v = cv2.equalizeHist(v)
        hsv = cv2.merge((h,s,v))
        img = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
        return img

    @staticmethod
    def power_law(img, power):
        img=img.astype('float32')
        img /=255.0
        img = cv2.GaussianBlur(img,(5,5),5)
        img = img**power
        img *= (255.0)
        img = img.astype('int32')
        return img

    @staticmethod
    def color_to_gray(img):
        return cv2.cvtColor(img, cv2.COLOR_RGBA2GRAY)

    @staticmethod
    def side_contour_points(cnt, rect):
        lb = rect[0]
        rb = rect[0] + rect[2] - 1
        lblist = []
        rblist = []
        for prevPoint,currPoint,nextPoint in Img.neighborhood(cnt):
            currPoint = (currPoint[0][0],currPoint[0][1])
            if currPoint[0] == lb:
                lblist.append(currPoint)
            if currPoint[0] == rb:
                rblist.append(currPoint)
        def compare(x,y):
            if x[1] < y[1]:
                return 1
            elif x[1] > y[1]:
                return -1
            else:
                return 0
        lblist.sort(compare)
        rblist.sort(compare)
        lbitem = lblist[0]
        rbitem = rblist[0]
        return lbitem, rbitem  

    @staticmethod
    def side_contour_points_modified(cnt, rect):
        lb = rect[0]
        rb = rect[0] + rect[2] - 1
        lblist = []
        rblist = []
        for prevPoint,currPoint,nextPoint in Img.neighborhood(cnt):
            currPoint = (currPoint[0],currPoint[1])
            if currPoint[0] == lb:
                lblist.append(currPoint)
            if currPoint[0] == rb:
                rblist.append(currPoint)
        def compare(x,y):
            if x[1] < y[1]:
                return 1
            elif x[1] > y[1]:
                return -1
            else:
                return 0
        lblist.sort(compare)
        rblist.sort(compare)
        lbitem = lblist[0]
        rbitem = rblist[0]
        return lbitem, rbitem  

    @staticmethod
    def neighborhood(iterable):
        iterator = iter(iterable)
        prev = None
        item = iterator.next()  # throws StopIteration if empty.
        for next in iterator:
            yield (prev,item,next)
            prev = item
            item = next
        yield (prev,item,None)

    @staticmethod
    def order_points(pts):
        rect = np.zeros((4, 2), dtype = "float32")
        s = pts.sum(axis = 1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        diff = np.diff(pts, axis = 1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        return rect

    @staticmethod
    def sonar_pixel(x,y):
        return sonar_pixel_map(x,y)

    @staticmethod
    def sonar_xym(xm,ym):
        return sonar_xym_map(xm,ym)    

    @staticmethod
    def distance(p1,p2):
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    @staticmethod
    def draw_arrow(image, p, angle, mag, color, arrow_magnitude=9, thickness=1, line_type=8, shift=0):
        q=[0,0]
        q[0] = p[0] + mag * np.cos(angle);
        q[1] = p[1] + mag * np.sin(angle);
        q=(int(q[0]),int(q[1]))
        cv2.line(image, p, q, color, thickness, line_type, shift)
        angle = np.arctan2(p[1]-q[1], p[0]-q[0])
        p = (int(q[0] + arrow_magnitude * np.cos(angle + np.pi/4)),
        int(q[1] + arrow_magnitude * np.sin(angle + np.pi/4)))
        cv2.line(image, p, q, color, thickness, line_type, shift)
        p = (int(q[0] + arrow_magnitude * np.cos(angle - np.pi/4)),
        int(q[1] + arrow_magnitude * np.sin(angle - np.pi/4)))
        cv2.line(image, p, q, color, thickness, line_type, shift)

    @staticmethod
    def draw_arrow_vector(image, p, vector, color, magnitude=1):
        dx = vector[0]*magnitude
        dy = vector[1]*magnitude
        rads = np.arctan2(-dy,dx)
        degs = np.degrees(rads)
        dist = np.sqrt( (dx)**2 + (dy)**2 )
        Img.draw_arrow(image, p, np.radians(degs+180), dist*100, color)

    @staticmethod
    def draw_rect(img, rect, color=(255,0,0)):
        cv2.rectangle(img,(rect[0],rect[1]),(rect[0]+rect[2],rect[1]+rect[3]),color,2)

    @staticmethod
    def draw_line(img, p1, p2, color):
        cv2.line(img, (p1[0], p1[1]), (p2[0], p2[1]), color, 1) 

class FeatureExtractor():
    def __init__(self):
        self.cameraStartRange = 0
        self.cameraEndRange = 85
        self.cameraStartRatio = 1.0
        self.cameraEndRatio = 2.0

    def extract_features_sonar(self, img, processed):
        # thresh = (np.max(img) - np.min(img)) / 2
        ret,thresholdedImage = cv2.threshold(img,100,255,cv2.THRESH_BINARY)
        ellipseStruct = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
        thresholdedImage = cv2.dilate(thresholdedImage,ellipseStruct,iterations = 10)
        thresholdedImage = cv2.erode(thresholdedImage,ellipseStruct,iterations = 6)
        ctr,heir = cv2.findContours(thresholdedImage,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

        # Feature extraction
        frameArray = []
        for cnt in ctr:

            # Get centroid
            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            centroid_x = int(M['m10']/M['m00'])
            centroid_y = int(M['m01']/M['m00'])
            cv2.circle(processed,(centroid_x,centroid_y), 2, (0,0,255), -1)
            cv2.drawContours(processed,[cnt],0,(0,0,255),1)

            # Area Perimeter
            area = cv2.contourArea(cnt)
            perimeter = cv2.arcLength(cnt,True)

            # Bounding Rect
            x,y,w,h = cv2.boundingRect(cnt)
            cx,cy = x+w/2, y+h/2   
            # cv2.putText(processed, str(area) + " " + str(angle), (cx, cy + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0))

            # Filtering
            if area > 5000:
                continue

            # Rect Fit
            rect = cv2.minAreaRect(cnt)
            box = cv2.cv.BoxPoints(rect)
            box = np.int0(box)
            boxArr = []
            try:
                for i in range(1,4):
                    boxRTheta1 = Img.sonar_pixel(box[0][0],box[0][1])
                    XY1 = (boxRTheta1[0] * np.sin(np.deg2rad(boxRTheta1[1])), boxRTheta1[0] * np.cos(np.deg2rad(boxRTheta1[1])))
                    boxRTheta2 = Img.sonar_pixel(box[i][0],box[i][1])
                    XY2 = (boxRTheta2[0] * np.sin(np.deg2rad(boxRTheta2[1])), boxRTheta2[0] * np.cos(np.deg2rad(boxRTheta2[1])))
                    boxArr.append({
                        "Cost": Math.l2_norm_2d(box[0], box[i]),
                        "PixelPoints": (box[0],box[i]) ,
                        "RealDistance":  Math.l2_norm_2d(XY1,XY2)
                    })
                boxArr.sort(Math.sort("Cost"))
                ma = boxArr[0]["RealDistance"]
                MA = boxArr[1]["RealDistance"]
                ratio = MA/ma
            except:
                ma = 0
                MA = 0
                ratio = 0
            
            # Ordered Points
            rect = cv2.minAreaRect(cnt)
            box = cv2.cv.BoxPoints(rect)
            box = np.int0(box)
            corners = [box][0]
            points = Img.order_points(corners)
            topLeft = (points[0][0], points[0][1])
            topRight = (points[1][0], points[1][1])
            bottomRight = (points[2][0], points[2][1])
            bottomLeft = (points[3][0], points[3][1])
            points = (topLeft, topRight, bottomRight, bottomLeft)

            # Compute side points
            lbitem, rbitem = Img.side_contour_points(cnt, [x,y,w,h])
            RThetaLeftSide = Img.sonar_pixel(lbitem[0],lbitem[1])
            RThetaRightSide = Img.sonar_pixel(rbitem[0],rbitem[1])

            # Compute RTheta and XY in metres for centroid
            RTheta = Img.sonar_pixel(centroid_x,centroid_y)
            XY = (RTheta[0] * np.sin(np.deg2rad(RTheta[1])), RTheta[0] * np.cos(np.deg2rad(RTheta[1])))
            # cv2.putText(processed, str(round(RTheta[0],2)), (cx, cy + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0))
            # cv2.putText(processed, str(round(RTheta[1],2)), (cx, cy + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0))

            # Assign data to dictionary
            item={
                # Params
                "Class": str(len(frameArray)),
                "Contour": cnt,
                "Centroid": (centroid_x, centroid_y),
                "Rect": [x,y,w,h],
                "Area": area,
                "MA": MA,
                "ma": ma,
                "Points": points,
                "Box": box,
                "Ratio": ratio,

                # Sonar mapped
                "RTheta": RTheta,
                "XY": XY,
                "RThetaLeftSide": RThetaLeftSide,
                "RThetaRightSide": RThetaRightSide,

                # Flags
                "PixelVector": None,
                "TrackValid": False,

                # Mapping
                "Mapping": None,
                "CameraItem": None,
                "SonarPosition": None,
                "VehiclePosition": None,

                "Cost": 1000000000000
            }
            frameArray.append(item)

        return frameArray

    def extract_features_camera(self, img, processed, multiple=True):
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        lower_color = np.array([self.cameraStartRange,40,40])
        upper_color = np.array([self.cameraEndRange,255,255])
        mask = cv2.inRange(hsv,lower_color, upper_color)
        mask = cv2.dilate(mask,None,iterations = 3)
        mask = cv2.erode(mask,None,iterations = 3)
        height, width, depth = img.shape        

        # Contours
        ctr,heir = cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        frameArray = []
        for cnt in ctr:

            cv2.drawContours(processed,[cnt],0,(0,0,255),1)

            # Check area
            area = cv2.contourArea(cnt)
            perimeter = cv2.arcLength(cnt,True)
            if area < 10 or area > 100000:
                continue

            # Rect Fit
            rect = cv2.minAreaRect(cnt)
            box = cv2.cv.BoxPoints(rect)
            box = np.int0(box)
            boxArr = []
            try:
                for i in range(1,4):
                    boxArr.append({
                        "Cost": Math.l2_norm_2d(box[0], box[i]),
                    })
                boxArr.sort(Math.sort("Cost"))
                ma = boxArr[0]["Cost"]
                MA = boxArr[1]["Cost"]
                ratio = MA/ma
            except:
                ma = 0
                MA = 0
                ratio = 0
            if ratio < self.cameraStartRatio or ratio > self.cameraEndRatio:
                continue

            # Bounding Rect
            x,y,w,h = cv2.boundingRect(cnt)
            cx,cy = x+w/2, y+h/2
            # cv2.putText(processed, str(ratio), (cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 50)

            # Centroid
            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            centroid_x = int(M['m10']/M['m00'])
            centroid_y = int(M['m01']/M['m00'])
            cv2.circle(processed,(centroid_x,centroid_y), 2, (0,0,255), -1)

            # Color
            roi = hsv[y+0:y+h-0, x+0:x+w-0]
            resized_roi = cv2.resize(roi, (0,0), fx=0.5, fy=0.5)
            hueVal,satVal,valVal = cv2.split(resized_roi)
            hueMean = np.mean(hueVal)

            # Cost
            cost = (hueMean - 30)**2 + (ratio-1)**2 + ((1/area)**0.5)*10000

            item={
                "Class": str(len(frameArray)),
                "Contour": cnt,
                "Centroid": (centroid_x, centroid_y),
                "Area": area,
                "Rect": [x,y,w,h],
                "TrackValid": False,
                "MA": MA,
                "ma": ma,
                "Ratio": ratio,
                "Cost": cost,
            }
            frameArray.append(item)

        # Sort cost
        for item in frameArray:
            Img.draw_rect(processed, item["Rect"], color=(0,0,255))
        item = Math.iterative_solve(frameArray)
        if item is not None:
            Img.draw_rect(processed, item["Rect"])
        if multiple:
            return frameArray
        else:
            return item

class LK():
    def __init__(self, sonar=False, points_wanted=5):
        self.sonar = sonar
        self.points_wanted = points_wanted
        self.track_len = 10
        self.detect_interval = 5
        self.tracks = []
        self.frame_idx = 0
        self.lk_params = dict( winSize  = (15, 15),
                          maxLevel = 2,
                          criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        self.feature_params = dict( maxCorners = 500,
                               qualityLevel = 0.3,
                               minDistance = 7,
                               blockSize = 7 )

        self.timeAccum = []
        for x in range(0, self.track_len):
            self.timeAccum.append(0)

    def process(self, img, processed, timePassed, frameArray):

        self.timeAccum.pop(0)
        self.timeAccum.append(timePassed)

        frame_gray = img
        vis = processed

        if len(self.tracks) > 0:
            img0, img1 = self.prev_gray, frame_gray
            p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 1, 2)
            p1, st, err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **self.lk_params)
            p0r, st, err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **self.lk_params)
            d = abs(p0-p0r).reshape(-1, 2).max(-1)
            good = d < 1
            new_tracks = []
            for tr, (x, y), good_flag in zip(self.tracks, p1.reshape(-1, 2), good):
                if not good_flag:
                    continue
                tr.append((x, y))
                if len(tr) > self.track_len:
                    del tr[0]
                new_tracks.append(tr)
                cv2.circle(vis, (x, y), 2, (0, 255, 0), -1)
            self.tracks = new_tracks
            cv2.polylines(vis, [np.int32(tr) for tr in self.tracks], False, (0, 255, 0))
            draw_str(vis, (20, 20), 'track count: %d' % len(self.tracks))

            # Assign all particle tracks to sonar objects if particle inside contour
            for item in frameArray:
                item["Track"] = []
                for tr in self.tracks:
                    latest_point = tr[-1]
                    if cv2.pointPolygonTest(item["Contour"], latest_point, False) > 0:
                        item["Track"].append(tr)
                        if len(tr) > 5:
                            item["TrackValid"] = True

            # Iterate through all sonar objects, calculate mean track velocity
            timeAccum = np.array(self.timeAccum)  
            for item in frameArray:
                velocityx_arr = []
                velocityy_arr = []
                for tr in item["Track"]:
                    if len(tr) >= self.points_wanted:
                        first_point = tr[-1]
                        last_point = tr[-1-(self.points_wanted-1)]
                        cv2.circle(vis, first_point, 1, (255, 120, 0), -1)
                        cv2.circle(vis, last_point, 1, (255, 120, 0), -1)
                        elapsed = sum(timeAccum[self.track_len-self.points_wanted:self.track_len])
                        
                        if self.sonar:
                            RTheta = sonar_pixel[int(first_point[0])][int(first_point[1])]
                            first_point = (RTheta[0] * sind(RTheta[1]), RTheta[0] * cosd(RTheta[1]))
                            RTheta = sonar_pixel[int(last_point[0])][int(last_point[1])]
                            last_point = (RTheta[0] * sind(RTheta[1]), RTheta[0] * cosd(RTheta[1]))
                            velocity = ((last_point[0]-first_point[0])/elapsed, (last_point[1]-first_point[1])/elapsed)
                        else:
                            velocity = ((last_point[0]-first_point[0]), (last_point[1]-first_point[1]))

                        velocityx_arr.append(velocity[0])
                        velocityy_arr.append(velocity[1])
                if len(velocityx_arr) > 0:
                    average_vel = (sum(velocityx_arr)/len(velocityx_arr), sum(velocityy_arr)/len(velocityy_arr))
                    item["PixelVector"] = average_vel

        if self.frame_idx % self.detect_interval == 0:
            mask = np.zeros_like(frame_gray)
            mask[:] = 255
            for x, y in [np.int32(tr[-1]) for tr in self.tracks]:
                cv2.circle(mask, (x, y), 5, 0, -1)
            p = cv2.goodFeaturesToTrack(frame_gray, mask = mask, **self.feature_params)
            if p is not None:
                for x, y in np.float32(p).reshape(-1, 2):
                    self.tracks.append([(x, y)])

        self.frame_idx += 1
        self.prev_gray = frame_gray

        return vis

class Mapper():

    def __init__(self):
        # Initial Params (TX TY TZ TRoll TPitch TYaw LY LZ)
        # For newer bags, camera housing got shifted yaw by +2 degrees 1.3287 original
        self.calib_params = [0.1394, 0.2500, 0.1413, -1.7862, 2.0516, 3.3287, 0.1460, 0.2514]
        self.TX = self.calib_params[0]
        self.TY = self.calib_params[1]
        self.TZ = self.calib_params[2]
        self.TRoll = self.calib_params[3]
        self.TPitch = self.calib_params[4]
        self.TYaw = self.calib_params[5]
        self.LY = self.calib_params[6]
        self.LZ = self.calib_params[7]

        self.CTS = self.getRT(Roll=self.TRoll, Pitch=self.TPitch, Yaw=self.TYaw, X=self.TX, Y=self.TY, Z=self.TZ)
        self.F = np.matrix([
            [735.4809,  0.,         388.9476,   0.],
            [0.,        733.6047,   292.0895,   0.],
            [0.,        0.,         1.0000,     0.]
        ])

    def getRT(self, Roll, Pitch, Yaw, X, Y, Z):
        RotationMatrix = np.matrix([
            [ cosd(Roll)*cosd(Yaw) + -sind(Pitch)*-sind(Roll)*-sind(Yaw), cosd(Pitch)*-sind(Roll), cosd(Roll)*sind(Yaw) + -sind(Pitch)*-sind(Roll)*cosd(Yaw), X],
            [ sind(Roll)*cosd(Yaw) + -sind(Pitch)*cosd(Roll)*-sind(Yaw), cosd(Pitch)*cosd(Roll), sind(Roll)*sind(Yaw) + -sind(Pitch)*cosd(Roll)*cosd(Yaw), Y],
            [               cosd(Pitch)*-sind(Yaw),     sind(Pitch),               cosd(Pitch)*cosd(Yaw), Z],
            [0., 0., 0., 1.]
        ])
        return RotationMatrix

    def yVehicleThroughYSonar(self, RSonar, YSonar, Pitch, LY, LZ):
        return (LY*cosd(Pitch)+LZ*sind(Pitch)+RSonar*sind(Pitch+asind(YSonar/RSonar)))

    def ySonarThroughYVehicle(self, RSonar, YVehicle, Pitch, LY, LZ):
        return (-RSonar*sind(Pitch+asind((LY*cosd(Pitch)-YVehicle+LZ*sind(Pitch))/RSonar)))

    def pixelMap(self, RSonar, ThetaSonar, YSonar):
        SpaceMatrix = np.matrix([
            [(sqrt(RSonar**2 - YSonar**2)*sind(ThetaSonar))],
            [(YSonar)],
            [(sqrt(RSonar**2 - YSonar**2)*cosd(ThetaSonar))],
            [1]
        ])
        FinalMatrix = self.F*self.CTS*SpaceMatrix
        UVMatrix = np.matrix([
            [FinalMatrix.item(0)/FinalMatrix.item(2)],
            [FinalMatrix.item(1)/FinalMatrix.item(2)],
        ])
        return (int(UVMatrix[0]),int(UVMatrix[1]))   

    def lineMap(self, RSonar, ThetaSonar):
        YSonarBot = sqrt((RSonar)**2 - (RSonar*cosd(20))**2)
        YSonarTop = sqrt((RSonar)**2 - (RSonar*cosd(20))**2)*-1
        MidPixel = self.pixelMap(RSonar=RSonar, ThetaSonar=ThetaSonar, YSonar=0)
        TopPixel = self.pixelMap(RSonar=RSonar, ThetaSonar=ThetaSonar, YSonar=YSonarTop)
        BotPixel = self.pixelMap(RSonar=RSonar, ThetaSonar=ThetaSonar, YSonar=YSonarBot)
        return TopPixel, MidPixel, BotPixel

    def squareMap(self, sonarItem = None, cameraProcessed = None):
        if sonarItem is None:
            return None

        RSonarLeft = sonarItem["RThetaLeftSide"][0]
        ThetaSonarLeft = sonarItem["RThetaLeftSide"][1]
        RSonarRight = sonarItem["RThetaRightSide"][0]
        ThetaSonarRight = sonarItem["RThetaRightSide"][1]
        RSonarCentre = sonarItem["RTheta"][0]
        ThetaSonarCentre = sonarItem["RTheta"][1]

        TopCentrePixel, MidCentrePixel, BotCentrePixel = self.lineMap(RSonar=RSonarCentre, ThetaSonar=ThetaSonarCentre)
        TopLeftPixel, MidLeftPixel, BotLeftPixel = self.lineMap(RSonar=RSonarLeft, ThetaSonar=ThetaSonarLeft)
        TopRightPixel, MidRightPixel, BotRightPixel = self.lineMap(RSonar=RSonarRight, ThetaSonar=ThetaSonarRight)
        fpoint = np.array([
            [int(TopLeftPixel[0]) ,int(TopLeftPixel[1])],
            [int(TopRightPixel[0])  ,int(TopRightPixel[1])],
            [int(BotRightPixel[0])  ,int(BotRightPixel[1])],
            [int(BotLeftPixel[0])  ,int(BotLeftPixel[1])]
        ])
        cv2.drawContours(cameraProcessed,[fpoint],0,(0,0,255),2)
        Img.draw_line(cameraProcessed, TopCentrePixel, BotCentrePixel, (0,255,0))
        return {
            "LeftLine": (TopLeftPixel, BotLeftPixel),
            "CentreLine": (TopCentrePixel, BotCentrePixel),
            "RightLine": (TopRightPixel, BotRightPixel),
            "FPoint": fpoint
        }

    def compute3DPoint(self, RSonar, ThetaSonar, VCamera, Pitch):
        CTS = self.CTS
        YSonar = -(1.0*(CTS[1,1]*CTS[1,3]*733.6047**2+CTS[2,1]*CTS[2,3]*VCamera**2+CTS[2,1]*CTS[2,3]*292.0895**2-1.0*CTS[1,1]*CTS[2,3]*733.6047*VCamera-1.0*CTS[1,3]*CTS[2,1]*733.6047*VCamera+CTS[1,1]*CTS[2,3]*733.6047*292.0895+CTS[1,3]*CTS[2,1]*733.6047*292.0895-2.0*CTS[2,1]*CTS[2,3]*VCamera*292.0895+CTS[1,2]*733.6047*cosd(ThetaSonar)*(CTS[1,1]**2*733.6047**2*RSonar**2-1.0*CTS[2,3]**2*VCamera**2-1.0*CTS[2,3]**2*292.0895**2-1.0*CTS[1,3]**2*733.6047**2+CTS[2,1]**2*RSonar**2*VCamera**2+CTS[2,1]**2*RSonar**2*292.0895**2+2.0*CTS[2,3]**2*VCamera*292.0895-2.0*CTS[2,1]**2*RSonar**2*VCamera*292.0895+CTS[1,2]**2*733.6047**2*RSonar**2*cosd(ThetaSonar)**2+CTS[2,2]**2*RSonar**2*VCamera**2*cosd(ThetaSonar)**2+CTS[2,2]**2*RSonar**2*292.0895**2*cosd(ThetaSonar)**2+CTS[1,0]**2*733.6047**2*RSonar**2*sind(ThetaSonar)**2+2.0*CTS[1,3]*CTS[2,3]*733.6047*VCamera-2.0*CTS[1,3]*CTS[2,3]*733.6047*292.0895+CTS[2,0]**2*RSonar**2*VCamera**2*sind(ThetaSonar)**2+CTS[2,0]**2*RSonar**2*292.0895**2*sind(ThetaSonar)**2-2.0*CTS[1,1]*CTS[2,1]*733.6047*RSonar**2*VCamera+2.0*CTS[1,1]*CTS[2,1]*733.6047*RSonar**2*292.0895-2.0*CTS[2,2]**2*RSonar**2*VCamera*292.0895*cosd(ThetaSonar)**2-2.0*CTS[2,0]**2*RSonar**2*VCamera*292.0895*sind(ThetaSonar)**2-2.0*CTS[1,2]*CTS[2,2]*733.6047*RSonar**2*VCamera*cosd(ThetaSonar)**2+2.0*CTS[1,2]*CTS[2,2]*733.6047*RSonar**2*292.0895*cosd(ThetaSonar)**2-2.0*CTS[1,0]*CTS[2,0]*733.6047*RSonar**2*VCamera*sind(ThetaSonar)**2+2.0*CTS[1,0]*CTS[2,0]*733.6047*RSonar**2*292.0895*sind(ThetaSonar)**2+2.0*CTS[1,0]*CTS[1,2]*733.6047**2*RSonar**2*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[2,0]*CTS[2,2]*RSonar**2*VCamera**2*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[2,0]*CTS[2,2]*RSonar**2*292.0895**2*cosd(ThetaSonar)*sind(ThetaSonar)-2.0*CTS[1,0]*CTS[2,2]*733.6047*RSonar**2*VCamera*cosd(ThetaSonar)*sind(ThetaSonar)-2.0*CTS[1,2]*CTS[2,0]*733.6047*RSonar**2*VCamera*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[1,0]*CTS[2,2]*733.6047*RSonar**2*292.0895*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[1,2]*CTS[2,0]*733.6047*RSonar**2*292.0895*cosd(ThetaSonar)*sind(ThetaSonar)-4.0*CTS[2,0]*CTS[2,2]*RSonar**2*VCamera*292.0895*cosd(ThetaSonar)*sind(ThetaSonar))**(0.5)-1.0*CTS[2,2]*VCamera*cosd(ThetaSonar)*(CTS[1,1]**2*733.6047**2*RSonar**2-1.0*CTS[2,3]**2*VCamera**2-1.0*CTS[2,3]**2*292.0895**2-1.0*CTS[1,3]**2*733.6047**2+CTS[2,1]**2*RSonar**2*VCamera**2+CTS[2,1]**2*RSonar**2*292.0895**2+2.0*CTS[2,3]**2*VCamera*292.0895-2.0*CTS[2,1]**2*RSonar**2*VCamera*292.0895+CTS[1,2]**2*733.6047**2*RSonar**2*cosd(ThetaSonar)**2+CTS[2,2]**2*RSonar**2*VCamera**2*cosd(ThetaSonar)**2+CTS[2,2]**2*RSonar**2*292.0895**2*cosd(ThetaSonar)**2+CTS[1,0]**2*733.6047**2*RSonar**2*sind(ThetaSonar)**2+2.0*CTS[1,3]*CTS[2,3]*733.6047*VCamera-2.0*CTS[1,3]*CTS[2,3]*733.6047*292.0895+CTS[2,0]**2*RSonar**2*VCamera**2*sind(ThetaSonar)**2+CTS[2,0]**2*RSonar**2*292.0895**2*sind(ThetaSonar)**2-2.0*CTS[1,1]*CTS[2,1]*733.6047*RSonar**2*VCamera+2.0*CTS[1,1]*CTS[2,1]*733.6047*RSonar**2*292.0895-2.0*CTS[2,2]**2*RSonar**2*VCamera*292.0895*cosd(ThetaSonar)**2-2.0*CTS[2,0]**2*RSonar**2*VCamera*292.0895*sind(ThetaSonar)**2-2.0*CTS[1,2]*CTS[2,2]*733.6047*RSonar**2*VCamera*cosd(ThetaSonar)**2+2.0*CTS[1,2]*CTS[2,2]*733.6047*RSonar**2*292.0895*cosd(ThetaSonar)**2-2.0*CTS[1,0]*CTS[2,0]*733.6047*RSonar**2*VCamera*sind(ThetaSonar)**2+2.0*CTS[1,0]*CTS[2,0]*733.6047*RSonar**2*292.0895*sind(ThetaSonar)**2+2.0*CTS[1,0]*CTS[1,2]*733.6047**2*RSonar**2*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[2,0]*CTS[2,2]*RSonar**2*VCamera**2*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[2,0]*CTS[2,2]*RSonar**2*292.0895**2*cosd(ThetaSonar)*sind(ThetaSonar)-2.0*CTS[1,0]*CTS[2,2]*733.6047*RSonar**2*VCamera*cosd(ThetaSonar)*sind(ThetaSonar)-2.0*CTS[1,2]*CTS[2,0]*733.6047*RSonar**2*VCamera*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[1,0]*CTS[2,2]*733.6047*RSonar**2*292.0895*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[1,2]*CTS[2,0]*733.6047*RSonar**2*292.0895*cosd(ThetaSonar)*sind(ThetaSonar)-4.0*CTS[2,0]*CTS[2,2]*RSonar**2*VCamera*292.0895*cosd(ThetaSonar)*sind(ThetaSonar))**(0.5)+CTS[2,2]*292.0895*cosd(ThetaSonar)*(CTS[1,1]**2*733.6047**2*RSonar**2-1.0*CTS[2,3]**2*VCamera**2-1.0*CTS[2,3]**2*292.0895**2-1.0*CTS[1,3]**2*733.6047**2+CTS[2,1]**2*RSonar**2*VCamera**2+CTS[2,1]**2*RSonar**2*292.0895**2+2.0*CTS[2,3]**2*VCamera*292.0895-2.0*CTS[2,1]**2*RSonar**2*VCamera*292.0895+CTS[1,2]**2*733.6047**2*RSonar**2*cosd(ThetaSonar)**2+CTS[2,2]**2*RSonar**2*VCamera**2*cosd(ThetaSonar)**2+CTS[2,2]**2*RSonar**2*292.0895**2*cosd(ThetaSonar)**2+CTS[1,0]**2*733.6047**2*RSonar**2*sind(ThetaSonar)**2+2.0*CTS[1,3]*CTS[2,3]*733.6047*VCamera-2.0*CTS[1,3]*CTS[2,3]*733.6047*292.0895+CTS[2,0]**2*RSonar**2*VCamera**2*sind(ThetaSonar)**2+CTS[2,0]**2*RSonar**2*292.0895**2*sind(ThetaSonar)**2-2.0*CTS[1,1]*CTS[2,1]*733.6047*RSonar**2*VCamera+2.0*CTS[1,1]*CTS[2,1]*733.6047*RSonar**2*292.0895-2.0*CTS[2,2]**2*RSonar**2*VCamera*292.0895*cosd(ThetaSonar)**2-2.0*CTS[2,0]**2*RSonar**2*VCamera*292.0895*sind(ThetaSonar)**2-2.0*CTS[1,2]*CTS[2,2]*733.6047*RSonar**2*VCamera*cosd(ThetaSonar)**2+2.0*CTS[1,2]*CTS[2,2]*733.6047*RSonar**2*292.0895*cosd(ThetaSonar)**2-2.0*CTS[1,0]*CTS[2,0]*733.6047*RSonar**2*VCamera*sind(ThetaSonar)**2+2.0*CTS[1,0]*CTS[2,0]*733.6047*RSonar**2*292.0895*sind(ThetaSonar)**2+2.0*CTS[1,0]*CTS[1,2]*733.6047**2*RSonar**2*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[2,0]*CTS[2,2]*RSonar**2*VCamera**2*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[2,0]*CTS[2,2]*RSonar**2*292.0895**2*cosd(ThetaSonar)*sind(ThetaSonar)-2.0*CTS[1,0]*CTS[2,2]*733.6047*RSonar**2*VCamera*cosd(ThetaSonar)*sind(ThetaSonar)-2.0*CTS[1,2]*CTS[2,0]*733.6047*RSonar**2*VCamera*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[1,0]*CTS[2,2]*733.6047*RSonar**2*292.0895*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[1,2]*CTS[2,0]*733.6047*RSonar**2*292.0895*cosd(ThetaSonar)*sind(ThetaSonar)-4.0*CTS[2,0]*CTS[2,2]*RSonar**2*VCamera*292.0895*cosd(ThetaSonar)*sind(ThetaSonar))**(0.5)+CTS[1,0]*733.6047*sind(ThetaSonar)*(CTS[1,1]**2*733.6047**2*RSonar**2-1.0*CTS[2,3]**2*VCamera**2-1.0*CTS[2,3]**2*292.0895**2-1.0*CTS[1,3]**2*733.6047**2+CTS[2,1]**2*RSonar**2*VCamera**2+CTS[2,1]**2*RSonar**2*292.0895**2+2.0*CTS[2,3]**2*VCamera*292.0895-2.0*CTS[2,1]**2*RSonar**2*VCamera*292.0895+CTS[1,2]**2*733.6047**2*RSonar**2*cosd(ThetaSonar)**2+CTS[2,2]**2*RSonar**2*VCamera**2*cosd(ThetaSonar)**2+CTS[2,2]**2*RSonar**2*292.0895**2*cosd(ThetaSonar)**2+CTS[1,0]**2*733.6047**2*RSonar**2*sind(ThetaSonar)**2+2.0*CTS[1,3]*CTS[2,3]*733.6047*VCamera-2.0*CTS[1,3]*CTS[2,3]*733.6047*292.0895+CTS[2,0]**2*RSonar**2*VCamera**2*sind(ThetaSonar)**2+CTS[2,0]**2*RSonar**2*292.0895**2*sind(ThetaSonar)**2-2.0*CTS[1,1]*CTS[2,1]*733.6047*RSonar**2*VCamera+2.0*CTS[1,1]*CTS[2,1]*733.6047*RSonar**2*292.0895-2.0*CTS[2,2]**2*RSonar**2*VCamera*292.0895*cosd(ThetaSonar)**2-2.0*CTS[2,0]**2*RSonar**2*VCamera*292.0895*sind(ThetaSonar)**2-2.0*CTS[1,2]*CTS[2,2]*733.6047*RSonar**2*VCamera*cosd(ThetaSonar)**2+2.0*CTS[1,2]*CTS[2,2]*733.6047*RSonar**2*292.0895*cosd(ThetaSonar)**2-2.0*CTS[1,0]*CTS[2,0]*733.6047*RSonar**2*VCamera*sind(ThetaSonar)**2+2.0*CTS[1,0]*CTS[2,0]*733.6047*RSonar**2*292.0895*sind(ThetaSonar)**2+2.0*CTS[1,0]*CTS[1,2]*733.6047**2*RSonar**2*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[2,0]*CTS[2,2]*RSonar**2*VCamera**2*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[2,0]*CTS[2,2]*RSonar**2*292.0895**2*cosd(ThetaSonar)*sind(ThetaSonar)-2.0*CTS[1,0]*CTS[2,2]*733.6047*RSonar**2*VCamera*cosd(ThetaSonar)*sind(ThetaSonar)-2.0*CTS[1,2]*CTS[2,0]*733.6047*RSonar**2*VCamera*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[1,0]*CTS[2,2]*733.6047*RSonar**2*292.0895*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[1,2]*CTS[2,0]*733.6047*RSonar**2*292.0895*cosd(ThetaSonar)*sind(ThetaSonar)-4.0*CTS[2,0]*CTS[2,2]*RSonar**2*VCamera*292.0895*cosd(ThetaSonar)*sind(ThetaSonar))**(0.5)-1.0*CTS[2,0]*VCamera*sind(ThetaSonar)*(CTS[1,1]**2*733.6047**2*RSonar**2-1.0*CTS[2,3]**2*VCamera**2-1.0*CTS[2,3]**2*292.0895**2-1.0*CTS[1,3]**2*733.6047**2+CTS[2,1]**2*RSonar**2*VCamera**2+CTS[2,1]**2*RSonar**2*292.0895**2+2.0*CTS[2,3]**2*VCamera*292.0895-2.0*CTS[2,1]**2*RSonar**2*VCamera*292.0895+CTS[1,2]**2*733.6047**2*RSonar**2*cosd(ThetaSonar)**2+CTS[2,2]**2*RSonar**2*VCamera**2*cosd(ThetaSonar)**2+CTS[2,2]**2*RSonar**2*292.0895**2*cosd(ThetaSonar)**2+CTS[1,0]**2*733.6047**2*RSonar**2*sind(ThetaSonar)**2+2.0*CTS[1,3]*CTS[2,3]*733.6047*VCamera-2.0*CTS[1,3]*CTS[2,3]*733.6047*292.0895+CTS[2,0]**2*RSonar**2*VCamera**2*sind(ThetaSonar)**2+CTS[2,0]**2*RSonar**2*292.0895**2*sind(ThetaSonar)**2-2.0*CTS[1,1]*CTS[2,1]*733.6047*RSonar**2*VCamera+2.0*CTS[1,1]*CTS[2,1]*733.6047*RSonar**2*292.0895-2.0*CTS[2,2]**2*RSonar**2*VCamera*292.0895*cosd(ThetaSonar)**2-2.0*CTS[2,0]**2*RSonar**2*VCamera*292.0895*sind(ThetaSonar)**2-2.0*CTS[1,2]*CTS[2,2]*733.6047*RSonar**2*VCamera*cosd(ThetaSonar)**2+2.0*CTS[1,2]*CTS[2,2]*733.6047*RSonar**2*292.0895*cosd(ThetaSonar)**2-2.0*CTS[1,0]*CTS[2,0]*733.6047*RSonar**2*VCamera*sind(ThetaSonar)**2+2.0*CTS[1,0]*CTS[2,0]*733.6047*RSonar**2*292.0895*sind(ThetaSonar)**2+2.0*CTS[1,0]*CTS[1,2]*733.6047**2*RSonar**2*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[2,0]*CTS[2,2]*RSonar**2*VCamera**2*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[2,0]*CTS[2,2]*RSonar**2*292.0895**2*cosd(ThetaSonar)*sind(ThetaSonar)-2.0*CTS[1,0]*CTS[2,2]*733.6047*RSonar**2*VCamera*cosd(ThetaSonar)*sind(ThetaSonar)-2.0*CTS[1,2]*CTS[2,0]*733.6047*RSonar**2*VCamera*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[1,0]*CTS[2,2]*733.6047*RSonar**2*292.0895*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[1,2]*CTS[2,0]*733.6047*RSonar**2*292.0895*cosd(ThetaSonar)*sind(ThetaSonar)-4.0*CTS[2,0]*CTS[2,2]*RSonar**2*VCamera*292.0895*cosd(ThetaSonar)*sind(ThetaSonar))**(0.5)+CTS[2,0]*292.0895*sind(ThetaSonar)*(CTS[1,1]**2*733.6047**2*RSonar**2-1.0*CTS[2,3]**2*VCamera**2-1.0*CTS[2,3]**2*292.0895**2-1.0*CTS[1,3]**2*733.6047**2+CTS[2,1]**2*RSonar**2*VCamera**2+CTS[2,1]**2*RSonar**2*292.0895**2+2.0*CTS[2,3]**2*VCamera*292.0895-2.0*CTS[2,1]**2*RSonar**2*VCamera*292.0895+CTS[1,2]**2*733.6047**2*RSonar**2*cosd(ThetaSonar)**2+CTS[2,2]**2*RSonar**2*VCamera**2*cosd(ThetaSonar)**2+CTS[2,2]**2*RSonar**2*292.0895**2*cosd(ThetaSonar)**2+CTS[1,0]**2*733.6047**2*RSonar**2*sind(ThetaSonar)**2+2.0*CTS[1,3]*CTS[2,3]*733.6047*VCamera-2.0*CTS[1,3]*CTS[2,3]*733.6047*292.0895+CTS[2,0]**2*RSonar**2*VCamera**2*sind(ThetaSonar)**2+CTS[2,0]**2*RSonar**2*292.0895**2*sind(ThetaSonar)**2-2.0*CTS[1,1]*CTS[2,1]*733.6047*RSonar**2*VCamera+2.0*CTS[1,1]*CTS[2,1]*733.6047*RSonar**2*292.0895-2.0*CTS[2,2]**2*RSonar**2*VCamera*292.0895*cosd(ThetaSonar)**2-2.0*CTS[2,0]**2*RSonar**2*VCamera*292.0895*sind(ThetaSonar)**2-2.0*CTS[1,2]*CTS[2,2]*733.6047*RSonar**2*VCamera*cosd(ThetaSonar)**2+2.0*CTS[1,2]*CTS[2,2]*733.6047*RSonar**2*292.0895*cosd(ThetaSonar)**2-2.0*CTS[1,0]*CTS[2,0]*733.6047*RSonar**2*VCamera*sind(ThetaSonar)**2+2.0*CTS[1,0]*CTS[2,0]*733.6047*RSonar**2*292.0895*sind(ThetaSonar)**2+2.0*CTS[1,0]*CTS[1,2]*733.6047**2*RSonar**2*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[2,0]*CTS[2,2]*RSonar**2*VCamera**2*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[2,0]*CTS[2,2]*RSonar**2*292.0895**2*cosd(ThetaSonar)*sind(ThetaSonar)-2.0*CTS[1,0]*CTS[2,2]*733.6047*RSonar**2*VCamera*cosd(ThetaSonar)*sind(ThetaSonar)-2.0*CTS[1,2]*CTS[2,0]*733.6047*RSonar**2*VCamera*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[1,0]*CTS[2,2]*733.6047*RSonar**2*292.0895*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[1,2]*CTS[2,0]*733.6047*RSonar**2*292.0895*cosd(ThetaSonar)*sind(ThetaSonar)-4.0*CTS[2,0]*CTS[2,2]*RSonar**2*VCamera*292.0895*cosd(ThetaSonar)*sind(ThetaSonar))**(0.5)))/(CTS[1,1]**2*733.6047**2+CTS[2,1]**2*VCamera**2+CTS[2,1]**2*292.0895**2-2.0*CTS[2,1]**2*VCamera*292.0895+CTS[1,2]**2*733.6047**2*cosd(ThetaSonar)**2+CTS[2,2]**2*VCamera**2*cosd(ThetaSonar)**2+CTS[2,2]**2*292.0895**2*cosd(ThetaSonar)**2+CTS[1,0]**2*733.6047**2*sind(ThetaSonar)**2+CTS[2,0]**2*VCamera**2*sind(ThetaSonar)**2+CTS[2,0]**2*292.0895**2*sind(ThetaSonar)**2-2.0*CTS[1,1]*CTS[2,1]*733.6047*VCamera+2.0*CTS[1,1]*CTS[2,1]*733.6047*292.0895-2.0*CTS[2,2]**2*VCamera*292.0895*cosd(ThetaSonar)**2-2.0*CTS[2,0]**2*VCamera*292.0895*sind(ThetaSonar)**2+2.0*CTS[1,0]*CTS[1,2]*733.6047**2*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[2,0]*CTS[2,2]*VCamera**2*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[2,0]*CTS[2,2]*292.0895**2*cosd(ThetaSonar)*sind(ThetaSonar)-2.0*CTS[1,2]*CTS[2,2]*733.6047*VCamera*cosd(ThetaSonar)**2+2.0*CTS[1,2]*CTS[2,2]*733.6047*292.0895*cosd(ThetaSonar)**2-2.0*CTS[1,0]*CTS[2,0]*733.6047*VCamera*sind(ThetaSonar)**2+2.0*CTS[1,0]*CTS[2,0]*733.6047*292.0895*sind(ThetaSonar)**2-2.0*CTS[1,0]*CTS[2,2]*733.6047*VCamera*cosd(ThetaSonar)*sind(ThetaSonar)-2.0*CTS[1,2]*CTS[2,0]*733.6047*VCamera*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[1,0]*CTS[2,2]*733.6047*292.0895*cosd(ThetaSonar)*sind(ThetaSonar)+2.0*CTS[1,2]*CTS[2,0]*733.6047*292.0895*cosd(ThetaSonar)*sind(ThetaSonar)-4.0*CTS[2,0]*CTS[2,2]*VCamera*292.0895*cosd(ThetaSonar)*sind(ThetaSonar))
        SonarSpaceMatrix = np.matrix([
            [(sqrt(RSonar**2 - YSonar**2)*sind(ThetaSonar))],
            [(YSonar)],
            [(sqrt(RSonar**2 - YSonar**2)*cosd(ThetaSonar))],
            [1]
        ])
        VehicleSpaceMatrix = self.getRT(0,Pitch,0,0,0,0)*self.getRT(0,0,0,0,self.calib_params[6],self.calib_params[7])*SonarSpaceMatrix;
        return VehicleSpaceMatrix, SonarSpaceMatrix

    def sonarCameraMapping(self, cameraArray, sonarArray, cameraProcessed, sonarProcessed, RPY, mapOnly=False):
        
        # Pre-Compute sonar mappings
        for sonarItem in sonarArray:
            if sonarItem["TrackValid"]:
                sonarItem["Mapping"] = self.squareMap(sonarItem, cameraProcessed)

        # Only do mapping
        if mapOnly:
            return

        # Iterate camera objects for mapping
        for cameraItem in cameraArray:
            if cameraItem["TrackValid"]:

                # Get camera points
                lpointCam = (cameraItem["Rect"][0], cameraItem["Rect"][1]+int(cameraItem["Rect"][3]/2))
                cpointCam = cameraItem["Centroid"]
                rpointCam = (cameraItem["Rect"][0]+int(cameraItem["Rect"][2]), cameraItem["Rect"][1]+int(cameraItem["Rect"][3]/2))

                # Iterate sonar objects for mapping
                for sonarItem in sonarArray:
                    if sonarItem["TrackValid"]:
                        squareMap = sonarItem["Mapping"]
                        llineSon = squareMap["LeftLine"]
                        clineSon = squareMap["CentreLine"]
                        rlineSon = squareMap["RightLine"]
                        sonarItem["Cost"] = \
                        Math.distance_to_line_2d(line=llineSon, p3=lpointCam) + \
                        Math.distance_to_line_2d(line=clineSon, p3=cpointCam) + \
                        Math.distance_to_line_2d(line=rlineSon, p3=rpointCam)

                # Choose best match based on cost function
                matchingSonarItem = Math.iterative_solve(sonarArray)
                if matchingSonarItem is not None and matchingSonarItem["Cost"] < 100:
                    vehiclePosition, sonarPosition = self.compute3DPoint(RSonar=matchingSonarItem["RTheta"][0], ThetaSonar=matchingSonarItem["RTheta"][1], VCamera=cameraItem["Centroid"][1], Pitch=RPY[1])
                    matchingSonarItem["CameraItem"] = cameraItem
                    matchingSonarItem["VehiclePosition"] = vehiclePosition
                    matchingSonarItem["SonarPosition"] = sonarPosition
                    posString = "X: " + str(round(vehiclePosition[0],2)) + "\n" + "Y: " + str(round(vehiclePosition[1],2)) + "\n" + "Z: " + str(round(vehiclePosition[2],2))
                    Img.draw_text(cameraProcessed, posString, cameraItem["Centroid"], 0.5, (255,255,0), 2, 20)

class ParticleFilter():
    def __init__(self, Sonar_resolution=(394,480), Camera_resolution=(780,580), Npop_particles=2000):

        # Set Resolutions and particle counts
        self.Sonar_resolution = Sonar_resolution
        self.Camera_resolution = Camera_resolution
        self.Npop_particles = Npop_particles

        # Set Noise levels and color levels (User must set these)
        self.Xstd_sonar_color = 25
        self.Xstd_sonar_size = 0.3
        self.Xstd_camera_color = 15
        self.Xstd_camera_ratio = 0.3
        self.Xstd_sonar_pos = 0.05
        self.Xstd_sonar_vec = 0.05
        self.Xstd_sonar_depth = 0.1
        self.Randomize_percentage = 0.3
        self.Sonar_color = [255.]
        self.Camera_color = [0.]
        self.sonar_dimensions = (0.5,0.5)
        self.sonar_tolerance = 0.5
        self.camera_ratio = 1.0
        self.camera_tolerance = 1.0
        self.Count = 0
        self.CamStart = 0
        self.Weightage = 0.5

        # For testing
        # self.Xstd_sonar_pos = 0.
        # self.Xstd_sonar_vec = 0.
        # self.Xstd_sonar_depth = 0.
        # self.Randomize_percentage = 0.0

        # Create Particles in 3D M
        self.create_particles()

    def set_position(self, position=(0,0,0), variance=(0,0,0)):
        self.State = np.asarray([
            position[0] + variance[0] * np.random.randn(self.Npop_particles), 
            position[1] + variance[1] * np.random.randn(self.Npop_particles), 
            position[2] + variance[2] * np.random.randn(self.Npop_particles), 
            np.zeros(self.Npop_particles),
            np.zeros(self.Npop_particles),
            np.zeros(self.Npop_particles),
            np.random.randint(0, 2, self.Npop_particles),
        ])

    def create_particles(self):

        # Update matrix for velocity to distance
        self.F_update = np.matrix([
            [1, 0, 0, 1, 0, 0, 0],
            [0, 1, 0, 0, 1, 0, 0],
            [0, 0, 1, 0, 0, 1, 0],
            [0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 1],
        ])

        # Set up weight vector
        self.Weights = np.zeros((1,self.Npop_particles))

        # Setup uniform pixel particles in sonar image
        self.SonarPixels = np.asarray([
            np.random.randint(1, self.Sonar_resolution[0], self.Npop_particles), 
            np.random.randint(1, self.Sonar_resolution[1], self.Npop_particles)
        ])

        # Setup uniform pixel particles in camera image
        self.CameraPixels = np.asarray([
            np.random.randint(1, self.Camera_resolution[0], self.Npop_particles), 
            np.random.randint(1, self.Camera_resolution[1], self.Npop_particles)
        ])

        # Map sonar pixels to RTheta
        RTheta = np.zeros((2,self.Npop_particles))
        for i, (xp, yp) in enumerate(zip(self.SonarPixels[0,:], self.SonarPixels[1,:])):
            RTheta[:,i] = sonar_pixel[xp][yp]

        # Generate a Y vector of range 2m
        Y = np.asarray([
            np.random.uniform(-3.0, 3.0, self.Npop_particles)
        ])

        # Generate State vector
        self.State = np.asarray([
            (np.sqrt(np.power(RTheta[0,:],2) - np.power(Y,2))*np.sin(np.radians(RTheta[1,:])))[0],
            Y[0],
            (np.sqrt(np.power(RTheta[0,:],2) - np.power(Y,2))*np.cos(np.radians(RTheta[1,:])))[0],
            np.zeros(self.Npop_particles),
            np.zeros(self.Npop_particles),
            np.zeros(self.Npop_particles),
            np.random.randint(0, 2, self.Npop_particles),
        ])

    def update_particles(self, vehicleVelocity=(0.,0.,0.), imuVelocity=(0.,0.,0.), fpsRate=1., mapper=None):
        # vehicleVelocity=(0.,0.,0.)
        # imuVelocity=(0.,0.,0.)
        # self.Xstd_sonar_vec = 0.2

        # Update count
        self.Count = self.Count + 1

        # Scale velocity based on data rate
        timeScale = 1./fpsRate
        imuVelocity = (-imuVelocity[0]*timeScale,imuVelocity[1]*timeScale,-imuVelocity[2]*timeScale)
        vehicleVelocity = (vehicleVelocity[0]*timeScale,-vehicleVelocity[1]*timeScale,vehicleVelocity[2]*timeScale)

        # Reset a percentage of the particles
        Hits = int(self.Npop_particles*self.Randomize_percentage)
        self.randomized_select = np.random.choice(range(self.Npop_particles), Hits, replace=False)
        self.State[:,self.randomized_select] = np.asarray([
            np.random.uniform(-4.7, 4.7, Hits), 
            np.random.uniform(-3.0, 3.0, Hits), 
            np.random.uniform(2.2, 14.1, Hits),
            np.zeros(Hits),
            np.zeros(Hits),
            np.zeros(Hits),
            np.random.randint(0, 2, Hits),
        ])

        # Predict next state based on velocity information
        R = (np.sqrt(np.power(self.State[0,:],2)+np.power(self.State[1,:],2)+np.power(self.State[2,:],2)))
        RThetaPhi = np.asarray([
            R,
            np.degrees(np.arctan2(self.State[0,:],self.State[2,:])),
            np.degrees(np.arctan2(self.State[1,:],R))
        ])
        self.State[3,:] = RThetaPhi[0,:] * -np.sin(np.radians(RThetaPhi[2,:])) * np.sin(np.radians(RThetaPhi[1,:])) * imuVelocity[1] + RThetaPhi[0,:] * np.cos(np.radians(RThetaPhi[2,:])) * np.cos(np.radians(RThetaPhi[1,:])) * imuVelocity[2] + vehicleVelocity[0]
        self.State[4,:] = RThetaPhi[0,:] * np.cos(np.radians(RThetaPhi[2,:])) * imuVelocity[1] + vehicleVelocity[1]
        self.State[5,:] = RThetaPhi[0,:] * -np.sin(np.radians(RThetaPhi[2,:])) * np.cos(np.radians(RThetaPhi[1,:])) * imuVelocity[1] + RThetaPhi[0,:] * np.cos(np.radians(RThetaPhi[2,:])) * -np.sin(np.radians(RThetaPhi[1,:])) * imuVelocity[2] + vehicleVelocity[2]
        self.State = np.asarray(self.F_update * self.State)

        # Set sonar Pixels
        xs, ys, zs, xvs, yvs, zvs, mults = self.State
        docalcs = (-4.7 <= xs) & (xs < 4.7) & (2.2 <= zs) & (zs < 14.1)
        for i, (x, z, docalc) in enumerate(zip(xs, zs, docalcs)):
            if not docalc:
                continue
            self.SonarPixels[:,i] = sonar_xym_map(x,z)

        # Add noise
        self.State[0:1,:] = self.State[0:1,:] + self.Xstd_sonar_pos * np.random.randn(1,self.Npop_particles)
        self.State[2:3,:] = self.State[2:3,:] + self.Xstd_sonar_pos * np.random.randn(1,self.Npop_particles)
        self.State[1:2,:] = self.State[1:2,:] + self.Xstd_sonar_depth * np.random.randn(1,self.Npop_particles)
        self.State[3:6,:] = self.State[3:6,:] + self.Xstd_sonar_vec * np.random.randn(3,self.Npop_particles)

        # Project particles into camera
        FinalMatrix = np.asarray(mapper.F * mapper.CTS * np.vstack([self.State[0:3,:], np.ones(self.Npop_particles)]))
        UVMatrix = np.asarray([
            (FinalMatrix[0,:]/FinalMatrix[2,:]),
            (FinalMatrix[1,:]/FinalMatrix[2,:]),
        ])
        UVMatrix[np.isnan(UVMatrix)] = 0.
        UVMatrix = UVMatrix.astype(int)
        self.CameraPixels = UVMatrix

    def calc_log_likelihood(self, sonarImage, cameraImage, sonarProcessed, cameraProcessed, sonarArray, cameraArray, ros):

        # Get Sonar Information
        A_sonar_color = -np.log(np.sqrt(2 * np.pi) * self.Xstd_sonar_color);
        B_sonar_color = - 0.5 / (self.Xstd_sonar_color**2);
        A_sonar_size = -np.log(np.sqrt(2 * np.pi) * self.Xstd_sonar_size);
        B_sonar_size = - 0.5 / (self.Xstd_sonar_size**2);
        A_camera_color = -np.log(np.sqrt(2 * np.pi) * self.Xstd_camera_color);
        B_camera_color = - 0.5 / (self.Xstd_camera_color**2);
        A_camera_ratio = -np.log(np.sqrt(2 * np.pi) * self.Xstd_camera_ratio);
        B_camera_ratio = - 0.5 / (self.Xstd_camera_ratio**2);

        # Setup state and weight matrices
        self.Weights = np.zeros((1,self.Npop_particles))
        self.Weights[...] = -10000000000

        # Precalculate Data
        xs, ys, zs, xvs, yvs, zvs, mults = self.State
        us, vs = self.CameraPixels
        calcSonars = (-4.7 <= xs) & (xs < 4.7) & (2.2 <= zs) & (zs < 14.1)
        calcCameras = (0 <= us) & (us < 780) & (0 <= vs) & (vs < 580)

        # Color differences
        Dsonar = self.Sonar_color - sonarImage
        cameraHSV = cv2.cvtColor(cameraImage,cv2.COLOR_BGR2HSV)
        Dcamera = self.Camera_color - cameraHSV

        # Sonar size (Higher it is, faster it converges)
        sonar_dimensions = self.sonar_dimensions
        sonar_tolerance = (abs(sonar_dimensions[0]-self.sonar_tolerance),abs(sonar_dimensions[1]-self.sonar_tolerance))

        # Camera size (Higher it is, faster it converges)
        camera_ratio = self.camera_ratio
        camera_tolerance = abs(camera_ratio - self.camera_tolerance)

        # Iterate through items
        for i, (x, z, u, v, mult, calcSonar, calcCamera) in enumerate(zip(xs, zs, us, vs, mults, calcSonars, calcCameras)):

            total_weight = 0

            # Use sonar
            if calcSonar and calcCamera:

                # Weights
                sonarW = 0
                cameraW = 0

                # Get Sonar Pixels
                x0p, x1p = self.SonarPixels[:,i]

                # Check if particle inside object and assign objects
                MA = sonar_tolerance[0]
                ma = sonar_tolerance[1]
                for sonarItem in sonarArray:
                    if cv2.pointPolygonTest(sonarItem["Contour"], (x0p,x1p), False) > 0:
                        if sonarItem["TrackValid"]:
                            MA = sonarItem["MA"]
                            ma = sonarItem["ma"]
                        break
                MA_weight = A_sonar_size + B_sonar_size * abs(sonar_dimensions[0]-MA)**1.5;
                ma_weight = A_sonar_size + B_sonar_size * abs(sonar_dimensions[1]-ma)**1.5;
                sonarW =  sonarW + MA_weight + ma_weight

                # Get color for that particle sonar
                D = Dsonar[x1p,x0p]
                color_weight = A_sonar_color + B_sonar_color * abs(D)**1.5;
                sonarW =  sonarW + color_weight

                # Custom function for multi track
                # sonarW =  sonarW + A_sonar_size + B_sonar_size * (abs(number-mult)**2);
                # if mult == 0:
                #     sonarW =  sonarW + A_sonar_color + B_sonar_color * (abs(number-)**2);
                # elif mult == 1:
                #     sonarW =  sonarW + A_sonar_color + B_sonar_color * (abs(300-x0p)**2);

                # Camera Ratio
                object_ratio = 0
                for cameraItem in cameraArray:
                    if cv2.pointPolygonTest(cameraItem["Contour"], (u,v), False) > 0:
                        object_ratio = cameraItem["Ratio"]
                        break
                camera_ratio_weight = A_camera_ratio + B_camera_ratio * abs(camera_ratio-object_ratio)**2;
                cameraW = cameraW + camera_ratio_weight

                # Camera Color
                D = Dcamera[v,u][0]
                color_weight = A_camera_color + B_camera_color *  abs(D)**1.5;
                cameraW =  cameraW + color_weight

                # Weightage
                cameraW = cameraW*(1. - self.Weightage)
                sonarW = sonarW*(self.Weightage)

                if self.Count > self.CamStart:
                    self.Weights[:,i] = sonarW + cameraW
                else:
                    self.Weights[:,i] = sonarW
                    

    def resample_particles(self):
        L = np.exp(self.Weights - np.amax(self.Weights));
        Q = L / np.sum(L);
        R = np.cumsum(Q);
        T = np.random.rand(1, self.Npop_particles)
        a,b = np.histogram(T,R)
        I = np.digitize(T.tolist()[0],b)
        self.State = self.State[:, I];

    def draw_particles(self, cameraProcessed, sonarProcessed, ros):

        # Draw sonar particles
        SonarPixels = np.delete(self.SonarPixels, self.randomized_select, axis=1)
        for i, (x0, x1) in enumerate(zip(SonarPixels[0,:], SonarPixels[1,:])):
            cv2.circle(sonarProcessed,(x0,x1), 2, (0,255,0), -1)

        # Draw camera particles
        CameraPixels = np.delete(self.CameraPixels, self.randomized_select, axis=1)
        for i, (x0, x1) in enumerate(zip(CameraPixels[0,:], CameraPixels[1,:])):
            cv2.circle(cameraProcessed,(x0,x1), 2, (0,255,0), -1)

        #rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0 0.0 1.0 base_link map 1000

    def compute_mean(self):

        State = np.delete(self.State, self.randomized_select, axis=1)
        space_mean = np.mean(State, axis=1)
        return space_mean


    def compute_gmm(self):

        State = np.delete(self.State, self.randomized_select, axis=1)
        X = State[0:3,:].T
        lowest_bic = np.infty
        bic = []
        n_components_range = [1,2]
        cv_types = ['full']
        for cv_type in cv_types:
            for n_components in n_components_range:
                gmm = mixture.GMM(n_components=n_components, covariance_type=cv_type)
                gmm.fit(X)
                bicval = gmm.bic(X)
                bic.append(bicval)
                if bic[-1] < lowest_bic:
                    lowest_bic = bic[-1]
                    best_gmm = gmm
        clf = best_gmm

        # Look at GMM solutions
        color_iter = itertools.cycle([(255,0,0),(0,255,255),(0,0,255),(255,0,255),(255,255,255)])
        self.Solutions = np.asarray([
            np.zeros(len(clf.means_)),
            np.zeros(len(clf.means_)),
            np.zeros(len(clf.means_))
        ])
        solutions = []
        for i, (mean, covar, color) in enumerate(zip(clf.means_, clf.covars_,color_iter)):
            solutions.append({"Mean":mean, "Convariance":covar})
            
        return solutions
            # print mean
            # print covar
            # print "--"
            # Get covariance features
            # cv2.circle(processed,(int(mean[0]),int(mean[1])), 5, (color), -1)
            # solution = {"Mean":mean, "Covar":covar, "Item":None}
            # v, w = linalg.eigh(covar)
            # angle = np.arctan2(w[0][1], w[0][0])
            # angle = 180 * angle / np.pi

            # # Match solution with sonar item if it exists
            # for item in frame_array:
            #     distance = Math.l2_norm_2d(item["Centroid"], mean)
            #     if distance < 20:
            #         solution["Item"] = item
            #         break

        #     # If i don't have a match use covariance
        #     # if solution["Item"] is None:
        #     #     poly = cv2.ellipse2Poly((int(mean[0]),int(mean[1])), (int(v[0]),int(v[1])), int(angle), 0, 360, 1)
        #     #     x,y,w,h = cv2.boundingRect(np.array([poly]))
        #     #     lbitem, rbitem = Img.side_contour_points_modified(poly, [x,y,w,h])
        #     #     cv2.circle(processed,lbitem, 2, (0,0,255), -1)
        #     #     cv2.circle(processed,rbitem, 2, (0,0,255), -1)
        #     #     RThetaLeftSide = Img.sonar_pixel(lbitem[0],lbitem[1])
        #     #     RThetaRightSide = Img.sonar_pixel(rbitem[0],rbitem[1])
        #     #     RTheta = Img.sonar_pixel(int(mean[0]),int(mean[1]))
        #     #     item = {}
        #     #     item["RTheta"] = RTheta
        #     #     item["RThetaLeftSide"] = RThetaLeftSide
        #     #     item["RThetaRightSide"] = RThetaRightSide
        #     #     solution["Item"] = item
        # print solutions

class CvAverage():

    maxSize=3
    queueList=[]

    def __init__(self, width, height, maxSize=3):
        self.width=width
        self.height=height
        self.maxSize = maxSize
        for x in range(0, self.maxSize):
            self.queueList.append(self.zero_image())

    def add_image(self, img):
        self.queueList.pop(0)
        self.queueList.append(img.astype(np.uint16))

    def process_image(self):
        accum = self.zero_image() 
        for x in range(0, self.maxSize):
            runimg=self.queueList[x]
            accum+=runimg

        averaged = accum/self.maxSize
        averaged = np.array(averaged, dtype=np.uint8)
        return averaged

    def zero_image(self):
        zeroMat=np.zeros((self.height, self.width, 3))
        zeroMat = np.array(zeroMat * 255, dtype = np.uint16)
        return zeroMat