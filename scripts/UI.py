# GUI
import sys
import resource
from Memory import *
from PyQt4 import QtGui, QtCore, uic

# CV
import cv2
from cv2 import cv
import numpy as np
import time

# ROS
from Ros import *

# GL
import sys
import math
from PyQt4 import QtCore, QtGui, QtOpenGL
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

VISUAL = True

class SonarWindow(QtGui.QWidget):
    def __init__(self):
        super(SonarWindow, self).__init__()
        self.ros = ROS(ros_callback=self.ros_callback, ui_callback=self.ui_callback, mode="UI")
        self.data = None
        uic.loadUi('UI.ui', self)
        self.initialize()
        self.show()

    def ui_callback(self):
        pass

    def ros_callback(self, data):

        """""""""""""""""""""""""""""""""
        Data capture
        """""""""""""""""""""""""""""""""

        # Get Data
        self.data = data


    def initialize(self):
        if VISUAL:
            self.glWidget = GLWidget()
            self.glWindow.addWidget(self.glWidget)

        self.contours_cb.stateChanged.connect(lambda : self.settings_changed(value=self.contours_cb.isChecked(), name="Contours"))
        self.spatial_cb.stateChanged.connect(lambda : self.settings_changed(value=self.spatial_cb.isChecked(), name="Spatial"))

        self.threshold_slider.valueChanged[int].connect(lambda : self.settings_changed(value=self.threshold_slider.value(), name="Threshold"))
        #self.threshold_slider.setSliderPosition(self.sonar.setThreshold)
        self.area_slider.valueChanged[int].connect(lambda : self.settings_changed(value=self.area_slider.value(), name="Area"))
        #self.area_slider.setSliderPosition(self.sonar.setThreshold)
        self.threshold_label.setText("Threshold: "+str(self.threshold_slider.value()))
        self.area_label.setText("Area: "+str(self.area_slider.value()))

        self.group1_next_button.clicked.connect(lambda : self.button_click("Group1Next"))
        self.group1_prev_button.clicked.connect(lambda : self.button_click("Group1Prev"))
        self.group2_next_button.clicked.connect(lambda : self.button_click("Group2Next"))
        self.group2_prev_button.clicked.connect(lambda : self.button_click("Group2Prev"))
        self.saveData_button.clicked.connect(lambda : self.button_click("SaveData"))
        self.writeData_button.clicked.connect(lambda : self.button_click("WriteData"))

        self.image1_label.mouseMoveEvent = self.mouse_move_image1
        self.image1_label_mouse = (0,0)
        self.image2_label.mouseMoveEvent = self.mouse_move_image2
        self.image2_label_mouse = (0,0)

        # self.set_image("image1", self.blank_image(480,394))
        # self.set_image("image2", self.blank_image(640,480))
        # self.set_image("image3", self.blank_image(100,100))
        # self.set_image("image4", self.blank_image(100,100))

        self.image_timer = QtCore.QTimer(self)
        self.image_timer.timeout.connect(self.image)
        self.image_timer.start(60)

        self.visual_timer = QtCore.QTimer(self)
        self.visual_timer.timeout.connect(self.visual)
        self.visual_timer.start(200)

        self.memory_timer = QtCore.QTimer(self)
        self.memory_timer.timeout.connect(self.memory)
        self.memory_timer.start(600)

        self.ros.start()

        self.group1_stack.setCurrentIndex(0)
        time.sleep(1)
        # self.group1_stack.setCurrentIndex(1)

    def mouse_move_image1(self, mevent):
        x = mevent.x()
        y = mevent.y()
        if x<0 or y<0:
            return
        self.image1_label_mouse = (x,y)
        self.ros.send_settings({"SonarImageMouse": self.image1_label_mouse})

    def mouse_move_image2(self, mevent):
        x = mevent.x()
        y = mevent.y()
        if x<0 or y<0:
            return
        self.image2_label_mouse = (x,y)
        self.ros.send_settings({"CameraImageMouse": self.image2_label_mouse})

    def visual(self):
        
        if VISUAL:
            if self.data is None:
                return
            self.glWidget.updateParams(
                x=self.data["Vehicle_Position"][0], 
                y=self.data["Vehicle_Position"][1], 
                depth=self.data["Vehicle_Position"][2], 
                yaw=self.data["IMU_RPY"][2], 
                pitch=self.data["IMU_RPY"][1], 
                roll=self.data["IMU_RPY"][0]
            )

    def image(self):
        if self.data is None:
            return

        # Set Visuals
        text = "Depth: " + str(self.data["Vehicle_Position"][2]) + "\n" + \
                "XYZ: " + str(self.data["Vehicle_Position"][0]) + " " + str(self.data["Vehicle_Position"][1]) + " " + str(self.data["Vehicle_Position"][2]) + "\n" + \
                "RPY: " + str(self.data["IMU_RPY"][0]) + " " + str(self.data["IMU_RPY"][1]) + " " + str(self.data["IMU_RPY"][2]) + "\n" + \
                "XYZ: " + str(self.data["Vehicle_Velocity"][0]) + " " + str(self.data["Vehicle_Velocity"][1]) + " " + str(self.data["Vehicle_Velocity"][2]) + "\n" + \
                ""

        self.data_textBox.setText(text)
        self.set_image("image1", self.data["CameraImage"])
        self.set_image("image2", self.data["SonarImage"])
        self.set_image("image3", self.data["CameraROI"])
        self.set_image("image4", self.data["SonarROI"])      

    def memory(self):
        mem = int(memory()/1024/1024)
        if mem > 3000:
            print "EXCEEDED MEMORY KILLING SCRIPT"
            sys.exit()

    def set_image(self, label, img):
        if img == None:
            return

        if label == "image1":
            img = img.copy()
            cv2.rectangle(img,(self.image1_label_mouse[0]-50,self.image1_label_mouse[1]-50),(self.image1_label_mouse[0]+50,self.image1_label_mouse[1]+50),[255,255,0],2)
        elif label == "image2":
            img = img.copy()
            cv2.rectangle(img,(self.image2_label_mouse[0]-50,self.image2_label_mouse[1]-50),(self.image2_label_mouse[0]+50,self.image2_label_mouse[1]+50),[255,255,0],2)

        height, width, bytesPerComponent = img.shape
        bytesPerLine = bytesPerComponent * width;
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        qtimg = QtGui.QImage(img.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)
        pixmap = QtGui.QPixmap.fromImage(qtimg)

        if label == "image1":
            self.image1_label.setPixmap(pixmap)
        elif label == "image2":
            self.image2_label.setPixmap(pixmap)
        elif label == "image3":
            self.image3_label.setPixmap(pixmap)
        elif label == "image4":
            self.image4_label.setPixmap(pixmap)
        else:
            print "No such label"

    def blank_image(self, x, y):
        return np.zeros((y,x,3), np.uint8)

    def button_click(self, name):
        print name

        if name == "SaveData":
            self.ros.send_settings({"SaveData": 1})
        elif name == "WriteData":
            self.ros.send_settings({"WriteData":  str(self.filename_line.text())})

        if name == "Group1Next":
            self.group1_stack.setCurrentIndex(0)
        elif name == "Group1Prev":
            self.group1_stack.setCurrentIndex(1)
        if name == "Group2Next":
            self.group2_stack.setCurrentIndex(1)
        elif name == "Group2Prev":
            self.group2_stack.setCurrentIndex(0)

    def settings_changed(self, value, name):

        if name == "Threshold" or name == "Area":
            self.threshold_label.setText("Threshold: "+str(self.threshold_slider.value()))
            self.area_label.setText("Area: "+str(self.area_slider.value()))
        
        self.ros.send_settings({name: value})


class GLWidget(QtOpenGL.QGLWidget):
    xRotationChanged = QtCore.pyqtSignal(int)
    yRotationChanged = QtCore.pyqtSignal(int)
    zRotationChanged = QtCore.pyqtSignal(int)

    def __init__(self, parent=None):
        super(GLWidget, self).__init__(parent)
        self.object = 0
        self.gridLines = 0
        self.xRot = 0
        self.yRot = 0
        self.zRot = 0

        self.lastPos = QtCore.QPoint()
        self.mult = 1.0

        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.depth = 0.0
        self.y = 0.0
        self.x = 0.0

        self.trolltechGreen = QtGui.QColor.fromCmykF(0.40, 0.0, 1.0, 0.0)
        self.trolltechPurple = QtGui.QColor.fromCmykF(0.1, 0.1, 0.1, 0.0)


    def minimumSizeHint(self):
        return QtCore.QSize(50, 50)

    def sizeHint(self):
        return QtCore.QSize(400, 400)

    def setXRotation(self, angle):
        angle = self.normalizeAngle(angle)
        if angle != self.xRot:
            self.xRot = angle
            self.xRotationChanged.emit(angle)
            self.updateGL()

    def setYRotation(self, angle):
        angle = self.normalizeAngle(angle)
        if angle != self.yRot:
            self.yRot = angle
            self.yRotationChanged.emit(angle)
            self.updateGL()

    def setZRotation(self, angle):
        angle = self.normalizeAngle(angle)
        if angle != self.zRot:
            self.zRot = angle
            self.zRotationChanged.emit(angle)
            self.updateGL()

    def initializeGL(self):
        self.qglClearColor(self.trolltechPurple.dark())
        glutInit('')
        self.object = self.makeObject()
        self.gridLines = self.makeGridLines()
        glShadeModel(GL_SMOOTH)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_CULL_FACE)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslated(0.0, 0.0, -10.0)
        glRotated(self.xRot / 16.0, 1.0, 0.0, 0.0)
        glRotated(self.yRot / 16.0, 0.0, 1.0, 0.0)
        glRotated(self.zRot / 16.0, 0.0, 0.0, 1.0)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glCallList(self.object)
        glDisable(GL_LIGHTING)
        glDisable(GL_LIGHT0)
        glCallList(self.gridLines)

    def wheelEvent(self,event):
        if event.delta() < 0:
            self.mult += 0.1
        elif event.delta() > 0:
            self.mult -= 0.1
        self.resizeGL(640,480);
        self.updateGL()

    def updateParams(self, x, y, depth, yaw, pitch, roll):
        self.x = 0
        self.y = y
        self.depth = depth
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        glDeleteLists(self.object,1)
        self.object=self.makeObject()
        self.updateGL()

    def resizeGL(self, width, height):
        side = min(width, height)
        if side < 0:
            return

        glViewport((width - side) // 2, (height - side) // 2, side, side)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glFrustum(-1.0*self.mult,1.0*self.mult,-1.0*self.mult,1.0*self.mult,1.5,20.0);
        # GLU.gluLookAt(1, 0, 0, 0, 1, 0, 0, 0, 1);
        glMatrixMode(GL_MODELVIEW)

    def mousePressEvent(self, event):
        self.lastPos = event.pos()

    def mouseMoveEvent(self, event):
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()

        if event.buttons() & QtCore.Qt.LeftButton:
            self.setXRotation(self.xRot + 8 * dy)
            self.setYRotation(self.yRot + 8 * dx)
        elif event.buttons() & QtCore.Qt.RightButton:
            self.setXRotation(self.xRot + 8 * dy)
            self.setZRotation(self.zRot + 8 * dx)

        self.lastPos = event.pos()


    def makeObject(self):
        genList = glGenLists(1)
        glNewList(genList, GL_COMPILE)

        glPushMatrix()
        glTranslatef(self.x,self.y,self.depth)
        glRotatef(self.pitch,0,1,0) # Pitch
        glRotatef(self.roll,1,0,0) # Roll
        glRotatef(self.yaw,0,0,1) # Yaw
        self.draw_sphere()
        glPopMatrix()

        glEndList()

        return genList

    def draw_sphere(self):
        glPushMatrix() 

        glPushMatrix()                   
        glTranslatef(0, 0, 0.0)    
        glColor3f(0.0, 1.0, 0.0)         
        glutSolidSphere(0.3, 250, 250)
        glPopMatrix() 

        glPushMatrix()
        glTranslatef(0.5, 0, 0) 
        glScalef(5,1,1);
        glutSolidSphere(0.1, 250, 250)
        glPopMatrix()

        glPushMatrix()
        glTranslatef(0, 0, 0) 
        glScalef(1,5,1);
        glutSolidSphere(0.1, 250, 250)
        glPopMatrix()

        glPopMatrix()  

    def makeGridLines(self):
        genList = glGenLists(1)
        glNewList(genList, GL_COMPILE)
        self.draw_lines()
        glEndList()
        return genList 

    def glut_print(self, x,  y, z,  font,  text, r,  g , b , a):

        blending = False 
        if glIsEnabled(GL_BLEND) :
            blending = True

        #glEnable(GL_BLEND)
        # glColor3f(1,1,1)
        glRasterPos3f(x,y,z)
        for ch in text :
            glutBitmapCharacter( font , ctypes.c_int( ord(ch) ) )


        if not blending :
            glDisable(GL_BLEND)

    def draw_lines(self):

        # Draw x-axis line.
        glColor3f( 0.3, 0.3, 0.3 )
        for i in np.arange(-10,10,0.5):         
            glBegin( GL_LINES )
            glVertex3f( -10, i, 0 )
            glVertex3f( 10, i, 0 )
            glEnd( )
            self.glut_print( i , 0 , 0, GLUT_BITMAP_8_BY_13 , str(i) , 1.0 , 1.0 , 1.0 , 1.0 )

        # Draw y-axis line.
        glColor3f( 0.3, 0.3, 0.3 )
        for i in np.arange(-10,10,0.5): 
            glBegin( GL_LINES )
            glVertex3f( i, -10, 0 )
            glVertex3f( i, 10, 0 )
            glEnd( )
            self.glut_print( 0 , i , 0, GLUT_BITMAP_8_BY_13 , str(i) , 1.0 , 1.0 , 1.0 , 1.0 )

        # Draw z-axis line.
        glColor3f( 0, 0, 1 )
        for i in np.arange(-10,10,0.5):
            glBegin( GL_LINES )
            glVertex3f( i, 0, 0 )
            glVertex3f( i, 0, 2.5 )
            glEnd( )

        # Draw z-axis line.
        glColor3f( 0, 0, 1 )
        for i in np.arange(0,3.0,0.5):
            glBegin( GL_LINES )
            glVertex3f( -10, 0, i )
            glVertex3f( 10, 0, i )
            glEnd( ) 
            self.glut_print( 0 , 0 , i, GLUT_BITMAP_9_BY_15 , str(i) , 1.0 , 1.0 , 1.0 , 1.0 )


    def quad(self, x1, y1, x2, y2, x3, y3, x4, y4):
        self.qglColor(self.trolltechGreen)

        glVertex3d(x1, y1, -0.05)
        glVertex3d(x2, y2, -0.05)
        glVertex3d(x3, y3, -0.05)
        glVertex3d(x4, y4, -0.05)

        glVertex3d(x4, y4, +0.05)
        glVertex3d(x3, y3, +0.05)
        glVertex3d(x2, y2, +0.05)
        glVertex3d(x1, y1, +0.05)

    def extrude(self, x1, y1, x2, y2):
        self.qglColor(self.trolltechGreen.dark(250 + int(100 * x1)))

        glVertex3d(x1, y1, +0.05)
        glVertex3d(x2, y2, +0.05)
        glVertex3d(x2, y2, -0.05)
        glVertex3d(x1, y1, -0.05)

    def normalizeAngle(self, angle):
        while angle < 0:
            angle += 360 * 16
        while angle > 360 * 16:
            angle -= 360 * 16
        return angle

if __name__ == '__main__':

    rospy.init_node('ui')
    app = QtGui.QApplication(sys.argv)
    window = SonarWindow()
    sys.exit(app.exec_())
