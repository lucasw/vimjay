#!/usr/bin/env python
# Lucas Walter

# http://stackoverflow.com/questions/14347163/pyqt-irregularly-shaped-windows-e-g-a-circular-without-a-border-decorations

import cv2
import numpy as np
import rospy
import signal
import sys
from sensor_msgs.msg import Image, RegionOfInterest
from PyQt4 import QtGui #, QtWebKit
from PyQt4.QtGui import QImage
from PyQt4.QtCore import Qt, QTimer, QSize

class RoundWindow(QtGui.QLabel):  
    def __init__(self):
        super(RoundWindow, self).__init__()
        signal.signal(signal.SIGINT, self.sigintHandler)
         
        self.wd  = rospy.get_param("~width", 360)
        self.ht = rospy.get_param("~height", 240)
        self.x = rospy.get_param("~x", 0)
        self.y = rospy.get_param("~y", 0)
        self.roi = None
        self.new_roi = False
        self.keep_aspect = rospy.get_param("~keep_aspect", True)

        self.initUI()
        self.update_im = False
        self.image_sub = rospy.Subscriber('image', Image, self.imageCallback,
                queue_size=1)
        self.roi_sub = rospy.Subscriber('roi', RegionOfInterest, self.roiCallback, 
                queue_size=1) 
        
        self.timer = QTimer()
        self.timer.start(100)
        self.timer.timeout.connect(self.update) 
        
    # TODO make a command line image publisher to test this
    # rosrun vimjay pub_image test.png /test
    def imageCallback(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        sz = (msg.height, msg.width, msg.step / msg.width)
        image_pre = np.reshape(np_arr, sz)
        #print image.shape
        #cv2.imshow("test", image)
        #cv2.waitKey(0)
       
        # don't keep aspect
        aspect_1 = float(self.wd) / float(self.ht)
        aspect_2 = float(msg.width) / float(msg.height)
        if not self.keep_aspect and aspect_1 != aspect_2:
            image = cv2.resize(image_pre, (self.wd, self.ht), 
                    interpolation = cv2.INTER_NEAREST)
        else:
            image = np.zeros((self.ht, self.wd, 3), np.uint8)
            # the destination is wider than the source
            sx = 0
            ex = self.wd
            sy = 0
            ey = self.ht
            if aspect_1 > aspect_2:
                sc = float(self.ht) / float(msg.height)
                nx = sc * msg.width
                sx = int((self.wd - nx)/2)
                ex = int(sx + nx)
            else:
                sc = float(self.wd) / float(msg.width)
                ny = sc * msg.height
                sy = int((self.ht - ny)/2)
                ey = int(sy + ny)
            image_sc = cv2.resize(image_pre, (int(ex - sx), int(ey - sy)),
                    interpolation = cv2.INTER_NEAREST)
            image[sy:ey, sx:ex, :] = image_sc 

        # TBD need to keep aspect ratio correct if desired
        self.qimage = QImage.rgbSwapped(QImage(image.tostring(), self.wd, self.ht, 
                QImage.Format_RGB888))
        self.update_im = True 
        #self.resize(msg.width
        #print sz

    def sigintHandler(self, signal, frame):
        print 'ctrl-c'
        quit()

    def roiCallback(self, msg):
        self.roi = msg
        self.new_roi = True

    def initUI(self):
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.qimage = QImage(self.wd, self.ht, QImage.Format_RGB32)
        # generate a test image
        for i in range(self.ht):
            for j in range(self.wd):
                self.qimage.setPixel(j, i, QtGui.qRgb(i%255,j%255,128))
        self.setPixmap(QtGui.QPixmap.fromImage(self.qimage))
        self.show()
    
    def update(self):
        if self.qimage is not None and self.update_im:
            self.setPixmap(QtGui.QPixmap.fromImage(self.qimage))
            self.update_im = False
        
        if self.new_roi:
            if self.roi.width != 0 and self.roi.height != 0:
                self.wd = self.roi.width
                self.ht = self.roi.height
                self.resize(self.roi.width, self.roi.height)
            self.x = self.roi.x_offset
            self.y = self.roi.y_offset
            self.move(self.x, self.y)
            self.new_roi = False

    def sizeHint(self):
        return QSize(self.wd, self.ht)

if __name__ == '__main__':
    rospy.init_node('image_undecorated')
    app = QtGui.QApplication([])
    # This timer allows ctrl-c message to be handled
    timer = QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None) 
    rw = RoundWindow()
    sys.exit(app.exec_())
