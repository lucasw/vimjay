#!/usr/bin/env python
# Lucas Walter

# http://stackoverflow.com/questions/14347163/pyqt-irregularly-shaped-windows-e-g-a-circular-without-a-border-decorations

#import cv2
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
        self.initUI()
        self.update_im = False
        self.image_sub = rospy.Subscriber('image', Image, self.imageCallback,
                queue_size=1)
        self.roi_sub = rospy.Subscriber('test', RegionOfInterest, self.roiCallback, 
                queue_size=1) 
        
        self.timer = QTimer()
        self.timer.start(100)
        self.timer.timeout.connect(self.update) 
        
    # TODO make a command line image publisher to test this
    # rosrun vimjay pub_image test.png /test
    def imageCallback(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        sz = (msg.height, msg.width, msg.step / msg.width)
        #image = np.reshape(np_arr, sz)
        #print image.shape
        #cv2.imshow("test", image)
        #cv2.waitKey(0)

        # TBD need to keep aspect ratio correct if desired
        self.qimage = QImage.rgbSwapped(QImage(msg.data, msg.width, msg.height, 
                QImage.Format_RGB888))
        self.update_im = True 
        #self.resize(msg.width
        #print sz

    def sigintHandler(self, signal, frame):
        print 'ctrl-c'
        quit()

    def roiCallback(self, msg):
        if msg.width != 0 and msg.height != 0:
            self.resize(msg.width, msg.height)
        self.move(msg.x_offset, msg.y_offset)

    def initUI(self):
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.qimage = QImage(300, 300, QImage.Format_RGB32)
        for i in range(255):
            for j in range(255):
                self.qimage.setPixel(j, i, QtGui.qRgb(i,j,128))
        self.setPixmap(QtGui.QPixmap.fromImage(self.qimage))
        self.show()
    
    def update(self):
        if self.qimage is not None and self.update_im:
            self.setPixmap(QtGui.QPixmap.fromImage(self.qimage))
        self.update_im = False

    def sizeHint(self):
        return QSize(300,300)

if __name__ == '__main__':
    rospy.init_node('image_undecorated')
    app = QtGui.QApplication([])
    # This timer allows ctrl-c message to be handled
    timer = QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None) 
    rw = RoundWindow()
    sys.exit(app.exec_())
