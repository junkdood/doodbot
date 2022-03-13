#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


class Imager():
    def __init__(self):
        self._sub = rospy.Subscriber('kinect2/hd/image_color', Image, self.callback, queue_size=1)
        self._pub = rospy.Publisher('newimage', Image, queue_size=1)
        self.rate = rospy.Rate(10)

        self._bridge = CvBridge()

    def callback(self, image_in):
        image_cv = self._bridge.imgmsg_to_cv2(image_in, desired_encoding='bgr8')

        #裁减
        rows, cols, _ = image_cv.shape
        rospy.loginfo("r:{} ,c:{}".format(rows,cols))
        image_cv = image_cv[int(rows*0.5):int(rows*0.9), int(cols*0.25):int(cols*0.6)]

        # #透视变换
        # rows, cols, _ = image_cv.shape
        # p1 = np.float32([[0,0], [cols-1,0], [0,rows-1], [cols-1,rows-1]])
        # p2 = np.float32([[0,rows*0.5], [cols*0.5,0], [cols*0.5,rows-1], [cols-1,rows*0.7]])
        # M = cv2.getPerspectiveTransform(p1,p2)
        # image_cv = cv2.warpPerspective(image_cv, M, (cols, rows))


        result = self._bridge.cv2_to_imgmsg(image_cv, encoding='bgr8')
        self._pub.publish(result)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('imager')
    imager = Imager()
    imager.main()
