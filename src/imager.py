#!/usr/bin/python3
# coding:utf-8 
import os
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from copy import deepcopy
import tensorflow as tf
from pretrain import CNN
from std_msgs.msg import Int32MultiArray

def get_letter(num):
    return chr(num + 64) + " / " + chr(num + 96)  # 大写/小写字母

class Predict(object):
    def __init__(self):
        latest = tf.train.latest_checkpoint('./src/doodbot/CNNdata/modelckpt')
        self.cnn = CNN()
        self.cnn.model.load_weights(latest)

    def predict_tf(self, image):
        image = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
        _, image = cv2.threshold(image,245,255,cv2.THRESH_BINARY)
        image = cv2.copyMakeBorder(image,10,10,10,10, cv2.BORDER_CONSTANT,value=255)
        image = cv2.resize(cv2.rotate(cv2.flip(image,1), cv2.ROTATE_90_CLOCKWISE), (28, 28), interpolation=cv2.INTER_AREA)
        image = np.reshape(image, (28, 28, 1)) / 255
        x = np.array([1 - image])

        y = self.cnn.model.predict(x)

        # print(np.argmax(y[0]))
        # print('\t-> 这个图片写的是：', get_letter(np.argmax(y[0])))
        return np.argmax(y[0])

    def predict_normal(self, image):
        pass


class Imager():
    def __init__(self):

        self._bridge = CvBridge()
        self._predicter = Predict()


        self._sub = rospy.Subscriber('kinect2/hd/image_color', Image, self.callback, queue_size=1)
        self._pub0 = rospy.Publisher('newimage', Image, queue_size=1)
        self._pub1 = rospy.Publisher('OXstate', Int32MultiArray, queue_size=1)
        self.rate = rospy.Rate(10)


    def linepainter(self, lines, image):
        # 把线在图上标出来
        for line in lines:
            r = line[0]
            theta = line[1]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*r
            y0 = b*r
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            cv2.line(image,(x1,y1), (x2,y2), (0,0,255),1)

    def preprocess(self, image):
        # 裁减
        rows, cols, _ = image.shape
        rospy.loginfo("r:{}, c:{}".format(rows,cols))
        return image[int(rows*0.65):int(rows*0.9), int(cols*0.27):int(cols*0.45)]


    def linefilter(self, lines):
        # 过滤重叠的线
        target_lines = []
        for line in lines:
            r = line[0][0]
            theta = line[0][1]
            flag = True
            for target_line in target_lines:
                if abs(r - target_line[0]) <20 and abs(theta - target_line[1]) < 20 * np.pi/180:
                    flag = False
                    break
            if flag:
                target_lines.append([r,theta])
                if len(target_lines) >= 4:
                    break

        return target_lines

    def pointfinder(self, target_lines):
        # 找到井字的四个交点
        target_points = []
        for i, target_line_0 in enumerate(target_lines):
            for j, target_line_1 in enumerate(target_lines):
                if j <= i:
                    continue
                r0, t0 = target_line_0[0], target_line_0[1]
                r1, t1 = target_line_1[0], target_line_1[1]
                if abs(t0 - t1) > 40 * np.pi/180:
                    A = np.array([
                        [np.cos(t0),np.sin(t0)],
                        [np.cos(t1),np.sin(t1)]
                    ])
                    b = np.array([r0,r1])
                    x0, y0 = np.linalg.solve(A,b)
                    target_points.append([int(np.round(x0)), int(np.round(y0))])
                    if len(target_points) >= 4:
                        break
            if len(target_points) >= 4:
                break

        return target_points

    def pointsorter(self, target_points):
        # 按照 下左上右 的顺序把交点排序
        sorted_points = []
        ##################sh*t code to sort the point###############################################
        for i, target_point_0 in enumerate(target_points):
            flag = True
            for j, target_point_1 in enumerate(target_points):
                if j == i:
                    continue
                if target_point_0[1] < target_point_1[1]:
                    flag = False
                    break
            if flag:
                sorted_points.append([target_point_0[0], target_point_0[1]])
                break
        for i, target_point_0 in enumerate(target_points):
            flag = True
            for j, target_point_1 in enumerate(target_points):
                if j == i:
                    continue
                if target_point_0[0] > target_point_1[0]:
                    flag = False
                    break
            if flag:
                sorted_points.append([target_point_0[0], target_point_0[1]])
                break
        for i, target_point_0 in enumerate(target_points):
            flag = True
            for j, target_point_1 in enumerate(target_points):
                if j == i:
                    continue
                if target_point_0[0] < target_point_1[0]:
                    flag = False
                    break
            if flag:
                sorted_points.append([target_point_0[0], target_point_0[1]])
                break
        for i, target_point_0 in enumerate(target_points):
            flag = True
            for j, target_point_1 in enumerate(target_points):
                if j == i:
                    continue
                if target_point_0[1] > target_point_1[1]:
                    flag = False
                    break
            if flag:
                sorted_points.append([target_point_0[0], target_point_0[1]])
                break
        ##################sh*t code to sort the point###############################################
        return sorted_points

    def findboard(self, image):
        # # 边缘检测
        # image = cv2.GaussianBlur(image, (3,3), 0)
        edges = cv2.Canny(image, 30, 70)

        # # 角点检测
        # corners = cv2.cornerHarris(edges,2,3,0.045)
        # corners = cv2.dilate(corners,None)
        # image[dst>0.2*dst.max()]=255

        lines = cv2.HoughLines(edges,1.5,np.pi/180, 100)

        if lines is None:
            rospy.loginfo("No line found!")
        else:
            target_lines = self.linefilter(lines)
            if len(target_lines) < 4:
                rospy.loginfo("Lines not enough!")
            else:
                # self.linepainter(target_lines, image)
                target_points = self.pointfinder(target_lines)
                if len(target_points) < 4:
                    rospy.loginfo("Points not enough!")
                else:
                    sorted_points = self.pointsorter(target_points)
                    return sorted_points

        return []

    def projection(self, crosspoints, image):
        # 把井字棋投影变换到上帝视角
        p1 = np.float32([
            [crosspoints[0][0],crosspoints[0][1]], 
            [crosspoints[1][0],crosspoints[1][1]],
            [crosspoints[2][0],crosspoints[2][1]],
            [crosspoints[3][0],crosspoints[3][1]]
        ])
        p2 = np.float32([
            [400,400], 
            [600,400],
            [400,600],
            [600,600]
        ])
        M = cv2.getPerspectiveTransform(p1,p2)
        return cv2.warpPerspective(image, M, (1000, 1000))

        

    def callback(self, image_in):
        image_cv = self._bridge.imgmsg_to_cv2(image_in, desired_encoding='bgr8')

        image_cv = self.preprocess(image_cv)

        crosspoints = self.findboard(image_cv)


        OXresult = [
            [' ', ' ', ' '],
            [' ', ' ', ' '],
            [' ', ' ', ' ']
        ]
        OXstate = Int32MultiArray()
        OXstate.data = [
            0, 0, 0,
            0, 0, 0,
            0, 0, 0
        ]
        if len(crosspoints) == 0:
            rospy.loginfo("No board found!")
        else:
            for crosspoint in crosspoints:
                # cv2.circle(image_cv,(crosspoint[0],crosspoint[1]),1,(0,255,0),4)
                rospy.loginfo("x:{}, y:{}".format(crosspoint[0],crosspoint[1]))

            image_cv = self.projection(crosspoints, image_cv)

            for i in range(3):
                for j in range(3):
                    begin_t = rospy.Time.now()
                    OXsymNum = self._predicter.predict_tf(image_cv[200 + i*200 + 10: 200 + i*200 + 190, 200 + j*200 + 10: 200 + j*200 + 190])
                    end_t = rospy.Time.now()
                    rospy.loginfo("Duration: {}".format((end_t - begin_t).to_sec()))

                    if OXsymNum == 15 or OXsymNum == 4:
                        # O / D
                        OXresult[i][j] = 'O'
                        OXstate.data[i*3+j] = 2
                    elif OXsymNum == 24 or OXsymNum == 25:
                        # X / Y
                        OXresult[i][j] = 'X'
                        OXstate.data[i*3+j] = 3
                    else:
                        OXresult[i][j] = ' '
                        OXstate.data[i*3+j] = 1
                        
                
        # i = 0
        # j = 2
        # tmp = image_cv[200 + i*200 + 10: 200 + i*200 + 190, 200 + j*200 + 10: 200 + j*200 + 190]
        # tmp = cv2.cvtColor(tmp, cv2.COLOR_RGB2GRAY)
        # _, tmp = cv2.threshold(tmp,245,255,cv2.THRESH_BINARY)
        # tmp = cv2.copyMakeBorder(tmp,10,10,10,10, cv2.BORDER_CONSTANT,value=255)
        # tmp = cv2.cvtColor(tmp, cv2.COLOR_GRAY2RGB)
        # result = self._bridge.cv2_to_imgmsg(tmp, encoding='bgr8')

        result = self._bridge.cv2_to_imgmsg(image_cv, encoding='bgr8')
        

        self._pub0.publish(result)
        self._pub1.publish(OXstate)
        print(OXresult[0])
        print(OXresult[1])
        print(OXresult[2])

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('imager')
    imager = Imager()
    imager.main()
