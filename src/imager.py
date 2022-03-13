#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
from copy import deepcopy


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
        rospy.loginfo("r:{}, c:{}".format(rows,cols))
        image_cv = image_cv[int(rows*0.65):int(rows*0.9), int(cols*0.25):int(cols*0.45)]

        # #透视变换
        # rows, cols, _ = image_cv.shape
        # p1 = np.float32([[0,0], [cols-1,0], [0,rows-1], [cols-1,rows-1]])
        # p2 = np.float32([[0,rows*0.5], [cols*0.5,0], [cols*0.5,rows-1], [cols-1,rows*0.7]])
        # M = cv2.getPerspectiveTransform(p1,p2)
        # image_cv = cv2.warpPerspective(image_cv, M, (cols, rows))
        
        # # 边缘检测
        # image_cv = cv2.GaussianBlur(image_cv, (3,3), 0)
        edges = cv2.Canny(image_cv, 30, 100)

        # # 角点检测
        corners = cv2.cornerHarris(edges,2,3,0.045)
        corners = cv2.dilate(corners,None)
        # image_cv[dst>0.2*dst.max()]=255

        lines = cv2.HoughLines(edges,0.95,2*np.pi/180, 100)
        target_lines = []
        if lines is not None:
            for line in lines:
                r = line[0][0]
                theta = line[0][1]
                flag = True
                for target_line in target_lines:
                    if abs(r - target_line[0]) <10 and abs(theta - target_line[1]) < 10 * np.pi/180:
                        flag = False
                        break
                if flag:
                    target_lines.append([r,theta])
                    if len(target_lines) >= 4:
                        break

        target_points = []
        if len(target_lines) >= 4:
            for target_line in target_lines:
                r = target_line[0]
                theta = target_line[1]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*r
                y0 = b*r
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                # cv2.line(image_cv,(x1,y1), (x2,y2), (0,0,255),1)

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

        if len(target_points) >= 4:
            temp_points = deepcopy(target_points)
            target_points = []
            for i, temp_point_0 in enumerate(temp_points):
                flag = True
                for j, temp_point_1 in enumerate(temp_points):
                    if j == i:
                        continue
                    if temp_point_0[0] > temp_point_1[0]:
                        flag = False
                        break
                if flag:
                    target_points.append([temp_point_0[0], temp_point_0[1]])

            for i, temp_point_0 in enumerate(temp_points):
                flag = True
                for j, temp_point_1 in enumerate(temp_points):
                    if j == i:
                        continue
                    if temp_point_0[1] > temp_point_1[1]:
                        flag = False
                        break
                if flag:
                    target_points.append([temp_point_0[0], temp_point_0[1]])
            
            for i, temp_point_0 in enumerate(temp_points):
                flag = True
                for j, temp_point_1 in enumerate(temp_points):
                    if j == i:
                        continue
                    if temp_point_0[1] < temp_point_1[1]:
                        flag = False
                        break
                if flag:
                    target_points.append([temp_point_0[0], temp_point_0[1]])

            for i, temp_point_0 in enumerate(temp_points):
                flag = True
                for j, temp_point_1 in enumerate(temp_points):
                    if j == i:
                        continue
                    if temp_point_0[0] < temp_point_1[0]:
                        flag = False
                        break
                if flag:
                    target_points.append([temp_point_0[0], temp_point_0[1]])

            for target_point in target_points:
                # cv2.circle(image_cv,(target_point[0],target_point[1]),1,(0,255,0),4)
                rospy.loginfo("x:{}, y:{}".format(target_point[0],target_point[1]))
            
            rows, cols, _ = image_cv.shape
            p1 = np.float32([
                [target_points[0][0],target_points[0][1]], 
                [target_points[1][0],target_points[1][1]],
                [target_points[2][0],target_points[2][1]],
                [target_points[3][0],target_points[3][1]]
            ])
            p2 = np.float32([
                [400,400], 
                [600,400],
                [400,600],
                [600,600]
            ])
            M = cv2.getPerspectiveTransform(p1,p2)
            image_cv = cv2.warpPerspective(image_cv, M, (1000, 1000))

                

        # image_cv = cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)
        result = self._bridge.cv2_to_imgmsg(image_cv, encoding='bgr8')
        self._pub.publish(result)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('imager')
    imager = Imager()
    imager.main()
