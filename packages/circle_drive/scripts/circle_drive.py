#!/usr/bin/env python
import os
import rospy
import cv2
import numpy as np
import sys
import threading
from duckietown import DTROS
from duckietown_msgs.msg import Twist2DStamped
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, Joy
from duckietown_msgs.msg import ObstacleImageDetection, ObstacleImageDetectionList, ObstacleType, Rect, BoolStamped
from std_msgs.msg import String
import Stem_graph as sg


# class MyNode(DTROS):
#
#     def __init__(self, node_name):
#         super(MyNode, self).__init__(node_name=node_name)
#         self.pub = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
#
#     def run(self):
#         # publish message every 1 second
#         rate = rospy.Rate(0.5) # 1Hz
#         while not rospy.is_shutdown():
#             msg = Twist2DStamped()
#             msg.v = 0.0
#             msg.omega = 0.5
#             rospy.loginfo("Publishing message")
#             self.pub.publish(msg)
#             rate.sleep()
#             msg.omega = 0.0
#             rospy.loginfo("Publishing message -")
#             self.pub.publish(msg)
#             rate.sleep()

thread_lock = threading.Lock()

rospy.init_node('duck_detector_node')
pub_image = rospy.Publisher("~cone_detection_image", Image, queue_size=1)
pub_stop = rospy.Publisher("/duckpi4/joy", Joy, queue_size=1)
pub_route = rospy.Publisher("/route", String, queue_size=1)
pub_move = rospy.Publisher("/duckpi4/coordinator_node/car_cmd", Twist2DStamped, queue_size=1)
bridge = CvBridge()

# CONE = [np.array(x, np.uint8) for x in [[0, 80, 80], [22, 255, 255]]]
DUCK = [np.array(x, np.uint8) for x in [[25, 100, 150], [35, 255, 255]]]
terms = {ObstacleType.CONE :"cone", ObstacleType.DUCKIE:"duck"}

stop_time = -1
stopped = False

# initializing graph
Rmap,R1,r1,R2,r2,R3,r3,R4,r4,R5,r5,R6,r6 = sg.init_lib()
# print(sp.find_route(R3, r1.id))
cur_road = R1
x = -1.0
y = -1.0


def get_filtered_contours(img, contour_type):
    # rospy.loginfo("get_filtered_contours")
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    if contour_type == "CONE":
        frame_threshed = cv2.inRange(hsv_img, CONE[0], CONE[1])
        ret, thresh = cv2.threshold(frame_threshed, 22, 255, 0)
    elif contour_type == "DUCK_COLOR":
        frame_threshed = cv2.inRange(hsv_img, DUCK[0], DUCK[1])
        ret, thresh = cv2.threshold(frame_threshed, 30, 255, 0)
    elif contour_type == "DUCK_CANNY":
        frame_threshed = cv2.inRange(hsv_img, DUCK[0], DUCK[1])
        frame_threshed = cv2.adaptiveThreshold(frame_threshed, 255,
                                               cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 5, 2)
        thresh = cv2.Canny(frame_threshed, 100, 200)
    else:
        return

    filtered_contours = []

    smth, contours, hierarchy = cv2.findContours(
        thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    contour_area = [(cv2.contourArea(c), (c)) for c in contours]
    contour_area = sorted(contour_area, reverse=True, key=lambda x: x[0])

    height, width = img.shape[:2]
    for (area, (cnt)) in contour_area:
        # plot box around contour
        x, y, w, h = cv2.boundingRect(cnt)
        if x < 7*width/12:
            continue
        box = (x, y, w, h)
        d = 0.5 * (x - width / 2) ** 2 + (y - height) ** 2
        if not (h > 15 and w > 10 and h < 200 and w < 200 and d < 120000):
            continue
        if contour_type == "DUCK_CANNY":
            continue
        if contour_type == "DUCK_COLOR":  # extra filtering to remove lines
            if not (h > 25 and w > 25):
                continue
            if d > 90000:
                if not (h > 35 and w > 35):
                    continue
            if cv2.contourArea(cnt) == 0:
                continue
            val = cv2.arcLength(cnt, True) ** 2 / cv2.contourArea(cnt)
            if val > 35: continue
            rect = cv2.minAreaRect(cnt)
            ctr, sides, deg = rect
            val = 0.5 * cv2.arcLength(cnt, True) / (w ** 2 + h ** 2) ** 0.5
            if val < 1.12: continue
            # if area > 1000: continue

        rospy.loginfo('duck shape: ' + str(w) + " " + str(h))

        mask = np.zeros(thresh.shape, np.uint8)
        cv2.drawContours(mask, [cnt], 0, 255, -1)
        mean_val = cv2.mean(img, mask=mask)
        aspect_ratio = float(w) / h
        filtered_contours.append((cnt, box, d, aspect_ratio, mean_val))
    rospy.loginfo("number of ducks: " + str(len(filtered_contours)))
    global stopped, x, y
    if len(filtered_contours) > 0:
        rospy.loginfo("duck detected, stopping")
        axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        buttons[6] = 1
        msg = Joy(header=None, axes=axes, buttons=buttons)
        # msg.axes[1] = -1
        pub_stop.publish(msg)
        rospy.sleep(0.5)

        # msg = Joy()
        # msg.header.seq = 0
        # msg.header.stamp.secs = 0
        # msg.header.stamp.nsecs = 0
        # msg.header.frame_id = ''
        # msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        # msg.buttons[6] = 1
        # pub_stop.publish(msg)

        # msg = Twist2DStamped()
        # msg.v = 0.0
        # msg.omega = 0.0
        # pub_move.publish(msg)

        stopped = True
        if x != -1.0:
            road = sg.det_points_road(x, y, Rmap)
            if road != 'Not on the road':
                route = sg.find_route(cur_road, road.id)
                s = ""
                for i in range(len(route)):
                    s += route[i]
                pub_route.publish(s)
                msg = Joy()
                msg.header.seq = 0
                msg.header.stamp.secs = 0
                msg.header.stamp.nsecs = 0
                msg.header.frame_id = ''
                msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                msg.buttons[6] = 1
                msg.axes[1] = -1
                pub_stop.publish(msg)
            else:
                rospy.loginfo("WARNING: requested dot is not on the road")

    return filtered_contours


def contour_match(img):

    # rospy.loginfo("contour_match")

    object_list = ObstacleImageDetectionList()
    object_list.list = []

    height, width = img.shape[:2]
    object_list.imwidth = width
    object_list.imheight = height

    # get filtered contours
    # cone_contours = get_filtered_contours(img, "CONE")
    duck_contours = get_filtered_contours(img, "DUCK_COLOR")

    all_contours = [duck_contours]
    for i, contours in enumerate(all_contours):
        for (cnt, box, ds, aspect_ratio, mean_color) in contours:
            # plot box around contour
            x, y, w, h = box
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img, terms[i], (x, y), font, 0.5, mean_color, 4)
            cv2.rectangle(img, (x, y), (x + w, y + h), mean_color, 2)

            r = Rect()
            r.x = x
            r.y = y
            r.w = w
            r.h = h

            t = ObstacleType()
            t.type = i

            d = ObstacleImageDetection()
            d.bounding_box = r
            d.type = t
            object_list.list.append(d)
    # rospy.loginfo("duck_contours" + str(len(duck_contours)))
    # rospy.loginfo("object_list" + str(len(object_list.list)))

    return img, object_list


def makeImageWithCones(img):
    # rospy.loginfo("makeImageWithCones")

    thread = threading.Thread(target=processImage, args=(img,))
    thread.setDaemon(True)
    thread.start()


def processImage(img):
    # if not thread_lock.acquire(False):
    #     return
    # rospy.loginfo("processImage")
    try:
        image_cv = bridge.imgmsg_to_cv2(img, "bgr8")
    except CvBridgeError as e:
        rospy.loginfo(e)
    img, detections = contour_match(image_cv)
    # detections.header.stamp = image_msg.header.stamp
    # detections.header.frame_id = image_msg.header.frame_id
    # self.pub_detections_list.publish(detections)
    # height, width = img.shape[:2]
    try:
        pub_image.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
    except CvBridgeError as e:
        rospy.loginfo(e)


def makeTurn(string):
    string = string.data
    rospy.loginfo("making turn: " + string)
    num = int(string)
    global cur_road
    if num == 1:
        cur_road = cur_road.Road1
    else:
        cur_road = cur_road.Road2

    rospy.loginfo("current road id: " + str(cur_road.id))


def getCoord(string):
    string = string.data
    s1, s2 = string.split(" ")
    global x, y
    x = float(s1)
    y = float(s2)
    rospy.loginfo("got dot")
    if stopped:
        road = sg.det_points_road(x, y, Rmap)
        if road != 'Not on the road':
            route = sg.find_route(cur_road, road.id)
            s = ""
            for i in range(route):
                s += route[i]
            pub_route.publish(s)
        else:
            rospy.loginfo("WARNING: requested dot is not on the road")


# if __name__ == '__main__':
    # # create the node
    # node = MyNode(node_name='circle_drive_node')
    # # run node
    # node.run()
    # # keep spinning
    # rospy.spin()


sub_image = rospy.Subscriber("/duckpi4/camera_node/image/raw", Image, makeImageWithCones, queue_size=1)
sub_turn = rospy.Subscriber("/turn", String, makeTurn, queue_size=5)
sub_coord = rospy.Subscriber("/coord", String, getCoord, queue_size=1)

rospy.spin()

