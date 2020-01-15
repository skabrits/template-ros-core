#!/usr/bin/env python
import os
import rospy
from duckietown import DTROS
from duckietown_msgs.msg import Twist2DStamped
from cv_bridge import CvBridge, CvBridgeError


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

pub_image = rospy.Publisher("~cone_detection_image", Image, queue_size=1)


def makeImageWithCones(img):
    thread = threading.Thread(target=processImage, args=(img,))
    thread.setDaemon(True)
    thread.start()


def processImage(img):
    if not thread_lock.acquire(False):
        return
    try:
        image_cv = bridge.imgmsg_to_cv2(img, "bgr8")
    except CvBridgeErrer as e:
        print(e)
    img, detections = self.tm.contour_match(image_cv)
    detections.header.stamp = image_msg.header.stamp
    detections.header.frame_id = image_msg.header.frame_id
    self.pub_detections_list.publish(detections)
    height, width = img.shape[:2]
    try:
        pub_image.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
    except CvBridgeError as e:
        print(e)

    self.thread_lock.release()


# if __name__ == '__main__':
    # # create the node
    # node = MyNode(node_name='circle_drive_node')
    # # run node
    # node.run()
    # # keep spinning
    # rospy.spin()
sub_image = rospy.Subscriber("~image_raw", Image, makeImageWithCones, queue_size=1)



