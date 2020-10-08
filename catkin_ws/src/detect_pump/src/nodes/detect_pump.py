#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
# known pump geometry
#  - units are pixels (of half-size image)
PUMP_DIAMETER = 360
PISTON_DIAMETER = 90
PISTON_COUNT = 7

def start_node():
    rospy.init_node('detect_pump')
    rospy.loginfo('detect_pump node started')
    rospy.Subscriber("image", Image, process_image)
    rospy.spin()
def process_image(msg):
    try:
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        drawImg = orig
        showImage(drawImg)
    except Exception as err:
	    showImage(drawImg)
def showImage(img):
    cv2.imshow('image',img)
    cv2.waitKey(1)
    
if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass