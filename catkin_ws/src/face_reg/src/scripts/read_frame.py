import rospy
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
def start_node():
    rospy.init_node('image_sub')
    rospy.loginfo('image_sub node started')
    rospy.SubscribeListener("/usb_cam/image_raw",Image,process_image)
    rospy.spin()

def process_image():
    try:
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        
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
