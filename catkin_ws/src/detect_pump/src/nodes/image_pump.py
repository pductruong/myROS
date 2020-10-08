#!/usr/bin/env python
import rospy
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def start_node():
    rospy.init_node('image_pub')
    rospy.loginfo('image_pub node started')
    filename = rospy.myargv(argv=sys.argv)[1]
    img = cv2.imread(filename)
    cv2.imshow("image",img)
    bridge = CvBridge()
    imgMsg = bridge.cv2_to_imgmsg(img,"bgr8")
    pub = rospy.Publisher('image',Image,queue_size=10)
    while not rospy.is_shutdown():
        pub.publish(imgMsg)
        rospy.Rate(1.0).sleep()
    
    
if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
