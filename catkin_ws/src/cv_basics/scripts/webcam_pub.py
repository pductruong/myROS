#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from imutils.video import VideoStream
import imutils

def publish_message():
	ret = True	
	pub = rospy.Publisher('video_frames',Image,queue_size = 10)
	rospy.init_node('video_pub',anonymous=True)
	rate = rospy.Rate(10)
	vs = VideoStream(src=0).start()
	br = CvBridge()
	while not rospy.is_shutdown():
		frame = vs.read()
		if ret == True:

			rospy.loginfo('publish video frame')
			pub.publish(br.cv2_to_imgmsg(frame))
		rate.sleep()

if __name__ == '__main__':
	try:
		publish_message()
	except rospy.ROSInterruptException:
		pass
