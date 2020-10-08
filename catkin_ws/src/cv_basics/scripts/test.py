#!/usr/bin/env python3
from imutils.video import VideoStream
import numpy as np
import argparse
import imutils
import pickle
import cv2
import os
import face_recognition
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
    
def face_recog():
    embeddings = "/home/tucuman/OpenCV-Face-Recognition/Face recognition-20201006T025807Z-001/Face recognition/output/embeddings_1.pickle"
    recognizer_path = "/home/tucuman/OpenCV-Face-Recognition/Face recognition-20201006T025807Z-001/Face recognition/output/recognizer.pickle"
    le_path = "/home/tucuman/OpenCV-Face-Recognition/Face recognition-20201006T025807Z-001/Face recognition/output/le.pickle"
    model = "/home/tucuman/OpenCV-Face-Recognition/Face recognition-20201006T025807Z-001/Face recognition/pyannote-data/openface.nn4.small2.v1.t7"

    pub = rospy.Publisher('video_frames',Image,queue_size = 10)
    rospy.init_node('frame_pub',anonymous=True)
    rate = rospy.Rate(10)
    vs = VideoStream(src=0).start()
    br = CvBridge()
    while not rospy.is_shutdown():
        frame = vs.read()
        frame = imutils.resize(frame, width = 500)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)            
        rospy.loginfo("[INFO] recognizing faces...")
        boxes = face_recognition.face_locations(rgb,model="cnn")
        encodings = face_recognition.face_encodings(rgb, boxes)
        embedder = cv2.dnn.readNetFromTorch(model)
        # load the actual face recognition model along with the label encoder
        recognizer = pickle.loads(open(recognizer_path, "rb").read())
        le = pickle.loads(open(le_path, "rb").read())
        # load the image, resize it to have a width of 600 pixels (while
        # maintaining the aspect ratio), and then grab the image dimensions
        #image = imutils.resize(image, width=600)
        (h, w) = frame.shape[:2]
        for ((startY, endX, endY, startX)) in (boxes):
            face = frame[startY:endY, startX:endX]
            (fH, fW) = face.shape[:2]
            if fW < 20 or fH < 20:
                continue
            faceBlob = cv2.dnn.blobFromImage(face, 1.0 / 255, (96, 96), (0, 0, 0), swapRB=True, crop=False)
            embedder.setInput(faceBlob)
            vec = embedder.forward()
            preds = recognizer.predict_proba(vec)[0]
            j = np.argmax(preds)
            proba = preds[j]
            if proba >= 0.7:
                name = le.classes_[j]
                text = "{}: {:.2f}%".format(name, proba * 100)
                y = startY - 10 if startY - 10 > 10 else startY + 10
                cv2.rectangle(frame, (startX, startY), (endX, endY),(0, 0, 255), 2)
                cv2.putText(frame, text, (startX, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
            else:
                name = "Unknow"
                text = "{} ".format(name)
                y = startY - 10 if startY - 10 > 10 else startY + 10
                cv2.rectangle(frame, (startX, startY), (endX, endY),(0, 0, 255), 2)
                cv2.putText(frame, text, (startX, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
        cv2.imshow("frame",frame)
        pub.publish(br.cv2_to_imgmsg(frame))
    rate.sleep()

if __name__ == '__main__':
	try:
		face_recog()
	except rospy.ROSInterruptException:
		pass