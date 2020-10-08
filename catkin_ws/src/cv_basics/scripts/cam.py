
#!/usr/bin/env python3
from imutils.video import VideoStream
from imutils.video import FPS
import numpy as np
import argparse
import imutils
import pickle
import cv2
import os
import face_recognition
import time

embeddings = "/home/tucuman/OpenCV-Face-Recognition/Face recognition-20201006T025807Z-001/Face recognition/output/embeddings_1.pickle"
recognizer_path = "/home/tucuman/OpenCV-Face-Recognition/Face recognition-20201006T025807Z-001/Face recognition/output/recognizer.pickle"
le_path = "/home/tucuman/OpenCV-Face-Recognition/Face recognition-20201006T025807Z-001/Face recognition/output/le.pickle"
model = "/home/tucuman/OpenCV-Face-Recognition/Face recognition-20201006T025807Z-001/Face recognition/pyannote-data/openface.nn4.small2.v1.t7"
confi = 0.5
# load the known faces and embeddings
print("[INFO] loading encodings...")
data = pickle.loads(open(embeddings, "rb").read())
vs = VideoStream(src=0).start()
while True:
    # grab the current frame
    frame = vs.read()
    frame = imutils.resize(frame, width = 700)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    # detect the (x, y)-coordinates of the bounding boxes corresponding
    # to each face in the input image, then compute the facial embeddings
    # for each face
    print("[INFO] recognizing faces...")
    boxes = face_recognition.face_locations(rgb,
        model="cnn")
    encodings = face_recognition.face_encodings(rgb, boxes)

    embedder = cv2.dnn.readNetFromTorch(model)
    # load the actual face recognition model along with the label encoder
    recognizer = pickle.loads(open(recognizer_path, "rb").read())
    le = pickle.loads(open(le_path, "rb").read())
    # load the image, resize it to have a width of 600 pixels (while
    # maintaining the aspect ratio), and then grab the image dimensions
    #image = imutils.resize(image, width=600)
    (h, w) = frame.shape[:2]
    # construct a blob from the image
    '''imageBlob = cv2.dnn.blobFromImage(
        cv2.resize(image, (300, 300)), 1.0, (300, 300),
        (104.0, 177.0, 123.0), swapRB=False, crop=False)'''
    # apply OpenCV's deep learning-based face detector to localize
    # faces in the input image
    for ((startY, endX, endY, startX)) in (boxes):
    #cv2.rectangle(image, (left, top), (right, bottom), (0, 255, 0), 2)
            face = frame[startY:endY, startX:endX]
            (fH, fW) = face.shape[:2]
            # ensure the face width and height are sufficiently large
            if fW < 20 or fH < 20:
                continue
        # construct a blob for the face ROI, then pass the blob
            # through our face embedding model to obtain the 128-d
            # quantification of the face
            faceBlob = cv2.dnn.blobFromImage(face, 1.0 / 255, (96, 96),
                (0, 0, 0), swapRB=True, crop=False)
            embedder.setInput(faceBlob)
            vec = embedder.forward()
            # perform classification to recognize the face

            preds = recognizer.predict_proba(vec)[0]
            j = np.argmax(preds)
            proba = preds[j]
            if proba >= 0.7:
                    name = le.classes_[j]
                    text = "{}: {:.2f}%".format(name, proba * 100)
                    y = startY - 10 if startY - 10 > 10 else startY + 10
                    cv2.rectangle(frame, (startX, startY), (endX, endY),
                    (0, 0, 255), 2)
                    cv2.putText(frame, text, (startX, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
            else : 
                        name = "Unknow"
                        text = "{} ".format(name)
                        y = startY - 10 if startY - 10 > 10 else startY + 10
                        cv2.rectangle(frame, (startX, startY), (endX, endY),
                (0, 0, 255), 2)
                        cv2.putText(frame, text, (startX, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
    cv2.imshow("frame",frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
vs.stop()
