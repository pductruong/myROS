import math
import cv2
import numpy as np
from imutils.video import VideoStream
import imutils
from pyzbar import pyzbar
import re

BLUR_VALUE = 3
SQUARE_TOLERANCE = 0.15
AREA_TOLERANCE = 0.15
DISTANCE_TOLERANCE = 0.25
WARP_DIM = 300
SMALL_DIM = 29


def count_children(hierarchy, parent, inner=False):
    if parent == -1:
        return 0
    elif not inner:
        return count_children(hierarchy, hierarchy[parent][2], True)
    return 1 + count_children(hierarchy, hierarchy[parent][0], True) + count_children(hierarchy, hierarchy[parent][2], True)


def has_square_parent(hierarchy, squares, parent):
    if hierarchy[parent][3] == -1:
        return False
    if hierarchy[parent][3] in squares:
        return True
    return has_square_parent(hierarchy, squares, hierarchy[parent][3])


def get_center(c):
    m = cv2.moments(c)
    return [int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])]


def get_angle(p1, p2):
    x_diff = p2[0] - p1[0]
    y_diff = p2[1] - p1[1]
    return math.degrees(math.atan2(y_diff, x_diff))


def get_midpoint(p1, p2):
    return [(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2]


def get_farthest_points(contour, center):
    distances = []
    distances_to_points = {}
    for point in contour:
        point = point[0]
        d = math.hypot(point[0] - center[0], point[1] - center[1])
        distances.append(d)
        distances_to_points[d] = point
    distances = sorted(distances)
    return [distances_to_points[distances[-1]], distances_to_points[distances[-2]]]


def line_intersection(line1, line2):
    x_diff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    y_diff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(x_diff, y_diff)
    if div == 0:
        return [-1, -1]

    d = (det(*line1), det(*line2))
    x = det(d, x_diff) / div
    y = det(d, y_diff) / div
    return [int(x), int(y)]


def extend(a, b, length, int_represent=False):
    length_ab = math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
    if length_ab * length <= 0:
        return b
    result = [b[0] + (b[0] - a[0]) / length_ab * length, b[1] + (b[1] - a[1]) / length_ab * length]
    if int_represent:
        return [int(result[0]), int(result[1])]
    else:
        return result

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
# vs = VideoStream(src=0).start()
vs = VideoStream(src=0).start()

qrcode_dim = 0.2

while True:
    
    frame = vs.read()
    frame = imutils.resize(frame, width=600)
    
    size = frame.shape
    image_center_x = size[1]/2
    image_center_y = size[0]/2
    image_center = (image_center_x, image_center_y)
    
    output = frame.copy()
    
    Rx = float()
    Ry = float()
    Rz = float()
    point_O_x = float()
    point_O_y = float()
    point_X_x = float()
    point_X_y = float()
    point_Y_x = float()
    point_Y_y = float()
    
    # Remove noise and unnecessary contours from frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.bilateralFilter(gray, 11, 17, 17)
    gray = cv2.GaussianBlur(gray, (BLUR_VALUE, BLUR_VALUE), 0)
    edged = cv2.Canny(gray, 30, 200)

    #_, contours, hierarchy = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours, hierarchy = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    squares = []
    square_indices = []

    i = 0
    for c in contours:
        # Approximate the contour
        peri = cv2.arcLength(c, True)
        area = cv2.contourArea(c)
        approx = cv2.approxPolyDP(c, 0.03 * peri, True)

        # Find all quadrilateral contours
        if len(approx) == 4:
            # Determine if quadrilateral is a square to within SQUARE_TOLERANCE
            if area > 25 and 1 - SQUARE_TOLERANCE < math.fabs((peri / 4) ** 2) / area < 1 + SQUARE_TOLERANCE and count_children(hierarchy[0], i) >= 2 and has_square_parent(hierarchy[0], square_indices, i) is False:
                squares.append(approx)
                square_indices.append(i)
        i += 1

    #print("squares:", squares)
    
    
    main_corners = []
    east_corners = []
    south_corners = []
    
    # Determine if squares are QR codes
    for square in squares:
        area = cv2.contourArea(square)
        center = get_center(square)
        peri = cv2.arcLength(square, True)

        similar = []
        tiny = []
        for other in squares:
            if square[0][0][0] != other[0][0][0]:
                # Determine if square is similar to other square within AREA_TOLERANCE
                if math.fabs(area - cv2.contourArea(other)) / max(area, cv2.contourArea(other)) <= AREA_TOLERANCE:
                    similar.append(other)
                elif peri / 4 / 2 > cv2.arcLength(other, True) / 4:
                    tiny.append(other)

        if len(similar) >= 2:
            distances = []
            distances_to_contours = {}
            for sim in similar:
                sim_center = get_center(sim)
                d = math.hypot(sim_center[0] - center[0], sim_center[1] - center[1])
                distances.append(d)
                distances_to_contours[d] = sim
            distances = sorted(distances)
            closest_a = distances[-1]
            closest_b = distances[-2]

            # Determine if this square is the top left QR code indicator
            if max(closest_a, closest_b) < cv2.arcLength(square, True) * 2.5 and math.fabs(closest_a - closest_b) / max(closest_a, closest_b) <= DISTANCE_TOLERANCE:
                # Determine placement of other indicators (even if code is rotated)
                angle_a = get_angle(get_center(distances_to_contours[closest_a]), center)
                angle_b = get_angle(get_center(distances_to_contours[closest_b]), center)
                if angle_a < angle_b or (angle_b < -90 and angle_a > 0):
                    east = distances_to_contours[closest_a]
                    south = distances_to_contours[closest_b]
                else:
                    east = distances_to_contours[closest_b]
                    south = distances_to_contours[closest_a]
                
                c_o = get_center(square)
                point_O_x = c_o[0]; point_O_y = c_o[1]
                cv2.circle(output, (point_O_x, point_O_y), 5, (0, 255, 255), -1)
                cv2.putText(output, "O", (point_O_x, point_O_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                
                c_s = get_center(south)
                point_X_x = c_s[0]
                point_X_y = c_s[1] 
                cv2.circle(output, (point_X_x, point_X_y), 5, (0, 255, 255), -1)
                cv2.putText(output, "X", (point_X_x, point_X_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                
                c_e = get_center(east)
                point_Y_x = c_e[0]
                point_Y_y = c_e[1]              
                cv2.circle(output, (point_Y_x, point_Y_y), 5, (0, 255, 255), -1)
                cv2.putText(output, "Y", (point_Y_x, point_Y_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                
                east_corners.append(east)
                south_corners.append(south)
                main_corners.append(square)
                            
    
    # find the qrcodes in the frame and decode each of the qrcodes
    frame = output
    qrcodes = pyzbar.decode(frame)
    
    if (point_O_x != 0) and (point_O_y != 0) and(point_O_x != 0) and (point_O_y != 0):
        # loop over the detected qrcodes
        for qrcode in qrcodes:
    
            # extract the bounding box location of the qrcode
            (x, y, w, h) = qrcode.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
    
            # Decode and get information from qrcode
            qrcodeData = qrcode.data.decode("utf-8")
            qrcodeType = qrcode.type
    
            # draw the qrcode data and qrcode type on the image
            text = "{} ({})".format(qrcodeData, qrcodeType)
            cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
            # Get the corners of the qrcode
            points = qrcode.polygon

            # Number of points in the convex hull
            n = len(points)
    
            # Draw the convext hull and marking the points
            for j in range(0,n):
                cv2.line(frame, points[j], points[ (j+1) % n], (255,0,0), 2)
    
            # Finding 3D coordinates of qrcode
            indexes = [i.start() for i in re.finditer(",", qrcodeData)]
    
            Rx = qrcodeData[indexes[0]+1 : indexes[1]]
            Rx = float(Rx)
            Ry = qrcodeData[indexes[1]+1 : indexes[2]]
            Ry = float(Ry)
            Rz = qrcodeData[indexes[2]+1 : len(qrcodeData)]
            Rz = float(Rz)

            # Finding the coordinates x, y of the camera related to qrcode
            # the real distance OX of the qrcode
            R_D = 0.134
            # the distance OX of the qrcode in the image
            I_D = math.hypot((point_X_x - point_O_x), (point_X_y - point_O_y))
            
            # Finding the distance from the center of the QR-code to the position of image in the image
            image_distance = math.hypot((point_O_x - image_center_x), (point_O_y - image_center_y))
                
            # the distance from the center of the QR-code to the robot in the real-world domain
            real_distance = image_distance * R_D/I_D
            #print("Real Distance: ", real_distance)
            
            image_height = point_O_y - image_center_y
            image_base = point_O_x - image_center_x
            # the coordinates of the robot in the real-world domain
            if image_distance == 0:
                camera_pose_x = Rx
                camera_pose_y = Ry
            else:
                camera_pose_x = Rx - real_distance * image_base/image_distance
                camera_pose_y = Ry - real_distance * image_height/image_distance
        
            # Finding the theta angle
            delta_x = point_X_x - point_O_y
            delta_y = point_X_y - point_O_y
            print("delta_x, delta_y: ", delta_x, delta_y)

            if (delta_y == 0) and (delta_x >= 0):
                camera_pose_theta = math.pi/2
            elif (delta_y == 0) and (delta_x < 0):
                camera_pose_theta = - math.pi/2
            else:
                    camera_pose_theta = math.atan2(delta_x, delta_y)
        
            print("Camera Pose: x, y, theta: ", camera_pose_x, camera_pose_y, math.degrees(camera_pose_theta))


    cv2.drawContours(frame, main_corners, -1, (0, 0, 255), 2)
    cv2.drawContours(frame, east_corners, -1, (0, 255, 0), 2)
    cv2.drawContours(frame, south_corners, -1, (255, 0, 0), 2)
	
    cv2.circle(frame, image_center, 5, (0, 255, 255), -1)
    cv2.putText(frame, "C", image_center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
       
    cv2.imshow("Logitech", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print "I quit!"
        break

# close the output CSV file do a bit of cleanup
print("[INFO] cleaning up...")
cv2.destroyAllWindows()
vs.stop()
