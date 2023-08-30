#!/usr/bin/env python3

# import libraries
import cv2 as cv
import numpy as np
import rospy
from std_msgs.msg import Int32MultiArray

cv.setUseOptimized(False)

# Load YOLO model
net = cv.dnn.readNetFromDarknet('/home/woojin/moon detection weight/ver2/yolov3.cfg', '/home/woojin/moon detection weight/ver2/yolov3_last.weights')
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

# Video capture
cap = cv.VideoCapture(0)

# Font, color, and scale for text
font = cv.FONT_HERSHEY_SIMPLEX
font_scale = 0.5
font_color = (0, 0, 0)
line_type = 1

# Draw coordinates on the frame
def draw_coordinates(frame):
    height, width = frame.shape[:2]

    # Draw x coordinates
    for x in range(0, width, 50):  
        cv.putText(frame, str(x), (x, 20), font, font_scale, font_color, line_type)

    # Draw y coordinates
    for y in range(0, height, 50):  
        cv.putText(frame, str(y), (10, y), font, font_scale, font_color, line_type)

    return frame

# Publish bounding box coordinates to a ROS topic
def publish_coordinates(x, y):
    pub = rospy.Publisher('bounding_box_coordinates', Int32MultiArray, queue_size=10)
    rospy.init_node('bounding_box_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        coordinates = Int32MultiArray()
        coordinates.data = [x, y]
        pub.publish(coordinates)
        rate.sleep()

while True:
    # Read frame from the camera
    ret, frame = cap.read()

    # Preprocessing for YOLO
    blob = cv.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    # Object detection using YOLO
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > 0.5 and class_id == 0:  # Class 0 corresponds to 'moon'
                center_x = int(detection[0] * frame.shape[1])
                center_y = int(detection[1] * frame.shape[0])
                w = int(detection[2] * frame.shape[1])
                h = int(detection[3] * frame.shape[0])
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                cv.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                label = f'Moon: {confidence:.2f}'
                cv.putText(frame, label, (x, y - 10), font, 0.9, (0, 0, 255), 2)

                center_x = int(x + w / 2)
                center_y = int(y + h / 2)
                cv.putText(frame, f'Center X: {center_x}', (10, 30), font, 0.9, (0, 0, 255), 2)
                cv.putText(frame, f'Center Y: {center_y}', (10, 70), font, 0.9, (0, 0, 255), 2)

                publish_coordinates(center_x, center_y)

    frame = draw_coordinates(frame)

    cv.imshow("Moon Detection", frame)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
