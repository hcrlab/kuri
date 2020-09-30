#!/usr/bin/env python

from __future__ import print_function
import numpy as np
import imutils
import cv2

from scipy import stats

import rospy
from sensor_msgs.msg import CompressedImage
from simple_person_detection.msg import BoundingBox, BoundingBoxes

import os 

DISPLAY = rospy.get_param('display', 'true') == 'true'

net = None
output_layers = None
classes = None
hog = None
cascade_body = None
cascade_lower_bod = None
cascade_upper_body = None

bb_publisher = None

""" This is just a misc script to verify the functionality of various yolo configurations
"""
def main():
    # init_classifiers()
    # init_yolo()

    net = cv2.dnn.readNet('net_cfg/yolov3-tiny.weights', 'net_cfg/yolov3-tiny.cfg')
    
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    # read class names from text file
    with open('net_cfg/coco.names', 'r') as f:
        LABELS = [line.strip() for line in f.readlines()]
    
    np.random.seed(42)
    COLORS = np.random.randint(0, 255, size=(len(LABELS), 3),
    	dtype="uint8")


    image = cv2.imread('images/1*EYFejGUjvjPcc4PZTwoufw.jpeg')
    (H, W) = image.shape[:2]

    
    blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    boxes = []
    confidences = []
    classIDs = []

    # loop over each of the layer outputs
    for output in outs:
        # loop over each of the detections
        for detection in output:
            # extract the class ID and confidence (i.e., probability) of
            # the current object detection
            scores = detection[5:]
            classID = np.argmax(scores)
            confidence = scores[classID]
            # filter out weak predictions by ensuring the detected
            # probability is greater than the minimum probability
            if confidence > 0.5:
                # scale the bounding box coordinates back relative to the
                # size of the image, keeping in mind that YOLO actually
                # returns the center (x, y)-coordinates of the bounding
                # box followed by the boxes' width and height
                box = detection[0:4] * np.array([W, H, W, H])
                (centerX, centerY, width, height) = box.astype("int")
                # use the center (x, y)-coordinates to derive the top and
                # and left corner of the bounding box
                x = int(centerX - (width / 2))
                y = int(centerY - (height / 2))
                # update our list of bounding box coordinates, confidences,
                # and class IDs
                boxes.append([x, y, int(width), int(height)])
                confidences.append(float(confidence))
                classIDs.append(classID)

    idxs = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.3)
	

    # ensure at least one detection exists
    if len(idxs) > 0:
        # loop over the indexes we are keeping
        for i in idxs.flatten():
            # extract the bounding box coordinates
            (x, y) = (boxes[i][0], boxes[i][1])
            (w, h) = (boxes[i][2], boxes[i][3])
            # draw a bounding box rectangle and label on the image
            color = [int(c) for c in COLORS[classIDs[i]]]
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
            text = "{}: {:.4f}".format(LABELS[classIDs[i]], confidences[i])
            cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, color, 2)
    # show the output image
    cv2.imshow("Image", image)
    cv2.waitKey(0)


def init_yolo():
    global net, output_layers, classes
    net = cv2.dnn.readNet('net_cfg/yolov3-tiny.weights', 'net_cfg/yolov3-tiny.cfg')
    
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    # read class names from text file
    with open('net_cfg/coco.names', 'r') as f:
        classes = [line.strip() for line in f.readlines()]



def init_ros():
    """ Initialize ros node and subs/pubs
    """
    global bb_publisher

    rospy.init_node('bounding_box_publisher')

    _wait_for_time()

    rospy.Subscriber("/upward_looking_camera/compressed", CompressedImage, image_callback, queue_size=1)

    bb_publisher = rospy.Publisher("/upward_looking_camera/bounding_boxes", BoundingBoxes, queue_size=1)

    rospy.spin()


def init_classifiers():
    """ Initialize the HOG descriptor/person detector
    """
    global hog, cascade_body, cascade_lower_body, cascade_upper_body

    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    cascade_body = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_fullbody.xml')
    cascade_lower_body = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_lowerbody.xml')
    cascade_upper_body = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_upperbody.xml')


def image_callback(data):
    global hog

    # uncompress compressed image
    image = cv2.imdecode( np.fromstring(data.data, np.uint8), 1 )
    # original_shape = image.shape[:2]

    # load the image and resize it to (1) reduce detection time
    # and (2) improve detection accuracy
    # image = imutils.resize( image, width=min(400, image.shape[1]) )
    # scaled_shape = image.shape[:2]
    
    # detect people in the image
    rects = detect_net(image)

    rects = np.array([ [x, y, x + w, y + h] for (x, y, w, h) in rects ])

    # draw the original bounding boxes
    print('rects')
    print(rects)
    for (x, y, xmax, ymax) in rects:
        cv2.rectangle(image, (int(x), int(y)), (int(xmax), int(ymax)), (0, 0, 255), 2)

    # rescale bounding boxes to the original image scale
    scale = np.flipud( np.divide(original_shape, scaled_shape) )

    # def rescale_rect(rect):
    #     bottom_left = np.multiply(rect[:2], scale)
    #     top_right = np.multiply(rect[2:4], scale)
    #     return [bottom_left[0], bottom_left[1], top_right[0], top_right[1]]

    # rects = list(map(rescale_rect, rects))

    # publish_bounding_box(rects)

    if DISPLAY:
        cv2.imshow('image', image)
        cv2.waitKey(1)


def detect_net(image):
    global net, output_layers

    print(image.shape)

    height = image.shape[0]
    width = image.shape[1]

    scale = 1
    blob = cv2.dnn.blobFromImage(image, 1.0 / 255, (416,416), swapRB=True, crop=False)

    net.setInput(blob)

    outs = net.forward(output_layers)

    class_ids = []
    confidences = []
    boxes = []
    conf_threshold = 0.5
    nms_threshold = 0.4

    # for each detection from each output layer get the confidence, class id, 
    # bounding box params and ignore weak detections (confidence < 0.5)
    print("outs len", len(outs))
    for out in outs:
        print("out len", len(out))
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > 0.5:
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                x = center_x - w / 2
                y = center_y - h / 2
                class_ids.append(class_id)
                confidences.append(float(confidence))
                boxes.append([x, y, w, h])

    print(boxes)
    print(class_ids)
    print(confidences)
    # for i in range(len(boxes)):
    #     print(classes[class_ids[i]])

    # apply non-max suppression
    # indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)
    
    # filter out non-person objects
    return [boxes[i] for i in range(len(boxes)) if classes[class_ids[i]] == 'person']
    # return filter(lambda i: classes[class_ids[i[0]]] == 'person', boxes)

    # return [boxes[i] for i in person_indices]


def detect_hog(image):
    global hog

    return hog.detectMultiScale(image, winStride=(4, 4),
                                padding=(8, 8), scale=1.1)


def detect_full_body(image):
    global cascade_body

    return cascade_body.detectMultiScale(image, scaleFactor=1.1, minNeighbors=3)


def detect_lower_body(image):
    global cascade_lower_body

    return cascade_lower_body.detectMultiScale(image, scaleFactor=1.1, minNeighbors=3)


def detect_upper_body(image):
    global cascade_upper_body

    return cascade_upper_body.detectMultiScale(image, scaleFactor=1.1, minNeighbors=3)


def publish_bounding_box(rects):
    global bb_publisher

    bbs = BoundingBoxes()
    boxes = []

    for xmin, ymin, xmax, ymax in rects:
        bb = BoundingBox()
        bb.xmin = round(xmin)
        bb.ymin = round(ymin)
        bb.xmax = round(xmax)
        bb.ymax = round(ymax)
        boxes.append(bb)

    bbs.bounding_boxes = boxes
    print(bbs)
    bb_publisher.publish(bbs)


def _wait_for_time():
    """ Wait for rospy to spool up
    """
    while rospy.Time().now().to_sec() == 0:
        pass


if __name__ == '__main__':
    main()
