#!/usr/bin/env python

import numpy as np
import imutils
import cv2
import threading
import time

import rospy
from sensor_msgs.msg import CompressedImage
from kuri_person_tracking.msg import BoundingBox, BoundingBoxes

import os

DISPLAY = rospy.get_param('display') # is already a bool

# Change this based on your own setup
NET_CFG_DIR = '/home/nikitaf/workspaces/main_workspace/src/kuri/kuri_person_tracking/net_cfg'

net = None
output_layers = None
classes = None

CONF_THRESHOLD = 0.1
NMS_THRESHOLD = 0.3
COLORS = None
BLOB_SIZE = 416

hog = None
cascade_body = None
cascade_lower_bod = None
cascade_upper_body = None

bb_publisher = None

# Synchronization constants for the image
image_data = None
image_lock = threading.Lock()

def main():
    init_yolo()
    init_ros()


def init_yolo():
    global net, output_layers, classes, COLORS
    net = cv2.dnn.readNet(os.path.join(NET_CFG_DIR, 'yolov3-tiny.weights'), \
                          os.path.join(NET_CFG_DIR, 'yolov3-tiny.cfg'))

    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    # read class names from text file
    with open(os.path.join(NET_CFG_DIR, 'coco.names'), 'r') as f:
        classes = [line.strip() for line in f.readlines()]

    COLORS = np.random.randint(0, 255, size=(len(classes), 3),
    	dtype="uint8")


def init_ros():
    global bb_publisher, eyes_publisher

    """ Initialize ros node and subs/pubs
    """
    rospy.init_node('bounding_box_publisher')

    # _wait_for_time()

    process_image_thread = threading.Thread(target=process_image_loop)
    process_image_thread.start()

    rospy.Subscriber("/upward_looking_camera/compressed", CompressedImage, image_callback, queue_size=1)

    bb_publisher = rospy.Publisher("/upward_looking_camera/bounding_boxes", BoundingBoxes, queue_size=1)

    rospy.spin()


def image_callback(data):
    global image_data
    image_lock.acquire()
    image_data = data
    image_lock.release()


def process_image_loop():
    loop_hz = 20
    r = rospy.Rate(loop_hz)

    while not rospy.is_shutdown():
        # Get the image data
        image_lock.acquire()
        data = image_data
        image_lock.release()

        if data is not None:
            start_time = time.time()

            # uncompress compressed image
            image = cv2.imdecode(np.fromstring(data.data, np.uint8), 1)

            boxes, confidences, class_ids = detect_net(image)
            # print("len(boxes)", len(boxes))

            idxs = cv2.dnn.NMSBoxes(boxes, confidences, CONF_THRESHOLD, NMS_THRESHOLD)

            person_boxes = []
            # ensure at least one detection exists
            if len(idxs) > 0:
                # loop over the indexes we are keeping
                for i in idxs.flatten():
                    box_class = classes[class_ids[i]]
                    if box_class == 'person':
                        # extract the bounding box coordinates
                        (x, y) = (int(boxes[i][0]), int(boxes[i][1]))
                        (w, h) = (int(boxes[i][2]), int(boxes[i][3]))
                        box = (x, y, x + w, y + h, confidences[i])

                        person_boxes.append(box)

                        if DISPLAY:
                            # draw a bounding box rectangle and label on the image
                            color = [int(c) for c in COLORS[class_ids[i]]]
                            cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                            text = "{}: {:.4f}".format(classes[class_ids[i]], confidences[i])
                            cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, color, 2)

            # show the output image
            if DISPLAY:
                cv2.imshow("Image", image)
                cv2.waitKey(1)

            publish_bounding_box(person_boxes, image.shape[1], image.shape[2])

            # print("Person Detection Latency: ", time.time() - start_time)

        r.sleep()


def detect_net(image):
    global net, output_layers, classes, COLORS

    height, width = image.shape[:2]

    blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (BLOB_SIZE, BLOB_SIZE), swapRB=True, crop=False)
    net.setInput(blob)

    outs = net.forward(output_layers)

    class_ids = []
    confidences = []
    boxes = []

    for out in outs:
        # print("len(out)", len(out))
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            if classes[class_id] != "person":
                continue
            confidence = scores[class_id]

            if confidence > CONF_THRESHOLD:
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                x = center_x - w / 2
                y = center_y - h / 2
                class_ids.append(class_id)
                confidences.append(float(confidence))
                boxes.append([x, y, w, h])

    return boxes, confidences, class_ids


def publish_bounding_box(rects, width, height):
    global bb_publisher

    bbs = BoundingBoxes()
    boxes = []

    for xmin, ymin, xmax, ymax, confidence in rects:
        bb = BoundingBox()
        bb.xmin = xmin/width
        bb.ymin = ymin/height
        bb.xmax = xmax/width
        bb.ymax = ymax/height
        bb.probability = confidence
        boxes.append(bb)

    bbs.bounding_boxes = boxes
    # print(bbs)
    bb_publisher.publish(bbs)



def _wait_for_time():
    """ Wait for rospy to spool up
    """
    while rospy.Time().now().to_sec() == 0:
        time.sleep(0.1)


if __name__ == '__main__':
    main()
