#!/usr/bin/env python
import madmux
import threading
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import rospy

isCompressed = True

rospy.init_node("kuri_camera_ros_publisher")

bridge = CvBridge()
publisher = rospy.Publisher("/upward_looking_camera/image_raw" + ("/compressed" if isCompressed else ""), CompressedImage if isCompressed else Image, queue_size=10)

imageRaw = None
newImageRaw = False
imageRawMutex = threading.Lock()

def stream_cb(data):
    global imageRaw, newImageRaw
    imageRawMutex.acquire()
    imageRaw = data
    newImageRaw = True
    imageRawMutex.release()

s = madmux.Stream("/var/run/madmux/ch3.sock")
s.register_cb(stream_cb)

numFrames = 0
timeAtStart = rospy.get_rostime()
r = rospy.Rate(10)
while not rospy.is_shutdown():
    imageRawMutex.acquire()
    if (newImageRaw):
        data = np.fromstring(imageRaw, np.uint8)
        newImageRaw = False
        imageRawMutex.release()
        decoded = cv2.imdecode(data, cv2.CV_LOAD_IMAGE_COLOR)
        if isCompressed:
            imageMsg = bridge.cv2_to_compressed_imgmsg(decoded)
        else:
            imageMsg = bridge.cv2_to_imgmsg(decoded, "rgb8")
        imageMsg.header.stamp = rospy.Time.now()
        publisher.publish(imageMsg)

        timeInterval = rospy.get_rostime() - timeAtStart
        timeIntervalFloat =  timeInterval.secs + timeInterval.nsecs/10.0**9.0
        numFrames += 1
        rospy.loginfo("got data, FPS %f", numFrames / timeIntervalFloat)
    else:
        imageRawMutex.release()
    r.sleep()



s.close()
