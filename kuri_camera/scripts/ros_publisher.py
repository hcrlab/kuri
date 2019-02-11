#!/usr/bin/env python
import madmux
import threading
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy

rospy.init_node("kuri_camera_ros_publisher")

# We'll treat this like reusable barriers
image_received = threading.Event()
image_published = threading.Event()

bridge = CvBridge()
publisher = rospy.Publisher("/upward_looking_camera/image_raw", Image, queue_size=10)

image = None
def stream_cb(data):
    global image
    # Make sure the previous message was sent before we take in the new one
    image_published.wait()
    image_published.clear()
    #rospy.logdebug("Got image")
    data = np.fromstring(data, np.uint8)
    decoded = cv2.imdecode(data, cv2.CV_LOAD_IMAGE_COLOR)
    #cv2.cvtColor(decoded, image, cv2.CV_RGB2)
    image = bridge.cv2_to_imgmsg(decoded, "rgb8")
    image_received.set()


s = madmux.Stream("/var/run/madmux/ch3.sock")
s.register_cb(stream_cb)

poll_subscribers = rospy.Rate(1)

image_published.set()
while not rospy.is_shutdown():
    if publisher.get_num_connections() == 0:
        rospy.loginfo_throttle(10,"Waiting for subscribers...")
        poll_subscribers.sleep()
        continue

    # Make sure we've got a message to send
    image_received.wait()
    image_received.clear()
    publisher.publish(image)
    #rospy.logdebug("Image published")

    # Let any pending stream callback threads know that they can scribble over image.
    image_published.set()

s.close()
