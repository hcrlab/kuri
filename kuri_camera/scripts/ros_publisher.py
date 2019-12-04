#!/usr/bin/env python
import madmux
import threading
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import rospy

rospy.init_node("kuri_camera_ros_publisher")

# Whether to publish the image msg as a CompressedImage or an Image
useCompression = True

# We'll treat this like reusable barriers
image_received = threading.Event()
image_published = threading.Event()

bridge = CvBridge()
base_topic = "/upward_looking_camera/image_raw"

if useCompression:
    publisher = rospy.Publisher(base_topic + "/compressed", CompressedImage, queue_size=10)
else:
    publisher = rospy.Publisher(base_topic, Image, queue_size=10)

image = None
def stream_cb(data):
  global image
  try:
    stamp = rospy.get_rostime()
    # Make sure the previous message was sent before we take in the new one
    image_published.wait()
    image_published.clear()

    # Read the bytes as a jpeg image
    data = np.fromstring(data, np.uint8)
    decoded = cv2.imdecode(data, cv2.CV_LOAD_IMAGE_COLOR)

    # Convert the decoded image to a ROS message
    if useCompression:
        image = bridge.cv2_to_compressed_imgmsg(decoded)
    else:
        image = bridge.cv2_to_imgmsg(decoded, "rgb8")

    image.header.stamp = stamp
    image_received.set()
  except Exception as e:
    print(e)

# The MJPEG channel
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

    # Let any pending stream callback threads know that they can overwrite the image.
    image_published.set()

s.close()
