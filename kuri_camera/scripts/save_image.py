#!/usr/bin/env python
# From https://github.com/KuriRobot/Kuri-Documentation/blob/master/reference/ros-packages/madmux.md
import threading

import madmux

done = threading.Event()


def cb(data):
    with open('camera_image.jpg', 'w') as f:
        f.write(data)
    done.set()


s = madmux.Stream("/var/run/madmux/ch3.sock")
s.register_cb(cb)
done.wait()
s.close()
