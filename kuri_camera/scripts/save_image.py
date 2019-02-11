#!/usr/bin/env python
import madmux
import threading

done = threading.Event()


def cb(data):
    with open('camera_image.jpg', 'w') as f:
        f.write(data)
    done.set()


s = madmux.Stream("/var/run/madmux/ch3.sock")
s.register_cb(cb)
done.wait()
s.close()