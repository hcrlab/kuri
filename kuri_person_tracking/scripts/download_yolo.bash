#!/bin/bash

# Pass full as an arg to also download the full yolov3 cfg/weights as opposed to just tiny

full=$1
if [[ $full == "full" ]]; then
    wget -P net_cfg https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/yolov3.cfg
    wget -P net_cfg https://pjreddie.com/media/files/yolov3.weights
fi

wget -P net_cfg https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/yolov3-tiny.cfg
wget -P net_cfg https://pjreddie.com/media/files/yolov3-tiny.weights

wget -P net_cfg https://raw.githubusercontent.com/pjreddie/darknet/master/data/coco.names
wget -P net_cfg https://raw.githubusercontent.com/pjreddie/darknet/master/data/voc.names
