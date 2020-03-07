import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
# Import argparse for command-line options
import argparse
# Import os.path for file path manipulation
import os.path
import rosbag

if __name__ == "__main__":
    depth = np.array([])
    color = np.array([])
    filename = 'test.bag'
    bag = rosbag.Bag(filename)
    bagContents = bag.read_messages()
    # print(bag.get_type_and_topic_info())
    print('Depth:', bag.get_message_count('/device_0/sensor_0/Depth_0/image/data'))
    print('RGB:', bag.get_message_count('/device_0/sensor_1/Color_0/image/data'))
    for topic, msg, timestamp in bag.read_messages():
        if topic == '/device_0/sensor_0/Depth_0/image/data':
            timestr = "%.9f" % msg.header.stamp.to_sec()
            depth = np.append(depth, timestr)
        elif topic == '/device_0/sensor_1/Color_0/image/data':
            timestr = "%.9f" % msg.header.stamp.to_sec()
            color = np.append(color, timestr)
    color = np.append(color, 'timestr')
    color = np.append(color, 'timestr')
    res = depth == color
    print(depth)
    print(color)
    print(res)




