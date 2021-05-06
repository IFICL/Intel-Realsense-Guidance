import argparse
import numpy as np
import cv2
import os
import pickle
import time
import ffmpeg

def read_from_pickel(bag_file):
    save_path = bag_file.split('.')[0] + '.pkl'
    pickle_in = open(save_path, "rb")
    data = pickle.load(pickle_in)
    pickle_in.close()
    return data


def create_video(data, video_name, fps):
    # video name should be ended with .avi
    sample = data[0]['color']
    H, W, C = sample.shape
    out = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*'DIVX'), fps, (W, H))
    for i in range(len(data)):
        out.write(data[i]['color'])
    out.release()

if __name__ == "__main__":
    fps = 15
    bag_file = 'test.bag'
    video_name = 'test.avi'
    data = read_from_pickel(bag_file)
    create_video(data, video_name, fps)
    time.sleep(3.0)
    