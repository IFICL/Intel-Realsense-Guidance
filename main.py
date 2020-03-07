import time

import sys
import sounddevice as sd
import soundfile as sf
import pyrealsense2 as rs
import cv2

import numpy as np
from scipy.io.wavfile import write

from threading import Thread

import pdb
# images = None
"""
summary:

    [
        {
            timestamp:
            depth:
            rgb:
            sound:
            others...
            sound start end
         },

    ]
"""

def record_sound(time0, total_seconds): 
    fs = 44100
    # sd.default.device = [12, 8]
    print('Start time of soound thread: ', time.time() - time0)
    myrecording = sd.rec(int(total_seconds * fs), samplerate=fs, channels=1, dtype='int16')
    sd.wait()
    write('output.wav', fs, myrecording)
    print('End time of soound thread: ', time.time() - time0)

class videoThread(object): 
    def __init__(self, time0, total_seconds): 
        self.time0 = time0
        self.total_seconds = total_seconds
        self.images = None
        self.thread = Thread(target=self.record_video)
        # sekf.threa.daemon = True
        self.thread.start()
        
    def record_video(self):
        pipeline = rs.pipeline()
        config = rs.config()
        fps = 15
        config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, fps)
        config.enable_record_to_file('test.bag')
        # Start streaming sazxaas
        colorizer = rs.colorizer()
        pipeline.start(config)

        e1 = cv2.getTickCount()
        # align_to = rs.stream.color
        # align = rs.align(align_to)

        print('Start time of video thread: ', time.time() - self.time0)
        

        count = 0
        try:
            while(True):

                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()
                # frames.keep()
                # if count == 0: 
                    # print('first wait:', time.time() - t1)
                # if count > 0: 
                    # print('wait:', time.time() - t1)
                # frames = align.process(frames)
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                # Convert images to numpy arrays
                # depth_color_frame = colorizer.colorize(depth_frame)
                # depth_color_image = np.asanyarray(depth_color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())


                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_color_image = cv2.applyColorMap(cv2.convertScaleAbs(
                    depth_image, alpha=0.03), cv2.COLORMAP_JET)

                # Stack both images horizontally
                self.images = np.hstack((color_image, depth_color_image))

                count += 1
                e2 = cv2.getTickCount()
                t = (e2 - e1) / cv2.getTickFrequency()
                if t >= self.total_seconds:  # change it to record what length of video you are interested in
                    # print("Done!")
                    print('End time of video thread: ', time.time() - self.time0)
                    print('Total frames: ', count)
                    break

        finally:
            # Stop streaming
            pipeline.stop()

    def show(self): 
        if self.images is not None: 
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', self.images)
            cv2.waitKey(1)


if __name__ == "__main__":
    total_seconds = 10
    t0 = time.time()
    video = videoThread(t0, total_seconds)
    # sound = Thread(target = record_sound, args=(t0, total_seconds))
    # sound.start() 
    while video.thread.is_alive(): 
        try: 
            video.show()
        except AttributeError: 
            pass
    
    
