import time
import argparse
import numpy as np
import sys
import os
from threading import Thread

import sounddevice as sd
import soundfile as sf
import pyrealsense2 as rs
import cv2
from scipy.io.wavfile import write

import pdb


parser = argparse.ArgumentParser(description="")

parser.add_argument('--filename', type=str, default='str', help='saved folder of .bag and .wav')
parser.add_argument('--seconds', type=int, default=0, help='total recorded seconds')


class soundThread(object): 
    def __init__(self, time0, total_seconds, sample_folder): 
        self.time0 = time0
        self.filename = os.path.join(sample_folder, 'sound.wav')
        self.total_seconds = total_seconds
        self.thread = Thread(target=self.record_sound)
        self.thread.start()
        self.end_sound = None

    def record_sound(self): 
        fs = 48000
        # sd.default.device = [1, 3]
        print('Start time of soound thread: ', time.time() - self.time0)
        myrecording = sd.rec(int(self.total_seconds * fs), samplerate=fs, channels=2, dtype='int16')
        sd.wait()
        write(self.filename, fs, myrecording)
        end_sound = time.time() - self.time0
        print('End time of soound thread: ', end_sound)
        if self.end_sound == None: 
            self.end_sound = end_sound


class videoThread(object): 
    def __init__(self, time0, total_seconds, sample_folder): 
        self.time0 = time0
        self.filename = os.path.join(sample_folder, 'video.bag')
        self.total_seconds = total_seconds
        self.images = None
        self.thread = Thread(target=self.record_video)
        self.thread.start()
        self.end_video = None
        
    def record_video(self):
        pipeline = rs.pipeline()
        config = rs.config()
        fps = 15
        config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, fps)
        config.enable_record_to_file(self.filename)
        # Start streaming sazxaas
        pipeline.start(config)
        e1 = cv2.getTickCount()
        print('Start time of video thread: ', time.time() - self.time0)
        count = 0
        try:
            while(True):
                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()
                frames.keep()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    pdb.set_trace()
                    continue
               
                count += 1
                e2 = cv2.getTickCount()
                t = (e2 - e1) / cv2.getTickFrequency()
                if t >= self.total_seconds:  # change it to record what length of video you are interested in
                    end_video = time.time() - self.time0
                    print('End time of video thread: ', end_video)
                    print('Total frames: ', count)
                    break

        finally:
            # Stop streaming
            pipeline.stop()
        
        if self.end_video == None: 
            self.end_video = end_video
        
    def show(self): 
        if self.images is not None: 
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', self.images)
            cv2.waitKey(1)


# Example: 
# python record_video_audio.py --filename=test --seconds=5

if __name__ == "__main__":
    args = parser.parse_args()
    # create folder
    sample_folder = os.path.join('./samples', args.filename)
    if not os.path.exists(sample_folder): 
        os.mkdir(sample_folder)
    
    t0 = time.time()
    video = videoThread(t0, args.seconds, sample_folder)
    sound = soundThread(t0, args.seconds, sample_folder)

    ## visualization of video, cannot record together with sound
    # while video.thread.is_alive(): 
    #     try: 
    #         video.show()
    #     except AttributeError: 
    #         pass   
    
    # compute time difference
    video.thread.join()
    sound.thread.join()
    end_sound = sound.end_sound
    end_video = video.end_video
    print('Time difference: ', end_sound - end_video)
    np.save(os.path.join(sample_folder, "td.npy"), np.asarray(end_sound - end_video))
