import time
import argparse
import numpy as np
import sys
import os
from threading import Thread
import multiprocessing
import glob

import sounddevice as sd
import soundfile as sf
import pyrealsense2 as rs
import cv2
import json
import pdb


parser = argparse.ArgumentParser(description="")

parser.add_argument('--building', type=str, default='str', help='saved folder of .bag and .wav')
parser.add_argument('--scene_id', type=int, default=0, help='saved folder of .bag and .wav')
parser.add_argument('--seconds', type=float, default=10.0, help='total recorded seconds')
parser.add_argument('--overwrite_previous', default=False, action='store_true', help='whether overwrite previous one')



class soundThread(object): 
    def __init__(self, time0, total_seconds, sample_folder): 
        self.time0 = time0
        self.filename = os.path.join(sample_folder, 'sound.wav')
        self.total_seconds = total_seconds
        # self.thread = multiprocessing.Process(target=self.record_sound)
        self.fs = 48000
        self.start_time = None
        self.end_time = None
        self.thread = Thread(target=self.record_sound)
        # self.thread.start()
        
        
    def record_sound(self): 
        
        myrecording = sd.rec(int(self.total_seconds * self.fs), samplerate=self.fs, channels=2, dtype='float64')
        self.start_time = time.time() - self.time0
        print('Start time of sound thread: ', self.start_time)
        sd.wait()
        end_time = time.time() - self.time0
        sf.write(self.filename, myrecording, self.fs)
        # write(self.filename, fs, myrecording)
        
        print('End time of sound thread: ', end_time)
        if self.end_time == None: 
            self.end_time = end_time


class videoThread(object): 
    def __init__(self, time0, total_seconds, sample_folder): 
        self.time0 = time0
        self.filename = os.path.join(sample_folder, 'video.bag')
        self.fps = 6
        self.total_seconds = total_seconds
        self.pipeline, self.profile = self.init_depth_camera()
        self.images = None
        self.start_time = None
        self.end_time = None
        self.depth = None
        # self.thread = multiprocessing.Process(target=self.record_video)
        self.thread = Thread(target=self.record_video)
        # self.thread.start()
        

    def record_video(self):
        depth_sensor = self.profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        
        e1 = cv2.getTickCount()
        count = 0
        self.start_time = time.time() - self.time0
        print('Start time of video thread: ', self.start_time)
        try:
            rs.recorder.resume(self.profile.get_device().as_recorder())
            while(True):
                # Wait for a coherent pair of frames: depth and color
                frames = self.pipeline.wait_for_frames()
                frames.keep()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    pdb.set_trace()
                    continue
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_image = depth_image * depth_scale
                # pdb.set_trace()
                depth = np.mean(depth_image[int(480/4):int(480-480/4), int(848/4):int(848-848/4)])
                print(f'current distance: {depth}', end="\r")
                count += 1
                e2 = cv2.getTickCount()
                t = (e2 - e1) / cv2.getTickFrequency()
                if t >= self.total_seconds:  # change it to record what length of video you are interested in
                    end_time = time.time() - self.time0
                    self.end_time = end_time
                    self.depth = depth
                    print("\n")
                    print('End time of video thread: ', end_time)
                    print('Total frames: ', count)
                    break
        finally:
            # Stop streaming
            self.pipeline.stop()
    
        
    def show(self): 
        if self.images is not None: 
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', self.images)
            cv2.waitKey(1)

    def init_depth_camera(self):
        pipeline = rs.pipeline()
        config = rs.config()
        
        config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, self.fps)
        config.enable_record_to_file(self.filename)
        # Start streaming sazxaas
        profile = pipeline.start(config)
        rgb_sensor = profile.get_device().query_sensors()[1]
        rgb_sensor.set_option(rs.option.enable_auto_exposure, True)
        rs.recorder.pause(profile.get_device().as_recorder())
        return pipeline, profile



def main():
    args = parser.parse_args()
    # set default device for recording
    sd.default.device = [0, 4]
    print(sd.query_devices())

    # create folder
    parent_folder = 'NewAudio3D-Dataset'
    scene_folder = os.path.join(parent_folder, args.building, 'scene-' + str(args.scene_id).zfill(3)) 
    os.makedirs(scene_folder, exist_ok=True)

    # check number of clip exists
    clip_list = glob.glob(f"{scene_folder}/*")
    clip_list.sort()
    clip_id = len(clip_list)
    if args.overwrite_previous and clip_id > 0:
        clip_id -= 1
        
    print(f"Number of clips in {scene_folder}: {clip_id}")
    sample_folder = os.path.join(scene_folder, 'clip-'+str(clip_id).zfill(3))
    os.makedirs(sample_folder, exist_ok=True)
    print(f"Start recording {sample_folder}")

    t0 = time.time()
    video = videoThread(t0, args.seconds, sample_folder)
    sound = soundThread(t0, args.seconds, sample_folder)
    time.sleep(5)
    ## visualization of video, cannot record together with sound
    # while video.thread.is_alive(): 
    #     try: 
    #         video.show()
    #     except AttributeError: 
    #         pass   
    
    # compute time difference
    video.thread.start()
    sound.thread.start()
    video.thread.join()
    sound.thread.join()
    time_diff = float(video.start_time - sound.start_time)
    print('Time difference: ', time_diff)
    info_dict = {
        'Audio Sample Rate': sound.fs,
        'Audio Length': args.seconds,
        'Video FPS': video.fps,
        'Time Difference(video - sound)': time_diff,
        'Building': args.building,
        'Scene id': args.scene_id,
        'Clip id': clip_id,
        'depth': video.depth
    }
    json_path = os.path.join(sample_folder, 'meta.json')
    with open(json_path, 'w') as fp:
        json.dump(info_dict, fp, sort_keys=False, indent=4)
    print(info_dict)



# To see current sound devices: 
# python -m sounddevice

# Example: 
# python record_video_audio.py --building='home' --scene_id=0 --seconds=10

if __name__ == "__main__":
    main()
