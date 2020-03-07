import argparse
import pyrealsense2 as rs
import numpy as np
import cv2
import os
import pickle
import time
from scipy.io import wavfile
import subprocess

import pdb


parser = argparse.ArgumentParser(description="")

parser.add_argument("-i", "--input", type=str,
                    help="Path to the folder saving bag and wav file", required=True)


def save_to_pickle(depth_set, color_set, name):
    save_path = name + '.pkl'
    stream_dict = {}
    stream_dict['depth'] = depth_set
    stream_dict['color'] = color_set
    pickle_out = open(save_path, "wb")
    pickle.dump(stream_dict, pickle_out)
    pickle_out.close()


def extract_from_bag(bag_file, FPS=15):
    config = rs.config()
    pipeline = rs.pipeline()
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, FPS)
    rs.config.enable_device_from_file(config, bag_file, repeat_playback=False)

    # Start
    profile = pipeline.start(config)
    # this makes it so no frames are dropped while writing video
    playback = profile.get_device().as_playback()
    playback.set_real_time(False)

    align_to = rs.stream.color
    align = rs.align(align_to)

    # Get data scale from the device
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    count = 0
    depth_set = []
    color_set = []
    frame_index = []
    time_stamps = []
    start_time = 0
    frame_num = -1

    while True:
        try:
            frames = pipeline.wait_for_frames()
            frames.keep()
            if frame_num == frames.get_frame_number(): 
                continue
            frame_num = frames.get_frame_number()
            
            print(frame_num)
            if count == 0: 
                time_stamps.append(0)
                start_time = frames.get_timestamp()
            else: 
                time_stamps.append(frames.get_timestamp() - start_time)
            frame_index.append(frame_num)
            if frames.size() < 2:
                # Inputs are not ready yet
                continue
        except (RuntimeError):
            print('Total frame count:', count)
            pipeline.stop()
            break

        # align the deph to color frame
        aligned_frames = align.process(frames)
        # Align depth frame according to color frame
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # validate that both frames are valid
        if not depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data())
        # convert to meters
        depth_image = depth_image * depth_scale
        color_image = np.asanyarray(color_frame.get_data())

        depth_set.append(depth_image)
        color_set.append(color_image)

        count += 1

    print("Video processing done!")
    return depth_set, color_set, np.asarray(frame_index), np.asarray(time_stamps)/1000


def generate_sync(folder, color_set, time_stamps, frame_index, wav_file, time_diff, fps=15): 
    depth_color_sync = os.path.join(folder, 'dep_color_sync.mp4')
    fourcc = cv2.VideoWriter_fourcc(*'MP4V')
    frame_start_time = 1/fps * frame_index[0]
    out = cv2.VideoWriter(depth_color_sync, fourcc, fps, (color_set[0].shape[1],color_set[0].shape[0]))
    if frame_start_time > time_diff:     
        for i in range(len(time_stamps)): 
            images = color_set[i]
            out.write(images)
        cv2.destroyAllWindows()
        out.release()

        rate, data = wavfile.read(wav_file) 
        # pdb.set_trace()
        clipped_wav_out = os.path.join(folder, 'sound_clipped.wav')
        wavfile.write(clipped_wav_out, rate, data[int((1/fps *frame_index[0] - time_diff) * rate):, :])
        # pdb.set_trace()
        # mixed_path = os.path.join(folder, 'mixed.mp4')
        # subprocess.run(['ffmpeg', '-i', depth_color_sync, '-i', clipped_wav_out, '-shortest', mixed_path])

    else: 
        start_frame_index = int((time_diff - frame_start_time) * 15)
        for i in range(start_frame_index, len(time_stamps)): 
            images = color_set[i]
            out.write(images)
        cv2.destroyAllWindows()
        out.release()

        rate, data = wavfile.read(wav_file) 
        # pdb.set_trace()
        clipped_wav_out = os.path.join(folder, 'sound_clipped.wav')
        wavfile.write(clipped_wav_out, rate, data)
        # pdb.set_trace()
    mixed_path = os.path.join(folder, 'mixed.mp4')
    subprocess.run(['ffmpeg', '-i', depth_color_sync, '-i', clipped_wav_out, '-shortest', mixed_path])


if __name__ == "__main__":

    # Parse the command line arguments to an object
    args = parser.parse_args()

    # create folder
    sync_folder = os.path.join('./sync', args.input)
    if not os.path.exists(sync_folder): 
        os.mkdir(sync_folder)

    bag = os.path.join('samples', args.input, 'video.bag')
    wav = os.path.join('samples', args.input, 'sound.wav')
    time_diff = np.load(os.path.join('samples', args.input, 'td.npy'))

    depth_set, color_set, frame_index, time_stamps = extract_from_bag(bag)

    generate_sync(sync_folder, color_set, time_stamps, frame_index, wav, time_diff)

# ffmpeg -i synchronize.mp4 -i out_clipped.wav -shortest mixed.mp4