import argparse
import pyrealsense2 as rs
import numpy as np
import cv2
import os
import pickle
import time
from scipy.io import wavfile
import subprocess
import json
import pdb


parser = argparse.ArgumentParser(description="")

parser.add_argument("-i", "--input", type=str,
                    help="Path to the folder saving bag and wav file", required=True)



def extract_from_bag(bag_file, FPS):
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
            
            # print(frame_num)
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
    frame_index = np.asarray(frame_index) - frame_index[0]
    time_stamps = np.asarray(time_stamps) / 1000
    return depth_set, color_set, frame_index, time_stamps 


def generate_sync(folder, color_set, time_stamps, frame_index, wav_file, time_diff, fps): 
    # import pdb; pdb.set_trace()
    depth_color_sync = os.path.join(folder, 'dep_color_sync.mp4')
    fourcc = cv2.VideoWriter_fourcc(*'MP4V')
    frame_start_time = 1 / fps * frame_index[0]
    total_time_diff = time_diff + frame_start_time
    if total_time_diff >= 0:
        audio_start_point = total_time_diff
    else:
        audio_start_point = 0.0
        frame_start = int(fps * np.abs(total_time_diff))
        color_set = color_set[frame_start:]

    # import pdb; pdb.set_trace()
    out = cv2.VideoWriter(depth_color_sync, fourcc, fps, (color_set[0].shape[1],color_set[0].shape[0]))
    # if frame_start_time > time_diff:     
    for i in range(len(color_set)): 
        images = color_set[i]
        out.write(images)
    cv2.destroyAllWindows()
    out.release()

    rate, data = wavfile.read(wav_file) 
    clipped_wav_out = os.path.join(folder, 'sound_clipped.wav')
    wavfile.write(clipped_wav_out, rate, data[int(audio_start_point * rate):, :])

    mixed_path = os.path.join(folder, 'mixed.mp4')
    subprocess.run(['ffmpeg', '-v', 'quiet', '-y', '-i', depth_color_sync, '-i', clipped_wav_out, '-shortest', mixed_path])


def main():
    args = parser.parse_args()
    # create folder
    sync_folder = os.path.join('./sync', args.input)
    os.makedirs(sync_folder, exist_ok=True)
    
    bag = os.path.join(args.input, 'video.bag')
    wav = os.path.join(args.input, 'sound.wav')
    meta = os.path.join(args.input, 'meta.json')
    with open(meta, 'r') as fp:
        meta_data = json.load(fp)
    time_diff = meta_data['Time Difference(video - sound)']

    depth_set, color_set, frame_index, time_stamps = extract_from_bag(bag, FPS=meta_data['Video FPS'])

    generate_sync(sync_folder, color_set, time_stamps, frame_index, wav, time_diff, fps=meta_data['Video FPS'])


if __name__ == "__main__":
    main()