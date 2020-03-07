import argparse
import pyrealsense2 as rs
import numpy as np
import cv2
import os
import pickle
import time

FPS = 15

def save_to_pickle(stream_list, bag_file):
    save_path = bag_file.split('.')[0] + '.pkl'
    pickle_out = open(save_path, "wb")
    pickle.dump(stream_list, pickle_out)
    pickle_out.close()

def extract_from_bag(bag_file):

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


    # Get data scale from the device
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    align_to = rs.stream.color
    align = rs.align(align_to)

    count = 0
    # depth_set = np.zeros([1, 480, 640])
    # color_set = np.zeros([1, 480, 640, 3])
    stream_list = []
    frame_num = -1
    # for i in range(2):
    #     frames = pipeline.wait_for_frames(timeout_ms=100)
    while True:
        try:
            frames = pipeline.wait_for_frames(timeout_ms=100)
            frames.keep()
            if frame_num == frames.get_frame_number():
                continue
            timestamp = frames.get_timestamp()
            frame_num = frames.get_frame_number()
            print(frames.get_frame_number())
            print(timestamp)
            if frames.size() < 2:
                # Inputs are not ready yet
                continue

        except (RuntimeError):
            print('Total frame count:', count)
            pipeline.stop()
            break
        
        frame_dict = {}
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
        # cv2.imwrite(str(count)+'.png', color_image)
        # convert color image to BGR for OpenCV
        # r, g, b = cv2.split(color_image)
        # color_image = cv2.merge((b, g, r))

        # Unaligned depth stream
        # unaligned_depth_frame = frames.get_depth_frame()
        # unaligned_depth_image = np.asanyarray(unaligned_depth_frame.get_data())
        # unaligned_depth_colormap = np.asanyarray(rs.colorizer().colorize(unaligned_depth_frame).get_data())

        # if count != 0:
        #     depth_set = np.vstack((depth_set, np.zeros([1, 480, 640])))
        #     color_set = np.vstack((color_set, np.zeros([1, 480, 640, 3])))
        # depth_set[count, :, :] = depth_image
        # color_set[count, :, :, :] = color_image

        # save info to dict
        frame_dict['timestamp'] = timestamp
        frame_dict['frame_number'] = frame_num
        frame_dict['depth'] = depth_image
        frame_dict['color'] = color_image

        stream_list.append(frame_dict)
        # CV display
        depth_colormap = np.asanyarray(rs.colorizer().colorize(depth_frame).get_data())

        images = np.hstack(
            (color_image, depth_colormap))
        cv2.namedWindow('RGB and Depth Stream', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RGB and Depth Stream', images)

        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break
        count += 1
        # time.sleep(0.1)
        # import pdb; pdb.set_trace()
    

    print("Done!")
    save_to_pickle(stream_list, bag_file)
    print('Saved!')
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Read recorded bag file and display depth stream in jet colormap. Remember to change the stream resolution, fps and format to match the recorded.")

    # Add argument which takes path to a bag file as an input
    parser.add_argument("-i", "--input", type=str, help="Path to the bag file", required=True)

    # Parse the command line arguments to an object
    args = parser.parse_args()

    # Check if the given file have bag extension
    if args.input.split('.')[-1] != 'bag':
        print("The given file is not of correct file format.")
        print("Only .bag files are accepted")
        exit()
    
    extract_from_bag(args.input)
