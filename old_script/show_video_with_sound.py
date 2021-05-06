import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import os.path
import rosbag

# Create object for parsing command-line options
parser = argparse.ArgumentParser(description="Read recorded bag file and display depth stream in jet colormap.\
                                Remember to change the stream resolution, fps and format to match the recorded.")
# Add argument which takes path to a bag file as an input
parser.add_argument("-i", "--input", type=str, help="Path to the bag file")
# Parse the command line arguments to an object
args = parser.parse_args()
# Safety if no parameter have been given
if not args.input:
    print("No input paramater have been given.")
    print("For help type --help")
    exit()
# Check if the given file have bag extension
if os.path.splitext(args.input)[1] != ".bag":
    print("The given file is not of correct file format.")
    print("Only .bag files are accepted")
    exit()
if not os.path.exists(args.input):
    print("The given file is not of correct file format.")
    print("Only .bag files are accepted")
    exit()
try:
    # Create pipeline
    fps = 15
    pipeline = rs.pipeline()
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Create a config object
    config = rs.config()
    # Tell config that we will use a recorded device from filem to be used by the pipeline through playback.
    rs.config.enable_device_from_file(config, args.input)
    # Configure the pipeline to stream the depth stream
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, fps)
    # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, fps)
    # Start streaming from file
    # Create opencv window to render image in

    # cv2.namedWindow("Depth Stream", cv2.WINDOW_AUTOSIZE)
    
    # Create colorizer object
    colorizer = rs.colorizer()

    pipeline.start(config)
    # Streaming loop
    while True:
        # Get frameset of depth

        frames = pipeline.wait_for_frames()
        print(frames.get_frame_number())
        # aligned_frames = align.process(frames)
        # Get depth frame
        # depth_frame = frames.get_depth_frame()

        # # Colorize depth frame to jet colormap
        # depth_color_frame = colorizer.colorize(depth_frame)

        # # Convert depth_frame to numpy array to render image in opencv
        # depth_color_image = np.asanyarray(depth_color_frame.get_data())

        # # Render image in opencv window
        # cv2.imshow("Depth Stream", depth_color_image)
        # key = cv2.waitKey(1)
        # if pressed escape exit program
        # if key == 27:
        #     cv2.destroyAllWindows()
        #     break

finally:
    pass
