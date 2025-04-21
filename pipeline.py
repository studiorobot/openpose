## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#####################################################
## librealsense tutorial #1 - Accessing depth data ##
#####################################################

# First import the library
import pyrealsense2 as rs
import numpy as np
import subprocess
import sys

try:
    # Create a context object. This object owns the handles to all connected realsense devices
    pipeline = rs.pipeline()


    # Use v4l2loopback device (adjust if necessary)
    virtual_cam = '/dev/video0'
    width, height, fps = 640, 480, 30

    ffmpeg_cmd = [
        'ffmpeg',
        '-loglevel', 'info',
        '-f', 'rawvideo',
        '-pix_fmt', 'bgr24',
        '-s', f'{width}x{height}',
        '-r', str(fps),
        '-i', '-',
        '-f', 'v4l2',
        virtual_cam
    ]
    ffmpeg_proc = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)

    # Configure streams
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    # config.enable_all_streams()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

    # Start streaming
    profile = pipeline.start(config)

    while True:
        # This call waits until a new coherent set of frames is available on a device
        # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        if not depth: continue

        # # Print a simple text-based representation of the image, by breaking it into 10x20 pixel regions and approximating the coverage of pixels within one meter
        # coverage = [0]*64
        # for y in range(480):
        #     for x in range(640):
        #         dist = depth.get_distance(x, y)
        #         if 0 < dist and dist < 1:
        #             coverage[x//10] += 1
            
        #     if y%20 is 19:
        #         line = ""
        #         for c in coverage:
        #             line += " .:nhBXWW"[c//25]
        #         coverage = [0]*64
        #         print(line)

        # Create alignment primitive with color as its target stream:
        align = rs.align(rs.stream.color)
        frames = align.process(frames)

        # Update color and depth frames:
        aligned_depth_frame = frames.get_depth_frame()
        depth = np.asanyarray(aligned_depth_frame.get_data())
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        depth = depth * depth_scale
        print("DEPTH", depth)


        if not color:
            print("NO COLOR")
            continue
        frame = np.asanyarray(color.get_data())
        try:
            print(" hELLO ")
            ffmpeg_proc.stdin.write(frame.tobytes())
        except BrokenPipeError:
            print("FFmpeg pipe broken.")
            break
    exit(0)
#except rs.error as e:
#    # Method calls agaisnt librealsense objects may throw exceptions of type pylibrs.error
#    print("pylibrs.error was thrown when calling %s(%s):\n", % (e.get_failed_function(), e.get_failed_args()))
#    print("    %s\n", e.what())
#    exit(1)
except Exception as e:
    print(e)
    pass