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
    virtual_cam = '/dev/video1'
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

        

        # Create alignment primitive with color as its target stream:
        align = rs.align(rs.stream.color)
        frames = align.process(frames)

        # Update color and depth frames:
        aligned_depth_frame = frames.get_depth_frame()
        depth = np.asanyarray(aligned_depth_frame.get_data())
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        depth = depth * depth_scale
        print("DEPTH", depth.shape, np.mean(depth))

        # Load last line from file
        with open('json/test.txt', 'r') as file:
            last_line = file.readlines()[-1].strip()

        # Convert to float array
        values = np.array(list(map(float, last_line.split(','))))

        # Reshape to (num_joints, 3)
        joints = values.reshape(-1, 3)

        # Scale to pixel coordinates
        x_coords = (np.floor(joints[:, 0] * height)).astype(int)
        y_coords = (np.floor(joints[:, 1] * width)).astype(int)

        # Get depth values (assumes depth is a 2D NumPy array)
        print(joints[:, 0])
        print(x_coords, y_coords)
        depths = depth[x_coords, y_coords]

        # Stack final output and format to string
        output_array = np.stack([x_coords, y_coords, depths], axis=1)
        new_line = ','.join(map(str, output_array.flatten()))

        # # read x y positions for all joints in order from json
        # lineLast = None
        # with open('json/test.txt', 'r') as file:
        #     lines = file.readlines()
        #     linelast = lines[-1]

        # values = linelast.strip().split(',')
        # new_line = ""
        # curr_height = curr_width = None
        # for index, value in enumerate(values):
        #     # counter = 0
        #     if index % 3 == 0: #even
        #         curr_width = value
        #     elif index % 3 == 1: #odd
        #         curr_height = value
        #     else: 
        #         pixel_x = int(width*float(curr_width))
        #         pixel_y = int(height*float(curr_height))
                
        #         # print("curr width height", float(curr_width), " ", curr_height)
        #         curr_depth = depth[pixel_x][pixel_y]
        #         # counter+=1
        #         new_line = new_line.join([str(pixel_x), " ", str(pixel_y), " ", str(curr_depth)])
        #         # print(index, "NEWLINE: ", new_line, "\n")
            
        # write new line to output
        # with open("output.txt", "a") as file:
        #     file.write(new_line)
        # print("HELLOOOOOOOOOOOOO")

        f = open("output.txt", "a")
        # f.write("Now the file has more content!")
        f.write(new_line + '\n')
        f.close()

        if not color:
            print("NO COLOR")
            continue
        frame = np.asanyarray(color.get_data())
        try:
            print("writing ffmpeg")
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