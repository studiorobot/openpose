#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import subprocess
import sys

import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures

# Check RealSense device
ctx = rs.context()
if len(ctx.devices) == 0:
    print("No RealSense devices found.")
    exit(1)

# RealSense config
pipeline = rs.pipeline()
config = rs.config()


# Find RealSense Devices
print("Searching Devices..")
selected_devices = []                     # Store connected device(s)

for d in rs.context().devices:
    selected_devices.append(d)
    print(d.get_info(rs.camera_info.name))
if not selected_devices:
    print("No RealSense device is connected!")

# Find Depth and RGB Sensors
rgb_sensor = depth_sensor = None

for device in selected_devices:                         
    print("Required sensors for device:", device.get_info(rs.camera_info.name))
    for s in device.sensors:                              # Show available sensors in each device
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            print(" - RGB sensor found")
            rgb_sensor = s                                # Set RGB sensor
        if s.get_info(rs.camera_info.name) == 'Stereo Module':
            depth_sensor = s                              # Set Depth sensor
            print(" - Depth sensor found")

colorizer = rs.colorizer()                                # Mapping depth data into RGB color space
profile = pipeline.start(config)                                 # Configure and start the pipeline

fig, axs = plt.subplots(nrows=1, ncols=2, figsize=(12,4)) # Show 1 row with 2 columns for Depth and RGB frames
title = ["Depth Image", "RGB Image"]                      # Title for each frame

for _ in range(10):                                       # Skip first frames to give syncer and auto-exposure time to adjust
    frameset = pipeline.wait_for_frames()
    
for _ in range(5):                                        # Increase to display more frames
    frameset = pipeline.wait_for_frames()                     # Read frames from the file, packaged as a frameset
    depth_frame = frameset.get_depth_frame()              # Get depth frame
    color_frame = frameset.get_color_frame()              # Get RGB frame

    colorized_streams = []                                # This is what we'll actually display
    if depth_frame:
        colorized_streams.append(np.asanyarray(colorizer.colorize(depth_frame).get_data()))
    if color_frame:
        colorized_streams.append(np.asanyarray(color_frame.get_data()))
    
    for i, ax in enumerate(axs.flatten()):                # Iterate over all (Depth and RGB) colorized frames
        if i >= len(colorized_streams): continue          # When getting less frames than expected
        plt.sca(ax)                                       # Set the current Axes and Figure
        plt.imshow(colorized_streams[i])                  # colorized frame to display
        plt.title(title[i])                               # Add title for each subplot
    # clear_output(wait=True)                               # Clear any previous frames from the display
    plt.tight_layout()                                    # Adjusts display size to fit frames
    plt.pause(1)                                          # Make the playback slower so it's noticeable
    
pipeline.stop()                                               # Stop the pipeline
print("Done!")




width, height, fps = 640, 480, 30
# config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
config.enable_all_streams()
profile = pipeline.start(config)

# Use v4l2loopback device (adjust if necessary)
virtual_cam = '/dev/video0'

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

try:
    for _ in range(10):                                       # Skip first frames to give syncer and auto-exposure time to adjust
        frames = pipeline.wait_for_frames()
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        # DEPTH STUFF
        depth_frame = frames.get_depth_frame()

        # Create alignment primitive with color as its target stream:
        align = rs.align(rs.stream.color)
        frames = align.process(frames)

        # Update color and depth frames:
        aligned_depth_frame = frames.get_depth_frame()
        depth = np.asanyarray(aligned_depth_frame.get_data())
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        depth = depth * depth_scale
        print("DEPTH", depth)
        

        if not color_frame:
            continue
        frame = np.asanyarray(color_frame.get_data())
        try:
            ffmpeg_proc.stdin.write(frame.tobytes())
        except BrokenPipeError:
            print("FFmpeg pipe broken.")
            break
except KeyboardInterrupt:
    print("\nStopping stream.")
finally:
    pipeline.stop()
    if ffmpeg_proc.stdin:
        ffmpeg_proc.stdin.close()
    ffmpeg_proc.wait()
