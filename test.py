import cv2
import sys
import os
import pyrealsense2 as rs
import numpy as np
import pyopenpose as op

 
def get_depth_value(depth_image, x, y,person_id):
    #座標在圖像裡面
    if x < 0 or y < 0 or x >= depth_image.shape[1] or y >= depth_image.shape[0]:
        print("Coordinates out of bounds")
        return None

    depth_value = depth_image[y, x]
    
    if depth_value == 0:
        print(f"Person_id:{person_id},Depth value at coordinates (x={x}, y={y}): 0 m (no depth information)")
    else:
        depth_in_meters = depth_value * 0.00025  #轉成公尺
        print(f"Person_id:{person_id},Depth value at coordinates (x={x}, y={y}): {depth_in_meters:.4f} m")

    return depth_value


def main():
    try:

        # 初始化RealSense pipeline
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        
        align_to = rs.stream.color
        align = rs.align(align_to)
        
        pipeline.start(config)
        #datum = op.Datum()
        
        
        # 初始化OpenPose
        params = dict()
        params["model_folder"] = "./models/"
        params["net_resolution"] = "256x256"
        params["model_pose"] = "BODY_25"

        opWrapper = op.WrapperPython()
        opWrapper.configure(params)
        opWrapper.start()
        
        while True:

            # L515拿深度
            frames = pipeline.wait_for_frames()
            
            
            aligned_frames = align.process(frames) 
            
            aligned_depth_frame = aligned_frames.get_depth_frame()
            aligned_color_frame = aligned_frames.get_color_frame()

            if not aligned_depth_frame or not aligned_color_frame:
                continue
            
            color_image = np.asanyarray(aligned_color_frame.get_data())
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            
            
            #depth_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,alpha=0.03),cv2.COLORMAP_JET)
            
            
            datum = op.Datum()
            datum.cvInputData = color_image
            datums = op.VectorDatum([datum])
            opWrapper.emplaceAndPop(datums)
            
            keypoints = datum.poseKeypoints
            
            if keypoints is not None:
                #for person in keypoints:
                for person_id, person in enumerate(keypoints):
                    #i = 0
                    nose_x, nose_y = int(person[8][0]), int(person[8][1])
                    #x,y,confidence = person[i][0]
                    #print("x:",nose_x,",y:",nose_y)
                    get_depth_value(depth_image, nose_x, nose_y,person_id)
                    
            
            #blend_image = cv2.addWeighted(color_image,0.7,depth_image,0.5,0)
            
            #images = np.hstack((color_image,depth_image))
            
            
            pose_frame = datum.cvOutputData
            frame = cv2.addWeighted(color_image, 0.7, pose_frame, 0.3, 0)
            
            cv2.imshow('window',frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            cv2.waitKey(1)

        # Release resources
        
    finally:
        pipeline.stop()

if __name__ == "__main__":
    main()