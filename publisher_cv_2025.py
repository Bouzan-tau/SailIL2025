import cv2
import pyzed.sl as sl
import math
import numpy as np
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import torch
import rospy
from cv_from_zed.msg import ObjectDistanceInfo

# Define node frequency in Hz
freq = 10

def object_detect():
    
    model = YOLO('/home/sail/Desktop/sail_git/computer_vision/Detect2024.pt')  # Load a custom model, weights file

    # Initialize the ROS node
    rospy.init_node('object_detect_node', anonymous=True)

    # Set node frequency
    rate = rospy.Rate(freq)  # 10 Hz to match 10 fps of object detect

    zed = sl.Camera()
    init_params = sl.InitParameters() # Configures the camera before opening it
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode (ideal for real-time object detection)
    init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = freq  # Set FPS at 10

    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Camera opening failed with error:", err)
        exit(1)

    runtime_parameters = sl.RuntimeParameters() # Set runtime parameters for depth and texture confidence
    runtime_parameters.confidence_threshold = 100 # Only use highly accurate depth measurements (confidence level at 100)
    runtime_parameters.texture_confidence_threshold = 100 # Filter out depth data from low-texture areas for reliable results

    image = sl.Mat() # Creates a sl.Mat object to store image data captured by the ZED camera.
    #depth_m = sl.Mat()
    point_cloud = sl.Mat() # Creates a sl.Mat object to store point cloud data (3D coordinates), representing position in space.

    # Create a publisher for the ObjectDistanceInfo message
    object_distance_pub = rospy.Publisher('object_distance_info', ObjectDistanceInfo, queue_size=10)

    while not rospy.is_shutdown():
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS: # Checks if the camera successfully captured a new frame
            zed.retrieve_image(image, sl.VIEW.LEFT) # Retrieve the left RGB image from the camera and store it in the 'image' object
            #zed.retrieve_measure(depth_m, sl.MEASURE.DEPTH)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA) # Retrieve the 3D coordinates (x, y, z) and color (RGBA) for each pixel and store in 'point_cloud'
            frame = image.get_data()  # Get the data as a numpy array
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)  # Convert BGRA to BGR for OpenCV processing
            # frame_resized = cv2.resize(frame, (640, 640))  # Resize the image
            results = model.predict(frame) # Passes the image (frame) to the YOLO model for object detection.

            # Check if any objects were detected
            if results:
                # Loop through all detected objects
                for img_results in results:
                    annotator = Annotator(frame) # # Initialize an Annotator object to draw bounding boxes and labels on the image
                    for box in img_results.boxes:  # use .boxes to access bounding box information
                        b = box.xyxy[0]  # Get the top-left and bottom-right bounding box coordinates of the detected object
                        c = box.cls # Get the class index of the detected object for labeling or categorization
                        object_label = model.names[int(c)] # Get the label name 
                        annotator.box_label(b, object_label)  # Draw a rectangle and label around each object

                        # Get the center of the bounding box
                        x = torch.round((b[0] + b[2]) / 2).item()
                        y = torch.round((b[1] + b[3]) / 2).item()

                        # Estimate the distance to the object
                        err, point_cloud_value = point_cloud.get_value(x, y)
                        #depth = depth_m.get_value(x, y)
                        
                        # print(f"Distance to {object_label} is: x: {point_cloud_value[0]}, y: {point_cloud_value[1]}, z: {point_cloud_value[2]}") # Here for debugging

                        # Create an ObjectDistanceInfo message
                        object_distance_msg = ObjectDistanceInfo()
                        object_distance_msg.label = object_label
                        object_distance_msg.distance_x = point_cloud_value[0]
                        object_distance_msg.distance_y = point_cloud_value[1]
                        object_distance_msg.distance_z = point_cloud_value[2]

                        # Publish the message to all subscribers of the object_distance_info topic.
                        object_distance_pub.publish(object_distance_msg)

                    # Create an ObjectDistanceInfo to mark the end of objects in a frame
                    object_distance_msg = ObjectDistanceInfo()
                    object_distance_msg.label = 'end' # Mark the message with the label 'end'
                    object_distance_msg.distance_x = 0 # No object data, set x-coordinate to 0
                    object_distance_msg.distance_y = 0 # No object data, set y-coordinate to 0
                    object_distance_msg.distance_z = 0 # No object data, set z-coordinate to 0

                    # Publishes the message to all subscribers of the object_distance_info topic.
                    object_distance_pub.publish(object_distance_msg)

                    frame = annotator.result()  # Update the image with annotations

            cv2.imshow("Image", frame)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # Ensures the loop runs at a consistent frequency by pausing for the remaining time 
        # after all tasks in this iteration are completed, as defined by rospy.Rate
        rate.sleep()


if __name__ == '__main__':
    try:
        object_detect()
    except rospy.ROSInterruptException:
        pass

    # When everything is done, release the capture
    cv2.destroyAllWindows()
