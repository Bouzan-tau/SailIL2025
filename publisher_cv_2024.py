import cv2
import pyzed.sl as sl
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import torch
import rospy
from cv_from_zed.msg import ObjectDistanceInfo

# Define node frequency in Hz
freq = 10

def object_detect():
    
    model = YOLO('YardenAsaf.pt')  # load a custom model

    # Initialize the ROS node
    rospy.init_node('object_detect_node', anonymous=True)

    # Set node frequency
    rate = rospy.Rate(freq)  # 10 Hz to match 10 fps of object detect

    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = freq  # Set fps at 10

    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Camera opening failed with error:", err)
        exit(1)

    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.confidence_threshold = 100
    runtime_parameters.texture_confidence_threshold = 100

    image = sl.Mat()
    #depth_m = sl.Mat()
    point_cloud = sl.Mat()

    # Create a publisher for the ObjectDistanceInfo message
    object_distance_pub = rospy.Publisher('object_distance_info', ObjectDistanceInfo, queue_size=10)

    while not rospy.is_shutdown():
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            #zed.retrieve_measure(depth_m, sl.MEASURE.DEPTH)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
            frame = image.get_data()  # Get the data as a numpy array
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)  # Convert from BGRA to BGR
            #frame_resized = cv2.resize(frame, (640, 640))  # Resize the image
            results = model.predict(frame)

            # Check if any objects were detected
            if results:
                # Loop through all detected objects
                for img_results in results:
                    annotator = Annotator(frame)
                    for box in img_results.boxes:  # use .boxes to access bounding box information
                        b = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
                        c = box.cls
                        annotator.box_label(b, model.names[int(c)])  # Draw a rectangle and label around each object

                        # Get the center of the bounding box
                        x = torch.round((b[0] + b[2]) / 2).item()
                        y = torch.round((b[1] + b[3]) / 2).item()

                        # Estimate the distance to the object
                        err, point_cloud_value = point_cloud.get_value(x, y)
                        #depth = depth_m.get_value(x, y)
                        object_label = model.names[int(c)]
                        #print(f"Distance to {object_label} is: x: {point_cloud_value[0]}, y: {point_cloud_value[1]}, z: {point_cloud_value[2]}")

                        # Create an ObjectDistanceInfo message
                        object_distance_msg = ObjectDistanceInfo()
                        object_distance_msg.label = object_label
                        object_distance_msg.distance_x = point_cloud_value[0]
                        object_distance_msg.distance_y = point_cloud_value[1]
                        object_distance_msg.distance_z = point_cloud_value[2]

                        # Publish the message
                        object_distance_pub.publish(object_distance_msg)

                    # Create an ObjectDistanceInfo to mark the end of objects in a frame
                    object_distance_msg = ObjectDistanceInfo()
                    object_distance_msg.label = 'end'
                    object_distance_msg.distance_x = 0
                    object_distance_msg.distance_y = 0
                    object_distance_msg.distance_z = 0

                    # Publish the message
                    object_distance_pub.publish(object_distance_msg)

                    frame = annotator.result()  # update the image with annotations

            cv2.imshow("Image", frame)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        rate.sleep()


if __name__ == '__main__':
    try:
        object_detect()
    except rospy.ROSInterruptException:
        pass

    # When everything is done, release the capture
    cv2.destroyAllWindows()
