import cv2
import pyzed.sl as sl
import math
import numpy as np
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import torch
import rclpy
from rclpy.node import Node
from cv_from_zed.msg import ObjectDistanceInfo
from std_msgs.msg import String

# Define node frequency in Hz
freq = 10

class ObjectDetectNode(Node):

    def __init__(self):
        super().__init__('object_detect_node')

        # Initialize YOLO model
        self.model = YOLO('/home/sail/Desktop/sail_git/computer_vision/Detect2024.pt')

        # Create a publisher for the ObjectDistanceInfo message
        self.object_distance_pub = self.create_publisher(ObjectDistanceInfo, 'object_distance_info', 10)

        # Initialize the ZED camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()  # Configures the camera before opening it
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
        init_params.coordinate_units = sl.UNIT.METER  # Use meter units for depth measurements
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = freq  # Set FPS at 10

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Camera opening failed with error: {err}")
            exit(1)

        self.runtime_parameters = sl.RuntimeParameters()  # Set runtime parameters
        self.runtime_parameters.confidence_threshold = 100  # Only use highly accurate depth measurements
        self.runtime_parameters.texture_confidence_threshold = 100  # Filter low-texture depth data

        self.image = sl.Mat()
        self.point_cloud = sl.Mat()  # Create point cloud for 3D data

        self.timer = self.create_timer(1.0 / freq, self.detect_objects)  # Timer to call detect_objects at the given frequency

    def detect_objects(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
            frame = self.image.get_data()  # Get image data as numpy array
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)  # Convert BGRA to BGR

            # Perform object detection
            results = self.model.predict(frame)

            if results:
                for img_results in results:
                    annotator = Annotator(frame)
                    for box in img_results.boxes:
                        b = box.xyxy[0]  # Get bounding box coordinates
                        c = box.cls  # Get class index
                        object_label = self.model.names[int(c)]  # Get object label
                        annotator.box_label(b, object_label)

                        # Get the center of the bounding box
                        x = torch.round((b[0] + b[2]) / 2).item()
                        y = torch.round((b[1] + b[3]) / 2).item()

                        # Estimate the distance to the object using the point cloud
                        err, point_cloud_value = self.point_cloud.get_value(x, y)

                        # Create an ObjectDistanceInfo message
                        object_distance_msg = ObjectDistanceInfo()
                        object_distance_msg.label = object_label
                        object_distance_msg.distance_x = point_cloud_value[0]
                        object_distance_msg.distance_y = point_cloud_value[1]
                        object_distance_msg.distance_z = point_cloud_value[2]

                        # Publish the message
                        self.object_distance_pub.publish(object_distance_msg)

                    # End of frame message
                    object_distance_msg = ObjectDistanceInfo()
                    object_distance_msg.label = 'end'
                    object_distance_msg.distance_x = 0
                    object_distance_msg.distance_y = 0
                    object_distance_msg.distance_z = 0
                    self.object_distance_pub.publish(object_distance_msg)

                    frame = annotator.result()  # Update image with annotations

            cv2.imshow("Image", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.destroy_node()
                cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectNode()

    try:
        rclpy.spin(node)  # ROS 2 spin function to keep the node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
