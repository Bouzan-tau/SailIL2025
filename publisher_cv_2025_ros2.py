import cv2
import numpy as np
import pyzed.sl as sl
import math
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import torch
import rclpy
from rclpy.node import Node
from cv_from_zed_ros2.msg import ObjectDistanceInfo
import requests
import socket
import os

TASK_CONF_MAP = {
    "task1": 0.9,
    "task2": 0.2,
    "task3": 0.9
}

def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "127.0.0.1"

server_url = f"http://{get_local_ip()}:5000/update"

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detect_node')
        self.publisher_ = self.create_publisher(ObjectDistanceInfo, 'object_distance_info', 10)
        self.timer = self.create_timer(0.1, self.detect_objects)  # 10 Hz
        self.model = YOLO('/home/sail/Desktop/danel_gadya_sail_2025_git/FinalVersionWeights.pt')
        self.init_camera()

    def init_camera(self):
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 10
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Camera opening failed with error: {err}")
            exit(1)
        self.runtime_parameters = sl.RuntimeParameters()
        self.runtime_parameters.confidence_threshold = 100
        self.runtime_parameters.texture_confidence_threshold = 100
        self.image = sl.Mat()
        self.point_cloud = sl.Mat()

    def detect_objects(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
            frame = self.image.get_data()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            height, width, _ = frame.shape
            left_cut = int(width * 0.05)
            frame_cropped = frame[:, left_cut:]
            frame_resized = cv2.resize(frame, (640, 640))
            try:
                with open("/tmp/current_task.txt", "r") as f:
                    task_name = f.read().strip()
                    conf_threshold = TASK_CONF_MAP.get(task_name, 0.2)
            except:
                conf_threshold = 0.2

            results = self.model.predict(source=frame_cropped, conf=conf_threshold)

            if results:
                for img_results in results:
                    annotator = Annotator(frame)
                    for box in img_results.boxes:
                        b = box.xyxy[0].clone()
                        b[0] += left_cut
                        b[2] += left_cut
                        c = box.cls
                        object_label = self.model.names[int(c)]
                        annotator.box_label(b, object_label)
                        x = torch.round((b[0] + b[2]) / 2).item()
                        y = torch.round((b[1] + b[3]) / 2).item()
                        err, point_cloud_value = self.point_cloud.get_value(x, y)

                        msg = ObjectDistanceInfo()
                        msg.label = object_label
                        msg.distance_x = point_cloud_value[0]
                        msg.distance_y = point_cloud_value[1]
                        msg.distance_z = point_cloud_value[2]
                        self.publisher_.publish(msg)

                        # ✅ Safe JSON float values only
                        if all(map(math.isfinite, [point_cloud_value[0], point_cloud_value[2]])):
                            try:
                                requests.post(
                                    server_url,
                                    json={
                                        "x": point_cloud_value[0],
                                        "y": point_cloud_value[2],
                                        "label": object_label
                                    }
                                )
                            except Exception as e:
                                self.get_logger().warn(f"Cannot connect to the server: {e}")
                        else:
                            self.get_logger().warn(f"Skipped invalid float values: {point_cloud_value}")

                    end_msg = ObjectDistanceInfo()
                    end_msg.label = 'end'
                    end_msg.distance_x = 0.0
                    end_msg.distance_y = 0.0
                    end_msg.distance_z = 0.0
                    self.publisher_.publish(end_msg)
                    frame = annotator.result()

            cv2.imshow("Image", frame_cropped)
            try:
                small_frame = cv2.resize(frame_cropped, (320, 320))
                cv2.imwrite("/tmp/zed_view.jpg", small_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            except Exception as e:
                self.get_logger().warn(f"❌ Failed to save zed_view: {e}")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
