#!/usr/bin/env python3

import os
import sys
import time
import math
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QPolygon
from PyQt5.QtCore import Qt, QPoint, QMetaObject, QTimer, pyqtSlot
from cv_from_zed_ros2.msg import ObjectDistanceInfo  # שים לב לשם החבילה החדש

class PyQtMap(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'map_gui_viewer')
        QWidget.__init__(self)

        self.setWindowTitle("Live Object Map (ROS2 PyQt)")
        self.setGeometry(100, 100, 800, 600)
        self.setFixedSize(800, 600)

        self.object_points = []
        self.scale = 50
        self.center_x = 400
        self.center_y = 300

        self.last_screenshot_time = time.time()
        self.screenshot_interval = 0.3

        self.last_object_time = time.time()
        self.object_timeout = 0.75

        # Timers
        self.wave_offset = 0
        self.wave_timer = QTimer(self)
        self.wave_timer.timeout.connect(self.animate_waves)
        self.wave_timer.start(30)

        self.clear_timer = QTimer(self)
        self.clear_timer.timeout.connect(self.check_object_timeout)
        self.clear_timer.start(200)

        self.screenshot_timer = QTimer(self)
        self.screenshot_timer.timeout.connect(self.check_screenshot_condition)
        self.screenshot_timer.start(100)

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.spin_once)
        self.ros_timer.start(10)  # אפשר גם 5ms

        # ROS2 subscription
        self.subscription = self.create_subscription(
            ObjectDistanceInfo,
            'object_distance_info',
            self.update_data,
            10
        )

        self.show()

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.001)

    def animate_waves(self):
        self.wave_offset += 5
        if self.wave_offset > 10000:
            self.wave_offset = 0
        self.update()

    def check_object_timeout(self):
        now = time.time()
        if self.object_points and (now - self.last_object_time > self.object_timeout):
            self.object_points.clear()
            self.update()

    def check_screenshot_condition(self):
        now = time.time()
        if now - self.last_screenshot_time < self.screenshot_interval:
            return
        if not self.object_points or (now - self.last_object_time > self.screenshot_interval):
            QMetaObject.invokeMethod(self, "export_screenshot", Qt.QueuedConnection)

    def update_data(self, msg):
        try:
            if msg.label == "end":
                return
            if not math.isfinite(msg.distance_x) or not math.isfinite(msg.distance_z):
                return

            self.object_points.append((msg.distance_x, msg.distance_z, msg.label))
            self.object_points = self.object_points[-12:]

            self.last_object_time = time.time()

            QMetaObject.invokeMethod(self, "update", Qt.QueuedConnection)
            QMetaObject.invokeMethod(self, "export_screenshot", Qt.QueuedConnection)
        except Exception:
            pass

    @pyqtSlot()
    def export_screenshot(self):
        now = time.time()
        if now - self.last_screenshot_time < self.screenshot_interval:
            return

        self.last_screenshot_time = now
        pixmap = self.grab()
        if pixmap.isNull():
            return
        temp_path = "/tmp/live_gui_temp.jpg"
        final_path = "/tmp/live_gui.jpg"
        pixmap.save(temp_path, "JPG")
        os.replace(temp_path, final_path)

    def paintEvent(self, event):
        with QPainter(self) as painter:
            painter.setRenderHint(QPainter.Antialiasing)
            painter.fillRect(self.rect(), QColor("#b3ecff"))

            painter.setPen(QPen(QColor("#3399ff"), 2))
            for x in range(0, self.width(), 5):
                y = int(self.height() - 20 + 10 * math.sin((x + self.wave_offset) * 0.05))
                painter.drawLine(x, y, x, self.height())

            painter.setPen(QPen(Qt.white, 2))
            painter.setBrush(QBrush(QColor("#005577")))
            painter.drawEllipse(self.center_x - 20, self.center_y - 5, 40, 15)
            painter.drawLine(self.center_x, self.center_y - 5, self.center_x, self.center_y - 25)

            sail = QPolygon([
                QPoint(self.center_x, self.center_y - 25),
                QPoint(self.center_x + 15, self.center_y - 10),
                QPoint(self.center_x, self.center_y - 5)
            ])
            painter.setBrush(QBrush(Qt.white))
            painter.drawPolygon(sail)
            painter.setPen(QPen(Qt.black))
            painter.drawText(self.center_x - 30, self.center_y + 30, "Sail-IL-2025")

            # === FOV ===
            horizontal_fov_deg = 86.02
            half_horizontal_fov_rad = math.radians(horizontal_fov_deg / 2)
            slope_left = math.tan(half_horizontal_fov_rad) * -1
            slope_right = math.tan(half_horizontal_fov_rad)
            end_y = 0
            left_x = self.center_x + (end_y - self.center_y) / slope_left
            right_x = self.center_x + (end_y - self.center_y) / slope_right
            fov_polygon = QPolygon([
                QPoint(int(self.center_x), int(self.center_y)),
                QPoint(int(left_x), int(end_y)),
                QPoint(int(right_x), int(end_y))
            ])
            painter.setBrush(QBrush(QColor(0, 0, 125, 75)))
            painter.setPen(Qt.NoPen)
            painter.drawPolygon(fov_polygon)
            painter.setPen(QPen(QColor(255, 255, 255, 128), 1, Qt.SolidLine))
            painter.drawLine(int(self.center_x), int(self.center_y), int(left_x), int(end_y))
            painter.drawLine(int(self.center_x), int(self.center_y), int(right_x), int(end_y))
            # === END FOV ===

            for x, z, label in self.object_points:
                try:
                    if not math.isfinite(x) or not math.isfinite(z):
                        continue
                    sx = int(self.center_x + x * self.scale)
                    sy = int(self.center_y - z * self.scale)
                except:
                    continue

                painter.setPen(QPen(Qt.black, 2))
                if label in ['Dock Triangle', 'Dock Circle', 'Dock Square']:
                    painter.setBrush(QBrush(Qt.white))
                    painter.drawRect(sx - 15, sy - 15, 30, 30)
                    if label == 'Dock Triangle':
                        painter.setBrush(QBrush(Qt.red))
                        painter.drawPolygon(QPolygon([
                            QPoint(sx, sy - 10),
                            QPoint(sx - 10, sy + 10),
                            QPoint(sx + 10, sy + 10)
                        ]))
                    elif label == 'Dock Circle':
                        painter.setBrush(QBrush(QColor("#32CD32")))
                        painter.drawEllipse(sx - 10, sy - 10, 20, 20)
                    elif label == 'Dock Square':
                        painter.setBrush(QBrush(Qt.blue))
                        painter.drawRect(sx - 10, sy - 10, 20, 20)
                elif label == 'Red Ball':
                    painter.setBrush(QBrush(Qt.red))
                    painter.drawEllipse(sx - 10, sy - 10, 20, 20)
                elif label == 'Green Ball':
                    painter.setBrush(QBrush(QColor("#006400")))
                    painter.drawEllipse(sx - 10, sy - 10, 20, 20)
                elif label == 'Yellow Ball':
                    painter.setBrush(QBrush(Qt.yellow))
                    painter.drawEllipse(sx - 10, sy - 10, 20, 20)
                elif label == 'Red Buoy':
                    painter.setBrush(QBrush(Qt.red))
                    painter.drawRect(sx - 5, sy - 30, 10, 30)
                    painter.drawEllipse(sx - 10, sy, 20, 10)
                elif label == 'Green Buoy':
                    painter.setBrush(QBrush(QColor("#006400")))
                    painter.drawRect(sx - 5, sy - 30, 10, 30)
                    painter.drawEllipse(sx - 10, sy, 20, 10)

if __name__ == "__main__":
    rclpy.init()
    app = QApplication(sys.argv)
    map_window = PyQtMap()
    sys.exit(app.exec_())
