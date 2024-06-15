#!/usr/bin/env python3
import sys
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from aruco_handler import ArUcoDetector
from navigation_node import ROVNavigator


class CameraHandler(Node):
    def __init__(self):
        super().__init__('camera_handler')
        self.publisher_ = self.create_publisher(Image, 'aruco_image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = self.init_camera()
        self.detector = ArUcoDetector()
        self.navigator = ROVNavigator()

    def timer_callback(self):
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                frame = self.resize_frame(frame, 1000)
                enhanced_frame = self.detector.enhance_image(frame)
                corners, ids, rejected = self.detector.detect_markers(enhanced_frame)
                pallet_center = self.detector.calculate_pallet_center(corners, ids)
                image_center = (frame.shape[1] // 2, frame.shape[0] // 2)
                commands = self.detector.get_navigation_command(pallet_center, image_center)

                for command in commands:
                    print(f"Command: {command}")

                frame = self.detector.display_markers(corners, ids, frame, pallet_center)
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher_.publish(msg)

    def init_camera(self):
        # Initialiserer og konfigurerer kameraet.
        cap = cv2.VideoCapture(0)  # Camera index 0 is main camera on device
        if not cap.isOpened():
            self.get_logger().error("Error: Could not open camera.")
            sys.exit("Error: Could not open camera.")
            #return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.get_logger().info("Camera initialized successfully.")
        return cap

    def resize_frame(self, frame, target_height):
        # Endrer størrelsen på bildet til ønsket bredde og høyde proporsjonalt.
        ratio = target_height / frame.shape[0]
        new_width = int(frame.shape[1] * ratio)
        resized_frame = cv2.resize(frame, (new_width, target_height), interpolation=cv2.INTER_AREA)
        return resized_frame  

def main(args=None):
    rclpy.init(args=args)
    camera_handler = CameraHandler()
    rclpy.spin(camera_handler)
    camera_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
