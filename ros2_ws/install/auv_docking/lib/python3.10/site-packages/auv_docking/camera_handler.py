#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from aruco_handler import ArUcoDetector
from navigation_node import ROVNavigator


class CameraHandler(Node):
    def __init__(self):
        super().__init__('camera_handler')
        
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(String, 'navigation/commands', 10)
        self.bridge = CvBridge()
        
        # Initialiser ArUco-detektor og navigator
        camera_matrix = np.array([[942.6, 0, 998.5], [0, 940.4, 483.6], [0, 0, 1]])
        dist_coeffs = np.array([-0.400, 0.210, 7.31e-3, -6.25e-3, -7.03e-2])
        self.detector = ArUcoDetector(camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
        self.navigator = ROVNavigator()

    def resize_frame(self, frame, target_height):
        # Endrer størrelsen på bildet til ønsket bredde og høyde proporsjonalt.
        ratio = target_height / frame.shape[0]
        new_width = int(frame.shape[1] * ratio)
        resized_frame = cv2.resize(frame, (new_width, target_height), interpolation=cv2.INTER_AREA)
        return resized_frame

    def image_callback(self, msg):
        # Callback for mottak av bilder fra kameraet.
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        frame = self.resize_frame(frame, 1000)
        enhanced_frame = self.detector.enhance_image(frame)
        corners, ids, rejected = self.detector.detect_markers(enhanced_frame)
        pallet_center = self.detector.calculate_pallet_center(corners, ids)

        image_center = (frame.shape[1] // 2, frame.shape[0] // 2)
        commands = self.detector.get_navigation_command(pallet_center, image_center)

        for command in commands:
            self.publisher.publish(String(data=command))

        frame = self.detector.display_markers(corners, ids, frame, pallet_center)
        undistorted_frame = self.detector.undistort_image(frame)
        cv2.imshow("ArUco: ", undistorted_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CameraHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
