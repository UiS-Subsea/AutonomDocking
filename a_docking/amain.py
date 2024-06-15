import cv2
import sys
import numpy as np
from aruco_detector import ArUcoDetector
from arov_navigator import ROVNavigator

def init_camera():
    """Initialiserer og konfigurerer kameraet."""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    return cap

def resize_frame(frame, target_height):
    """Endrer størrelsen på bildet til ønsket bredde og høyde proporsjonalt."""
    ratio = target_height / frame.shape[0]
    new_width = int(frame.shape[1] * ratio)
    resized_frame = cv2.resize(frame, (new_width, target_height), interpolation=cv2.INTER_AREA)
    return resized_frame

def main():
    cap = init_camera()
    if not cap:
        sys.exit("Error: Unable to initialize camera.")
    camera_matrix = np.array([[942.6, 0, 998.5], [0, 940.4, 483.6], [0, 0, 1]]) 
    dist_coeffs = np.array([-0.400, 0.210, 7.31e-3, -6.25e-3, -7.03e-2]) 
    detector = ArUcoDetector(camera_matrix=camera_matrix, dist_coeffs=dist_coeffs) 

    navigator = ROVNavigator()

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Error: Can't read frame.")
            break
        
        frame = resize_frame(frame, 1000)
        enhanced_frame = detector.enhance_image(frame)
        corners, ids, rejected = detector.detect_markers(enhanced_frame)
        pallet_center = detector.calculate_pallet_center(corners, ids)

        image_center = (frame.shape[1] // 2, frame.shape[0] // 2)
        commands = detector.get_navigation_command(pallet_center, image_center)

        for command in commands:
            print(f"Command: {command}")

        frame = detector.display_markers(corners, ids, frame, pallet_center)
        undistorted_frame = detector.undistort_image(frame)
        cv2.imshow("ArUco: ", undistorted_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
    print("I'm good")
