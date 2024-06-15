import numpy as np
import cv2
import sys

class ArUcoDetector: 
    """Klasse for deteksjon av ArUco-markører og utregning av midtpunktet i pallens overflate."""
    def __init__(self, aruco_type=cv2.aruco.DICT_ARUCO_ORIGINAL, camera_matrix=None, dist_coeffs=None):
        self.a_dict = cv2.aruco.getPredefinedDictionary(aruco_type) 
        self.a_params = cv2.aruco.DetectorParameters() 
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.camera_offset_y = 50  # Offset i mm (5 cm), HUSK Å JUSTERE


    def undistort_image(self, image):
         """Fjerner forvrengning fra bildet basert på kalibreringsmatrisen og forvrengningskoeffisientene."""
         h, w = image.shape[:2]
         newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w,h), 1, (w,h))
         mapx, mapy = cv2.initUndistortRectifyMap(self.camera_matrix, self.dist_coeffs, None, newcameramtx, (w,h), 5)
         dst = cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR)
         x, y, w, h = roi
         dst = dst[y:y+h, x:x+w]
         return dst

    def enhance_image(self, image):
        """Bildeforbedring ved å konvertere til gråskala, histogramutjevning og unsharp Masking.""" 
        if len(image.shape) == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = cv2.equalizeHist(image)
        gaussian_blur = cv2.GaussianBlur(image, (9, 9), 2.0)
        new_image = cv2.addWeighted(image, 1.5, gaussian_blur, -0.5, 0)
        return new_image

    def detect_markers(self, image): 
        """Detekterer ArUco-markører i det gitte bildet."""
        corners, ids, rejected = cv2.aruco.detectMarkers(image, self.a_dict, parameters=self.a_params) 
        if ids is not None:
            wanted_ids = np.array([28, 7, 19, 96]) 
            indexes = np.isin(ids.flatten(), wanted_ids)            #### ISIN ####
            filtered_corners = [corners[i] for i in range(len(corners)) if indexes[i]]
            filtered_ids = ids[indexes]
            return filtered_corners, np.array(filtered_ids), rejected
        return [], np.array([]), rejected
    

    def calculate_pallet_center(self, corners, ids):
        """Beregner midten av pallen basert på hjørnemarkørene."""
        if ids is not None and len(ids) >= 4:  # Hvis det er minst 4 markører
            center_points = [np.mean(corner[0], axis=0) for corner in corners]
            if len(center_points) == 4: 
                pallet_center = np.mean(center_points, axis=0).astype(int).tolist() 
                pallet_center[1] += self.camera_offset_y 
                return pallet_center
        return None

            
    def is_centered(self, pallet_center, image):
        """Sjekker om pallen er sentrert i bildet og gir visuell tilbakemelding."""
        if pallet_center:
            image_center = (image.shape[1] // 2, image.shape[0] // 2) 
            dx, dy = pallet_center[0] - image_center[0], pallet_center[1] - image_center[1]
            distance = np.hypot(dx, dy) 
            move_threshold = 25  # Terskler for bevegelseskommando, JUSTER ETTER BEHOV
            directions = []
            
            if abs(dx) > move_threshold or abs(dy) > move_threshold:  
                if abs(dx) > move_threshold:
                    directions.append("RIGHT" if dx > 0 else "LEFT")
                if abs(dy) > move_threshold:
                    directions.append("BACK" if dy > 0 else "FORWARD")
                cv2.putText(image, f"Distance: {distance:.2f}", (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 8) 
                cv2.putText(image, f"Distance: {distance:.2f}", (40, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2) 
            for i, direction in enumerate(directions, start=1):
                cv2.putText(image, direction, (40, 40 +i*30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 8) # der 30 er x og 30+i*20 er y
                cv2.putText(image, direction, (40, 40 +i*30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)   
            cv2.circle(image, tuple(pallet_center), 50, (0, 255, 0) if distance < 10 else (0,0,255),3) 
           

    def display_markers(self, corners, ids, image, pallet_center=None):
        """Tegner ut markørene og midten av pallen i bildet."""
        if ids is not None and len(ids) > 0:  # Legg til en sjekk for lengde
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                cv2.polylines(image, [corners.astype(int)], isClosed=True, color=(0, 255, 0), thickness=2)
                center = corners.mean(axis=0).astype(int)
                cv2.circle(image, tuple(center), 4, (0, 0, 255), -1)
                print("ArUco marker ID: {}".format(markerID))
                offset = 30
                cv2.putText(image, str(markerID), (center[0] + offset, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            if pallet_center:
                self.is_centered(pallet_center, image)
        else:
            print("No markers found")
        return image
    

def init_camera():
    """ Initialiserer og konfigurerer kameraet."""
    cap = cv2.VideoCapture(0) 
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1080)
    return cap


def resize_frame(frame, target_height):
    """Endrer størrelsen på bildet til ønsket bredde og høyde proporsjonalt."""
    ratio = target_height / frame.shape[0]
    new_width = int(frame.shape[1] * ratio)
    resized_frame = cv2.resize(frame, (new_width, target_height), interpolation=cv2.INTER_AREA)
    return resized_frame

def main():
    cap = init_camera()
    camera_matrix = np.array([[942.6, 0, 998.5], [0, 940.4, 483.6], [0, 0, 1]]) 
    dist_coeffs = np.array([-0.400, 0.210, 7.31e-3, -6.25e-3, -7.03e-2]) 
    detector = ArUcoDetector(camera_matrix=camera_matrix, dist_coeffs=dist_coeffs) 

    while cap.isOpened():
        ret, frame = cap.read() 
        if not ret:
            sys.exit("Error: Can't open camera.") 
        frame = resize_frame(frame, 1000) 
        enhanced_frame = detector.enhance_image(frame)  
        corners, ids, rejected = detector.detect_markers(enhanced_frame) 
        pallet_center = detector.calculate_pallet_center(corners, ids)
        frame = detector.display_markers(corners, ids, frame, pallet_center)
        undistorted_frame = detector.undistort_image(frame)
        cv2.imshow("ArUco: ", undistorted_frame)
        if cv2.waitKey(1) & 0xFF  == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ =="__main__":
    main()
    print("I'm good")