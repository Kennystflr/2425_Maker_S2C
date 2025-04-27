import cv2
import cv2.aruco as aruco
import numpy as np
from picamera2 import Picamera2
from libcamera import Transform

def detect_aruco():
    # Initialiser la Picamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (640, 480)},
        transform=Transform(hflip=False, vflip=False)
    )
    picam2.configure(config)
    picam2.start()

    # Charger le dictionnaire ArUco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters()
    
    # Créer le détecteur
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    try:
        while True:
            # Capturer l'image depuis la Picamera2
            frame = picam2.capture_array()
            
            # Convertir l'image en BGR (format compatible OpenCV)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Convertir en niveaux de gris
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Détecter les marqueurs ArUco
            corners, ids, rejected = detector.detectMarkers(gray)
            
            # Dessiner les marqueurs détectés
            if ids is not None:
                aruco.drawDetectedMarkers(frame, corners, ids)
                
                # Afficher les IDs des marqueurs
                for i, corner in enumerate(corners):
                    # Position pour afficher l'ID
                    pos = corner[0][0]
                    cv2.putText(frame, str(ids[i][0]), 
                              (int(pos[0]), int(pos[1] - 10)),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Afficher l'image
            cv2.imshow("ArUco Detection", frame)
            
            # Quitter avec la touche 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        # Libérer les ressources
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_aruco()
