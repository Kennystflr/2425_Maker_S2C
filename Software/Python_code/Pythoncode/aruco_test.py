from picamera2 import Picamera2
import cv2
import numpy as np

# Initialiser la caméra
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()

# Charger le dictionnaire Aruco
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)  # Choisis ton type ici
parameters = cv2.aruco.DetectorParameters_create()

print("Démarrage de la détection Aruco... Appuie sur 'q' pour quitter.")

while True:
    # Capture d'une image
    frame = picam2.capture_array()

    # Convertir en niveaux de gris pour la détection
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    # Détection des marqueurs
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # Si on détecte des marqueurs
    if ids is not None:
        # Dessiner les marqueurs détectés
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        for i in range(len(ids)):
            c = corners[i][0]
            # Afficher l'ID à côté du tag
            center_x = int(np.mean(c[:, 0]))
            center_y = int(np.mean(c[:, 1]))
            cv2.putText(frame, f"ID: {ids[i][0]}", (center_x - 20, center_y - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Afficher l'image
    cv2.imshow("Aruco Detection", frame)

    # Quitter avec la touche 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libérer les ressources
cv2.destroyAllWindows()
picam2.close()