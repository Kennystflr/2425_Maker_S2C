#!/usr/bin/env python3
"""Suivi de chemin ArUco avec PiCamera

Ce script détecte des balises ArUco successives pour guider un robot mobile.
Il calcule en temps réel :
  • une direction absolue 0–360° (convertie sur 8 bits 0–255)
  • une vitesse normalisée 0–255 (8 bits)

La paire d’octets est envoyée sur un port série pour piloter les roues.

Matériel / dépendances :
  - Raspberry Pi + PiCamera (ou PiCamera2, à adapter)
  - OpenCV (contrib) ≥ 4.0 : sudo apt install python3-opencv
  - numpy, pyserial

Méthode :
  1. Détecter la balise ArUco correspondant au prochain point du chemin.
  2. Calculer le vecteur image → balise ; en déduire l’angle robot.
  3. Adapter la vitesse selon la taille apparente de la balise (≈ distance).
  4. Envoyer les deux octets [direction, vitesse] sur /dev/ttyACM0 (115200 bauds).
  5. Quand on est suffisamment proche, passer au point suivant.

Notes :
  • Calibrez l’optique et placez « calibration.npz » (cameraMatrix, distCoeffs)
    pour la pose 3D si vous en avez besoin.
  • Modifiez PATH_IDS pour décrire votre itinéraire.
  • Adaptez la condition « marker_size_px > 200 » au rayon de proximité désiré.
"""

import cv2
import numpy as np
import time
import math
import os
from picamera.array import PiRGBArray
from picamera import PiCamera
import serial

# ———————————————————————————————————————————————————————————
# Paramètres
# ———————————————————————————————————————————————————————————

SERIAL_PORT = "/dev/ttyACM0"     # Port série vers contrôleur de moteurs (modifié)
BAUDRATE    = 115200            # Débit (bit/s)
ARUCO_DICT  = cv2.aruco.DICT_4X4_50  # Dictionnaire ArUco
PATH_IDS    = [0, 1, 2, 3, 4]   # Séquence d’ID à suivre
SEARCH_TURN = 30                # Taux de rotation (°) pendant la recherche
MIN_SPEED   = 0.2               # Vitesse mini quand la balise est loin

# ———————————————————————————————————————————————————————————
# Utilitaires de conversion
# ———————————————————————————————————————————————————————————

def angle_to_byte(angle_deg: float) -> int:
    """Convertit un angle 0–360° vers 0–255 (8 bits)."""
    angle_deg %= 360.0
    return int(angle_deg / 360.0 * 255)

def speed_to_byte(speed_norm: float) -> int:
    """Convertit une vitesse normalisée 0–1 vers 0–255 (8 bits)."""
    speed_norm = max(0.0, min(speed_norm, 1.0))
    return int(speed_norm * 255)

def send_command(ser, direction_deg: float, speed_norm: float):
    """Envoie deux octets [dir, speed] sur le port série."""
    packet = bytes((angle_to_byte(direction_deg), speed_to_byte(speed_norm)))
    ser.write(packet)

# ———————————————————————————————————————————————————————————
# Vision : détection et navigation
# ———————————————————————————————————————————————————————————

def vec_angle(x: float, y: float) -> float:
    """Angle (°) entre le vecteur (x,y) et l’axe avant du robot (vers +y)."""
    angle_rad = math.atan2(x, y)  # Attention : argument inverse pour atan2
    angle_deg = math.degrees(angle_rad)
    if angle_deg < 0:
        angle_deg += 360
    return angle_deg

def load_calibration(path="calibration.npz"):
    if os.path.exists(path):
        data = np.load(path)
        return data["cameraMatrix"], data["distCoeffs"]
    return None, None

# ———————————————————————————————————————————————————————————
# Boucle principale
# ———————————————————————————————————————————————————————————

def main():
    # Initialisations hardwares
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.05)
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 30
    raw = PiRGBArray(camera, size=camera.resolution)
    time.sleep(0.2)  # Laisser le temps au capteur

    aruco_dict  = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    parameters  = cv2.aruco.DetectorParameters_create()

    cam_matrix, dist_coeffs = load_calibration()

    current_goal_idx = 0

    for frame in camera.capture_continuous(raw, format="bgr", use_video_port=True):
        img  = frame.array
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            ids = ids.flatten()
            goal_id = PATH_IDS[current_goal_idx]

            if goal_id in ids:
                i = np.where(ids == goal_id)[0][0]
                marker_corners = corners[i][0]  # (4,2)

                # Centre de la balise dans l’image
                cx = marker_corners[:, 0].mean()
                cy = marker_corners[:, 1].mean()

                # Vecteur image → balise, référentiel image : origine centre
                vx = cx - img.shape[1] / 2
                vy = (img.shape[0] / 2) - cy  # inverser y : haut positif → avant

                direction_deg = vec_angle(vx, vy)

                # Taille apparente : diagonale px
                diag_px = np.linalg.norm(marker_corners[0] - marker_corners[2])
                speed_norm = np.clip(1.0 - diag_px / 400.0, MIN_SPEED, 1.0)

                send_command(ser, direction_deg, speed_norm)

                # Passage à la prochaine balise quand on est proche
                if diag_px > 200 and current_goal_idx < len(PATH_IDS) - 1:
                    current_goal_idx += 1
            else:
                # Balise cible non vue → rotation lente
                send_command(ser, SEARCH_TURN, MIN_SPEED)
        else:
            # Aucune balise → arrêt
            send_command(ser, 0, 0)

        raw.truncate(0)  # Réinitialiser le buffer pour la trame suivante

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # Arrêt d’urgence
        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.05) as ser:
            send_command(ser, 0, 0)
        print("Interrompu par l’utilisateur.")
