#!/usr/bin/env python3
"""Suivi de chemin **entre deux balises ArUco** (portiques) – PiCamera 2

### Protection courant/batterie
Pour éviter un appel de courant brutal :
* **Vitesse maximale** limitée à `MAX_SPEED` (0 ↔ 1).
* **Rampe d’accélération** : la variation instantanée ne dépasse pas `SPEED_STEP`
  par boucle (≈ frame).

Trame série : `[0x24, direction_byte, speed_byte, 0x00]`.
"""

import cv2
import numpy as np
import time
import math
import serial
from picamera2 import Picamera2

# ———————————————————————————————————————————————————————————
# Paramètres utilisateur
# ———————————————————————————————————————————————————————————

SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE    = 115200
ARUCO_DICT  = cv2.aruco.DICT_4X4_50
PATH_GATES  = [(0, 1), (2, 3), (4, 5)]      # (gauche, droite)

SEARCH_TURN = 30.0     # °/s
MIN_SPEED   = 0.2      # Vitesse mini normalisée
PASS_SPEED  = 0.4      # Vitesse overshoot (≤ MAX_SPEED)
MAX_SPEED   = 0.6      # Limite hard pour protéger la batterie
SPEED_STEP  = 0.05     # Rampe : Δv max par boucle
NEAR_THRESH_DIAG_PX = 200

# ———————————————————————————————————————————————————————————
# Utilitaires
# ———————————————————————————————————————————————————————————

def angle_to_byte(angle: float) -> int:
    return int((angle % 360) / 360 * 255)

def speed_to_byte(v: float) -> int:
    return int(max(0.0, min(v, 1.0)) * 255)

def send_command(ser: serial.Serial, dir_deg: float, speed: float):
    packet = bytearray([0x24, angle_to_byte(dir_deg), speed_to_byte(speed), 0x00])
    ser.write(packet)

# ———————————————————————————————————————————————————————————

def vec_angle(x, y):
    a = math.degrees(math.atan2(x, y))
    return a + 360 if a < 0 else a

# ———————————————————————————————————————————————————————————
# Boucle principale
# ———————————————————————————————————————————————————————————

def main():
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.05)
    cam = Picamera2()
    cam.configure(cam.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
    cam.start()
    time.sleep(0.2)

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    params = cv2.aruco.DetectorParameters()

    gate_idx = 0
    overshoot = False
    current_speed = 0.0
    search_direction = 1

    try:
        while True:
            start_time = time.time()
            img = cam.capture_array()
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
            ids = ids.flatten() if ids is not None else []

            left_id, right_id = PATH_GATES[gate_idx]
            left_vis = left_id in ids
            right_vis = right_id in ids

            if overshoot:
                target_speed = PASS_SPEED
                dir_deg = 0.0
                if not (left_vis or right_vis):
                    overshoot = False
                    if gate_idx < len(PATH_GATES) - 1:
                        gate_idx += 1
            elif left_vis and right_vis:
                li = np.where(ids == left_id)[0][0]
                ri = np.where(ids == right_id)[0][0]
                lc, rc = corners[li][0], corners[ri][0]
                if lc.mean(axis=0)[0] < rc.mean(axis=0)[0]:  # Validate left-right order
                    midx, midy = np.vstack((lc, rc)).mean(axis=0)
                    dir_deg = vec_angle(midx - img.shape[1]/2, img.shape[0]/2 - midy)
                    d_mean = (np.linalg.norm(lc[0]-lc[2]) + np.linalg.norm(rc[0]-rc[2]))/2
                    target_speed = np.clip(1.0 - d_mean/400.0, MIN_SPEED, MAX_SPEED)
                    if d_mean < NEAR_THRESH_DIAG_PX:
                        overshoot = True
                else:
                    dir_deg = SEARCH_TURN * search_direction
                    target_speed = MIN_SPEED
                    search_direction *= -1
            else:
                dir_deg = SEARCH_TURN * search_direction
                target_speed = MIN_SPEED
                search_direction *= -1

            # Speed ramp
            if target_speed > current_speed + SPEED_STEP:
                current_speed += SPEED_STEP
            elif target_speed < current_speed - SPEED_STEP:
                current_speed -= SPEED_STEP
            else:
                current_speed = target_speed
            current_speed = max(MIN_SPEED if current_speed > 0 else 0.0, min(current_speed, MAX_SPEED))

            send_command(ser, dir_deg, current_speed)

            # Optional visualization
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
            cv2.imshow("Camera", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Dynamic sleep
            elapsed = time.time() - start_time
            sleep_time = max(0.0, 0.05 - elapsed)
            time.sleep(sleep_time)

    finally:
        send_command(ser, 0, 0)
        ser.close()
        cam.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass  # Cleanup handled in finally block