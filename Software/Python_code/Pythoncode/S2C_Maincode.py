#!/usr/bin/env python3
"""
Robot ArUco Gate Follower – v3.3
================================

### Nouveautés de la v3.3
* **Balayage plus naturel** : la direction de recherche n’est plus inversée à
  chaque frame mais toutes les `SEARCH_PERSIST` itérations (par défaut 5 →
  250 ms). Fini les micro‑zigzags.
* Limiteur déterministe, séquence de portiques, trame binaire 4 octets et
  fermeture série restent inchangés.

```bash
pip install opencv-python pyserial numpy picamera2
python robot_aruco_controller.py --port /dev/ttyUSB0 --res 640 480
```
"""

from __future__ import annotations

import argparse
import time
from typing import List, Tuple

import cv2
import numpy as np
import serial
from picamera2 import Picamera2

# ---------------------------------------------------------------------------
# Constantes utilisateur & contrôle
# ---------------------------------------------------------------------------
LOOP_DT = 0.05              # 50 ms → 20 Hz
DIR_RAMP_DPS = 180          # °/s
SPD_RAMP_PWMS = 300         # PWM unités / s

# Séquence de portiques : (id_gauche, id_droite)
PATH_GATES: List[Tuple[int, int]] = [(0, 1), (2, 3), (4, 5)]

# Logique porte / vitesse
SEARCH_TURN = 30            # ° d’angle fixe en recherche
SEARCH_PERSIST = 5          # nb de cycles avant inversion (5 × 50 ms = 250 ms)
MIN_SPEED   = 80            # PWM mini (sur 0–255) en recherche
PASS_SPEED  = 200           # PWM de croisière quand porte visible
NEAR_THRESH_DIAG_PX = 200   # On considère la porte franchie sous ce seuil

# ---------------------------------------------------------------------------
# Ramp limiter (pas fixe ⇢ comportement déterministe)
# ---------------------------------------------------------------------------
class RampLimiter:
    """Limite la variation d’un signal à ±`max_step` par boucle."""

    def __init__(self, max_delta_per_s: float, init: float = 0.0):
        self.val = init
        self.max_step = max_delta_per_s * LOOP_DT  # Δmax par itération

    def update(self, target: float) -> float:
        delta = np.clip(target - self.val, -self.max_step, self.max_step)
        self.val += delta
        return self.val

# ---------------------------------------------------------------------------
# Liaison série binaire 4 octets
# ---------------------------------------------------------------------------
class RobotLink:
    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.02)

    def send(self, direction_deg: int, speed: int) -> None:
        dir_byte = int(np.clip(direction_deg + 128, 0, 255))
        spd_byte = int(np.clip(speed, 0, 255))
        xor_byte = dir_byte ^ spd_byte
        self.ser.write(bytearray([0x24, dir_byte, spd_byte, xor_byte]))

# ---------------------------------------------------------------------------
# Suivi ArUco par PiCamera2
# ---------------------------------------------------------------------------
class PiCamArucoFollower:
    def __init__(self, port: str, res: tuple[int, int]):
        # Camera -------------------------------------------------------------
        self.picam = Picamera2()
        w, h = res
        self.picam.configure(self.picam.create_preview_configuration(
            main={"format": "RGB888", "size": (w, h)}))
        self.picam.start()
        self.cx = w / 2

        # ArUco --------------------------------------------------------------
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
            cv2.aruco.DetectorParameters())

        # Contrôle -----------------------------------------------------------
        self.link = RobotLink(port)
        self.dir_ramp = RampLimiter(DIR_RAMP_DPS)
        self.spd_ramp = RampLimiter(SPD_RAMP_PWMS)

        # Séquence de portes -------------------------------------------------
        self.gate_idx = 0
        self.passed_current = False

        # Recherche lorsque la porte est perdue -----------------------------
        self.search_dir = 1               # +1 → droite, −1 → gauche
        self.search_counter = SEARCH_PERSIST

    # ------------------------------------------------------------------
    @staticmethod
    def _gate_center(corners_left, corners_right):
        lc = corners_left.mean(axis=1).squeeze()
        rc = corners_right.mean(axis=1).squeeze()
        return np.array([(lc[0] + rc[0]) / 2, (lc[1] + rc[1]) / 2])

    # ------------------------------------------------------------------
    def _update_search_dir(self):
        """Incrémente le compteur et inverse `search_dir` quand il atteint 0."""
        self.search_counter -= 1
        if self.search_counter <= 0:
            self.search_dir *= -1
            self.search_counter = SEARCH_PERSIST

    # ------------------------------------------------------------------
    def run(self):
        try:
            while True:
                loop_start = time.time()

                # -------- Capture image
                frame = self.picam.capture_array()
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                corners, ids, _ = self.detector.detectMarkers(gray)
                ids = ids.flatten() if ids is not None else np.array([])

                # -------- Logique de séquence
                target_dir = 0
                target_spd = 0

                if self.gate_idx >= len(PATH_GATES):
                    # Toutes les portes franchies : stop
                    target_spd = 0
                    target_dir = 0
                else:
                    left_id, right_id = PATH_GATES[self.gate_idx]
                    left_vis = left_id in ids
                    right_vis = right_id in ids

                    if self.passed_current:
                        if not (left_vis or right_vis):  # porte disparue ➜ suivante
                            self.passed_current = False
                            self.gate_idx += 1
                            # reset recherche pour prochaine porte
                            self.search_counter = SEARCH_PERSIST
                            self.search_dir = 1
                    elif left_vis and right_vis:
                        # Deux balises visibles : viser centre
                        li = np.where(ids == left_id)[0][0]
                        ri = np.where(ids == right_id)[0][0]
                        centre = self._gate_center(corners[li][0], corners[ri][0])
                        err_px = centre[0] - self.cx
                        target_dir = int((err_px / self.cx) * 90)
                        d_mean = (np.linalg.norm(corners[li][0][0]-corners[li][0][2]) +
                                  np.linalg.norm(corners[ri][0][0]-corners[ri][0][2])) / 2
                        target_spd = np.clip(int((1.0 - d_mean/400.0) * 255), MIN_SPEED, PASS_SPEED)
                        if d_mean < NEAR_THRESH_DIAG_PX:
                            self.passed_current = True
                    else:
                        # Porte non détectée : recherche persistante
                        target_dir = SEARCH_TURN * self.search_dir
                        target_spd = MIN_SPEED
                        self._update_search_dir()

                # -------- Limiteurs courant
                cmd_dir = int(self.dir_ramp.update(target_dir))
                cmd_spd = int(self.spd_ramp.update(target_spd))

                # -------- Envoi trame
                self.link.send(cmd_dir, cmd_spd)

                # -------- Debug visuel
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.line(frame, (int(self.cx), 0), (int(self.cx), frame.shape[0]), (0, 255, 0), 1)
                cv2.imshow("PiCam ArUco Follower", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                # -------- Bouclage 50 ms
                elapsed = time.time() - loop_start
                if elapsed < LOOP_DT:
                    time.sleep(LOOP_DT - elapsed)
        finally:
            # Stop moteurs, fermer série & caméra
            self.link.send(0, 0)
            self.link.ser.close()
            cv2.destroyAllWindows()
            self.picam.stop()

# ---------------------------------------------------------------------------
# Entrée CLI
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Suivi de portes ArUco séquentielles (PiCamera2, 20 Hz)")
    ap.add_argument("--port", required=True, help="Port série USB → MCU")
    ap.add_argument("--res", nargs=2, type=int, default=[640, 480], metavar=("W", "H"), help="Résolution caméra")
    args = ap.parse_args()

    follower = PiCamArucoFollower(port=args.port, res=tuple(args.res))
    follower.run()
