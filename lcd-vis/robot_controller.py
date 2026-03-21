"""
robot_controller.py  —  runs on the Raspberry Pi
Reads camera, runs vision, decides a movement command, and sends it over USB
serial to the Pico running movement_listener.py.

Serial protocol (Pi → Pico):  one letter + 5-digit zero-padded speed + newline
  S00000   stop
  F20000   move forward at duty 20000
  L15000   spin left  (CCW) at duty 15000
  R15000   spin right (CW)  at duty 15000
  X12000   search spin (no human visible) at duty 12000
"""

import cv2
import mediapipe as mp
import face_recognition
import numpy as np
import os
import serial                        # pip install pyserial
from picamera2 import Picamera2

# ── Serial port to Pico ────────────────────────────────────────────────────────
PICO_PORT  = "/dev/ttyACM0"          # change to /dev/ttyUSB0 if needed
PICO_BAUD  = 115200

# ── Vision config ──────────────────────────────────────────────────────────────
FRAME_W, FRAME_H     = 640, 480
SHOULDER_WIDTH_MM    = 400
FOCAL_LENGTH_PX      = 493
OBSTACLE_FLOW_THRESH = 3.0
OBSTACLE_ZONE_Y      = int(FRAME_H * 0.5)
FACE_RECOG_EVERY_N   = 5
KNOWN_FACES_DIR      = os.path.join(os.path.dirname(__file__), "known_faces")

# ── Behaviour tuning ───────────────────────────────────────────────────────────
TARGET_DISTANCE_MM  = 1000   # stop when human is ~1 m away
MIN_DISTANCE_MM     = 600    # back off if closer than this
TURN_THRESHOLD_PX   = 60     # pixels of offset before we bother turning
TURN_SPEED          = 15000  # duty for spin-in-place to face human
MOVE_SPEED          = 20000  # duty for driving toward human
SEARCH_SPEED        = 12000  # duty for slow search spin

# ── Load known faces ───────────────────────────────────────────────────────────
known_encodings, known_names = [], []
if os.path.isdir(KNOWN_FACES_DIR):
    for fname in sorted(os.listdir(KNOWN_FACES_DIR)):
        if fname.lower().endswith((".jpg", ".jpeg", ".png")):
            img  = face_recognition.load_image_file(os.path.join(KNOWN_FACES_DIR, fname))
            encs = face_recognition.face_encodings(img)
            if encs:
                known_encodings.append(encs[0])
                known_names.append(os.path.splitext(fname)[0])
                print(f"  Loaded face: {fname}")

# ── MediaPipe Pose ─────────────────────────────────────────────────────────────
mp_pose    = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
pose = mp_pose.Pose(
    model_complexity=0,
    enable_segmentation=False,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
)

# ── Camera ─────────────────────────────────────────────────────────────────────
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"size": (FRAME_W, FRAME_H), "format": "RGB888"}
))
picam2.start()

# ── Serial ─────────────────────────────────────────────────────────────────────
ser = serial.Serial(PICO_PORT, PICO_BAUD, timeout=0)
print(f"  Serial open: {PICO_PORT}")

def send_cmd(letter: str, speed: int):
    """Send a command to the Pico, e.g. send_cmd('F', 20000) → 'F20000\n'"""
    msg = f"{letter}{speed:05d}\n"
    ser.write(msg.encode())

# ── Decision logic ─────────────────────────────────────────────────────────────
def decide(obstacle: bool, human: bool, distance_mm, offset_x: int):
    """
    Priority order:
      1. Obstacle detected           → STOP  (safety first)
      2. No human in frame           → SEARCH (slow spin to find one)
      3. Human off-centre            → TURN to face them
      4. Human too far               → FORWARD to get closer
      5. Human too close             → STOP (already close enough / back off)
      6. Everything OK               → STOP
    """
    if obstacle:
        return 'S', 0

    if not human:
        return 'X', SEARCH_SPEED

    # Human visible — turn to face them first
    if offset_x > TURN_THRESHOLD_PX:          # person is right of centre
        return 'R', TURN_SPEED
    if offset_x < -TURN_THRESHOLD_PX:         # person is left of centre
        return 'L', TURN_SPEED

    # Centred — now manage distance
    if distance_mm is not None:
        if distance_mm > TARGET_DISTANCE_MM:
            return 'F', MOVE_SPEED
        if distance_mm < MIN_DISTANCE_MM:
            return 'S', 0                     # too close — stop (add 'B' back-up if needed)

    return 'S', 0

# ── State ──────────────────────────────────────────────────────────────────────
prev_gray   = None
frame_count = 0
face_labels = []
last_cmd    = None

print("Robot controller running — press 'q' to quit")

# ── Main loop ──────────────────────────────────────────────────────────────────
while True:
    frame_rgb = picam2.capture_array()
    frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    gray      = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    frame_count += 1
    cx, cy = FRAME_W // 2, FRAME_H // 2

    # 1. Obstacle (optical flow) ────────────────────────────────────────────────
    obstacle = False
    if prev_gray is not None:
        flow = cv2.calcOpticalFlowFarneback(
            prev_gray, gray, None,
            pyr_scale=0.5, levels=3, winsize=15,
            iterations=3, poly_n=5, poly_sigma=1.2, flags=0,
        )
        mag, _ = cv2.cartToPolar(
            flow[OBSTACLE_ZONE_Y:, :, 0],
            flow[OBSTACLE_ZONE_Y:, :, 1],
        )
        if float(np.mean(mag)) > OBSTACLE_FLOW_THRESH:
            obstacle = True
    prev_gray = gray.copy()

    # 2. Human / Pose ───────────────────────────────────────────────────────────
    results     = pose.process(frame_rgb)
    human       = False
    distance_mm = None
    offset_x    = 0
    offset_y    = 0

    if results.pose_landmarks:
        human = True
        lm    = results.pose_landmarks.landmark
        mp_drawing.draw_landmarks(
            frame_bgr, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
            mp_drawing.DrawingSpec(color=(0, 255, 0),   thickness=2, circle_radius=2),
            mp_drawing.DrawingSpec(color=(0, 128, 255), thickness=2),
        )
        ls = lm[mp_pose.PoseLandmark.LEFT_SHOULDER]
        rs = lm[mp_pose.PoseLandmark.RIGHT_SHOULDER]
        if ls.visibility > 0.5 and rs.visibility > 0.5:
            shoulder_px = abs(int(rs.x * FRAME_W) - int(ls.x * FRAME_W))
            if shoulder_px > 5:
                distance_mm = int((SHOULDER_WIDTH_MM * FOCAL_LENGTH_PX) / shoulder_px)
        nose     = lm[mp_pose.PoseLandmark.NOSE]
        body_cx  = int(nose.x * FRAME_W)
        body_cy  = int(nose.y * FRAME_H)
        offset_x = body_cx - cx
        offset_y = body_cy - cy

    # 3. Face recognition ───────────────────────────────────────────────────────
    if frame_count % FACE_RECOG_EVERY_N == 0:
        small     = cv2.resize(frame_rgb, (0, 0), fx=0.5, fy=0.5)
        face_locs = face_recognition.face_locations(small, model="hog")
        face_encs = face_recognition.face_encodings(small, face_locs)
        face_labels = []
        for (top, right, bottom, left), enc in zip(face_locs, face_encs):
            name = "Unknown"
            if known_encodings:
                dists = face_recognition.face_distance(known_encodings, enc)
                best  = int(np.argmin(dists))
                if dists[best] < 0.55:
                    name = known_names[best]
            face_labels.append((top*2, right*2, bottom*2, left*2, name))

    for (top, right, bottom, left, name) in face_labels:
        color = (255, 200, 0) if name != "Unknown" else (120, 120, 255)
        cv2.rectangle(frame_bgr, (left, top), (right, bottom), color, 2)
        cv2.putText(frame_bgr, name, (left, top - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    # 4. Decide + send serial command ───────────────────────────────────────────
    cmd, speed = decide(obstacle, human, distance_mm, offset_x)
    if (cmd, speed) != last_cmd:           # only send when command changes
        send_cmd(cmd, speed)
        last_cmd = (cmd, speed)

    # 5. HUD ────────────────────────────────────────────────────────────────────
    cv2.drawMarker(frame_bgr, (cx, cy), (200, 200, 200), cv2.MARKER_CROSS, 20, 1)

    if human:
        dist_str = f"{distance_mm/1000:.2f} m" if distance_mm else "dist: ?"
        cv2.putText(frame_bgr, f"Human | {dist_str}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame_bgr, f"Offset  x={offset_x:+d}  y={offset_y:+d} px",
                    (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2)
    else:
        cv2.putText(frame_bgr, "No human detected",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cmd_colours = {'S': (128,128,128), 'F': (0,255,0), 'L': (255,200,0),
                   'R': (255,200,0),   'X': (200,200,0)}
    cv2.putText(frame_bgr, f"CMD: {cmd}  spd: {speed}",
                (10, FRAME_H - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                cmd_colours.get(cmd, (200,200,200)), 2)

    if obstacle:
        cv2.rectangle(frame_bgr, (0, 0), (FRAME_W - 1, FRAME_H - 1), (0, 0, 255), 4)
        cv2.putText(frame_bgr, "!! OBSTACLE — STOPPED !!",
                    (cx - 150, FRAME_H - 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    cv2.imshow("ArtyPi Robot Controller", frame_bgr)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ── Cleanup ────────────────────────────────────────────────────────────────────
send_cmd('S', 0)
ser.close()
picam2.stop()
cv2.destroyAllWindows()
pose.close()
