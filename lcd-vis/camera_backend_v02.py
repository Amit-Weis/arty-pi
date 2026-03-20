import cv2
import mediapipe as mp
import face_recognition
import numpy as np
import os
import math
from picamera2 import Picamera2

# ── Config ─────────────────────────────────────────────────────────────────────
FRAME_W, FRAME_H     = 640, 480
SHOULDER_WIDTH_MM    = 400        # average adult shoulder width (mm)
# Pi Camera Module 3 has ~66° HFOV → focal_px ≈ (W/2) / tan(33°)
FOCAL_LENGTH_PX      = 493        # recalibrate: focal_px = (known_dist_mm * shoulder_px) / SHOULDER_WIDTH_MM

OBSTACLE_FLOW_THRESH = 3.0        # mean optical-flow magnitude (px/frame) to flag obstacle
OBSTACLE_ZONE_Y      = int(FRAME_H * 0.5)  # only watch lower half for ground-level obstacles

FACE_RECOG_EVERY_N   = 5          # run face recognition every N frames (saves CPU)
KNOWN_FACES_DIR      = os.path.join(os.path.dirname(__file__), "known_faces")
# └─ drop name.jpg files in that folder to register people

# ── Load Known Faces ──────────────────────────────────────────────────────────
known_encodings: list = []
known_names:     list = []

if os.path.isdir(KNOWN_FACES_DIR):
    for fname in sorted(os.listdir(KNOWN_FACES_DIR)):
        if fname.lower().endswith((".jpg", ".jpeg", ".png")):
            img  = face_recognition.load_image_file(os.path.join(KNOWN_FACES_DIR, fname))
            encs = face_recognition.face_encodings(img)
            if encs:
                known_encodings.append(encs[0])
                known_names.append(os.path.splitext(fname)[0])
                print(f"  Loaded face: {fname}")
else:
    print(f"  No known_faces/ folder found at {KNOWN_FACES_DIR} — recognition will label everyone Unknown")

# ── MediaPipe Pose ─────────────────────────────────────────────────────────────
mp_pose    = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
pose = mp_pose.Pose(
    model_complexity=0,            # 0 = Lite (fastest on Pi), 1 = Full, 2 = Heavy
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

# ── State ──────────────────────────────────────────────────────────────────────
prev_gray     = None
frame_count   = 0
face_labels   = []  # persisted between recog frames so display doesn't flicker

print("ArtyPi Vision running — press 'q' to quit")

# ── Main Loop ─────────────────────────────────────────────────────────────────
while True:
    frame_rgb = picam2.capture_array()
    frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    gray      = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    frame_count += 1

    cx = FRAME_W // 2
    cy = FRAME_H // 2

    # ── 1. Obstacle Detection via Optical Flow ─────────────────────────────────
    obstacle = False
    if prev_gray is not None:
        flow = cv2.calcOpticalFlowFarneback(
            prev_gray, gray, None,
            pyr_scale=0.5, levels=3, winsize=15,
            iterations=3, poly_n=5, poly_sigma=1.2, flags=0,
        )
        # Only care about the lower half (ground-level obstacles)
        mag, _ = cv2.cartToPolar(
            flow[OBSTACLE_ZONE_Y:, :, 0],
            flow[OBSTACLE_ZONE_Y:, :, 1],
        )
        if float(np.mean(mag)) > OBSTACLE_FLOW_THRESH:
            obstacle = True
    prev_gray = gray.copy()

    # ── 2. Human Detection + Distance via MediaPipe Pose ──────────────────────
    results       = pose.process(frame_rgb)
    human         = False
    distance_mm   = None
    offset_x      = 0
    offset_y      = 0

    if results.pose_landmarks:
        human = True
        lm    = results.pose_landmarks.landmark

        # Skeleton overlay
        mp_drawing.draw_landmarks(
            frame_bgr,
            results.pose_landmarks,
            mp_pose.POSE_CONNECTIONS,
            mp_drawing.DrawingSpec(color=(0, 255, 0),   thickness=2, circle_radius=2),
            mp_drawing.DrawingSpec(color=(0, 128, 255), thickness=2),
        )

        # Distance from shoulder width (pinhole model)
        ls = lm[mp_pose.PoseLandmark.LEFT_SHOULDER]
        rs = lm[mp_pose.PoseLandmark.RIGHT_SHOULDER]
        if ls.visibility > 0.5 and rs.visibility > 0.5:
            shoulder_px = abs(int(rs.x * FRAME_W) - int(ls.x * FRAME_W))
            if shoulder_px > 5:
                distance_mm = int((SHOULDER_WIDTH_MM * FOCAL_LENGTH_PX) / shoulder_px)

        # Body centre offset from frame centre (for motor targeting)
        nose     = lm[mp_pose.PoseLandmark.NOSE]
        body_cx  = int(nose.x * FRAME_W)
        body_cy  = int(nose.y * FRAME_H)
        offset_x = body_cx - cx   # negative = person left of centre
        offset_y = body_cy - cy   # negative = person above centre

    # ── 3. Face Recognition (every N frames to save CPU) ──────────────────────
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
            # Scale coords back to full frame
            face_labels.append((top*2, right*2, bottom*2, left*2, name))

    # Draw persisted face labels
    for (top, right, bottom, left, name) in face_labels:
        color = (255, 200, 0) if name != "Unknown" else (120, 120, 255)
        cv2.rectangle(frame_bgr, (left, top), (right, bottom), color, 2)
        cv2.putText(frame_bgr, name, (left, top - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    # ── 4. HUD ─────────────────────────────────────────────────────────────────
    # Crosshair
    cv2.drawMarker(frame_bgr, (cx, cy), (200, 200, 200), cv2.MARKER_CROSS, 20, 1)

    # Human / distance
    if human:
        dist_str = f"{distance_mm/1000:.2f} m" if distance_mm else "dist: ?"
        cv2.putText(frame_bgr, f"Human | {dist_str}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame_bgr, f"Offset  x={offset_x:+d}  y={offset_y:+d} px",
                    (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2)
    else:
        cv2.putText(frame_bgr, "No human detected",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Obstacle warning
    if obstacle:
        cv2.rectangle(frame_bgr, (0, 0), (FRAME_W - 1, FRAME_H - 1), (0, 0, 255), 4)
        cv2.putText(frame_bgr, "!! OBSTACLE !!",
                    (cx - 110, FRAME_H - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 3)

    cv2.imshow("ArtyPi Vision", frame_bgr)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ── Cleanup ────────────────────────────────────────────────────────────────────
picam2.stop()
cv2.destroyAllWindows()
pose.close()
