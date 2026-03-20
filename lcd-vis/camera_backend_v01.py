import cv2
import mediapipe as mp
from picamera2 import Picamera2

# Initialize MediaPipe Face Detection
mp_face = mp.solutions.face_detection
face_detection = mp_face.FaceDetection(model_selection=0, min_detection_confidence=0.5)
# model_selection=0 is for close-range faces (within 2m), use 1 for farther range

# Initialize Pi Camera
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(config)
picam2.start()

print("Face tracker running — press 'q' to quit")

while True:
    # Picamera2 gives us RGB directly
    frame = picam2.capture_array()

    frame_h, frame_w, _ = frame.shape
    frame_center_x = frame_w // 2
    frame_center_y = frame_h // 2

    # MediaPipe expects RGB, which Picamera2 already gives us
    results = face_detection.process(frame)

    # Convert to BGR for OpenCV display
    display_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    if results.detections:
        # Get the first (most confident) face
        detection = results.detections[0]
        bbox = detection.location_data.relative_bounding_box

        # Convert relative coordinates to pixel values
        x = int(bbox.xmin * frame_w)
        y = int(bbox.ymin * frame_h)
        w = int(bbox.width * frame_w)
        h = int(bbox.height * frame_h)

        # Face center
        face_center_x = x + w // 2
        face_center_y = y + h // 2

        # How far off-center the face is (send this to motors)
        offset_x = face_center_x - frame_center_x  # negative = left, positive = right
        offset_y = face_center_y - frame_center_y  # negative = up, positive = down

        # Draw bounding box
        cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Draw face center dot
        cv2.circle(display_frame, (face_center_x, face_center_y), 5, (0, 0, 255), -1)

        # Draw line from frame center to face center
        cv2.line(display_frame, (frame_center_x, frame_center_y), (face_center_x, face_center_y), (255, 0, 0), 2)

        # Direction text
        if abs(offset_x) < 50:
            direction_x = "CENTERED"
        elif offset_x < 0:
            direction_x = f"LEFT ({abs(offset_x)}px)"
        else:
            direction_x = f"RIGHT ({offset_x}px)"

        confidence = int(detection.score[0] * 100)
        cv2.putText(display_frame, f"Face: {direction_x} | Conf: {confidence}%",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    else:
        cv2.putText(display_frame, "No face detected", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Draw crosshair at frame center
    cv2.drawMarker(display_frame, (frame_center_x, frame_center_y),
                   (200, 200, 200), cv2.MARKER_CROSS, 20, 1)

    cv2.imshow("Face Tracker", display_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
face_detection.close()