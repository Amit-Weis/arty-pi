from picamera2 import Picamera2
import cv2

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration())
picam2.start()

frame = picam2.capture_array()

if frame is not None and frame.size > 0:
    print(f"Camera OK — frame shape: {frame.shape}")
    cv2.imwrite("test_frame.jpg", frame)
    print("Saved test_frame.jpg")
else:
    print("Camera FAILED — no frame captured")

picam2.stop()
