#!/usr/bin/env python3
import cv2
import sys

print("=== Camera Detection Test ===")
print(f"OpenCV version: {cv2.__version__}")
print()

# Test camera indices 0-9
available_cameras = []
camera_details = []

for i in range(10):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        # Try to get camera properties
        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = cap.get(cv2.CAP_PROP_FPS)
        backend = cap.getBackendName()
        
        available_cameras.append(i)
        camera_details.append({
            'index': i,
            'width': width,
            'height': height,
            'fps': fps,
            'backend': backend
        })
        print(f"✓ Camera {i}: Available")
        print(f"  Resolution: {width}x{height}")
        print(f"  FPS: {fps}")
        print(f"  Backend: {backend}")
        print()
        cap.release()
    else:
        print(f"✗ Camera {i}: Not available")

print(f"\nTotal cameras found: {len(available_cameras)}")
print(f"Camera indices: {available_cameras}")

# Try to capture a test frame from each camera
print("\n=== Test Frame Capture ===")
for cam in camera_details:
    idx = cam['index']
    cap = cv2.VideoCapture(idx)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret and frame is not None:
            print(f"✓ Camera {idx}: Frame captured successfully ({frame.shape})")
        else:
            print(f"✗ Camera {idx}: Failed to capture frame")
        cap.release()
