#!/usr/bin/env python3
import cv2
import sys

print("=== Insta360 Link 2 Test ===")
print(f"OpenCV version: {cv2.__version__}")

# Test different backends for camera 2 (Insta360)
backends = []
if hasattr(cv2, "CAP_DSHOW"):
    backends.append(("DSHOW", cv2.CAP_DSHOW))
if hasattr(cv2, "CAP_MSMF"):
    backends.append(("MSMF", cv2.CAP_MSMF))
backends.append(("ANY", cv2.CAP_ANY))

for name, backend in backends:
    print(f"\n--- Testing {name} backend ---")
    cap = cv2.VideoCapture(2, backend)
    if cap.isOpened():
        print(f"✓ Camera 2 opened with {name}")
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        print(f"  Resolution: {w}x{h}")
        print(f"  FPS: {fps}")
        
        # Try to read a frame
        ret, frame = cap.read()
        if ret and frame is not None:
            print(f"  ✓ Frame captured: {frame.shape}")
        else:
            print(f"  ✗ Failed to capture frame")
        
        cap.release()
    else:
        print(f"✗ Camera 2 failed to open with {name}")
