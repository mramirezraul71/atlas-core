"""Atlas Code-Quant — Hardware Interface Layer.

Módulo 1: Insta360 camera, OCR pipeline, HID control, ROS2 bridge, Serial controller.
Optimizado para NVIDIA Jetson Orin Nano (ARM64 + CUDA).
"""
from .camera_interface import CameraInterface, FrameCapture, OCRResult
from .control_interface import HIDController, TradierWebController
from .ros2_bridge import ATLASRos2Bridge, Ros2TradingNode
from .serial_controller import ATLASSerialController, SerialDevice, ActuatorState

__all__ = [
    "CameraInterface",
    "FrameCapture",
    "OCRResult",
    "HIDController",
    "TradierWebController",
    "ATLASRos2Bridge",
    "Ros2TradingNode",
    "ATLASSerialController",
    "SerialDevice",
    "ActuatorState",
]
