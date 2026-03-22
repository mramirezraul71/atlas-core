# ATLAS-Quant — Módulo 8: Calibración Física + Voz + Test Runner
"""
Paquete de calibración y validación pre-live para ATLAS-Quant-Core.

Componentes:
  - PhysicalCalibrator  : mapeo automático de UI via YOLO + Insta360
  - VoiceFeedback       : síntesis de voz en español (pyttsx3 / espeak)
  - TestRunner          : ejecución de N ciclos paper + reporte final

Diagrama Mermaid — Flujo de Calibración Física:

.. code-block:: text

    flowchart TD
        CAM[Insta360 RTMP] -->|frames| YOLO[YOLOv8 screen_detect]
        YOLO -->|bboxes| UI[UI Element Analysis]
        UI -->|coords| MAP[(atlas_screen_map.json)]

        MAP --> LL[LiveLoop HID coords]
        MAP --> HC[HIDController calibration]

        VOICE[VoiceFeedback TTS] -->|"Apunta cámara..."| USER
        HC -->|círculo físico| ROS2[ROS2 /atlas/calibration/ack]

        subgraph Validación
            MAP --> VAL{Precisión ±8px?}
            VAL -- NO --> YOLO
            VAL -- SI --> DONE[✓ Calibración Completa]
        end

        ROS2 --> VAL
"""
from atlas_code_quant.calibration.voice_feedback import VoiceFeedback
from atlas_code_quant.calibration.physical_calibration import PhysicalCalibrator

__all__ = ["PhysicalCalibrator", "VoiceFeedback"]
