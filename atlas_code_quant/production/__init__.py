# ATLAS-Quant — Módulo 9: Production Safety + Telegram + Grafana
"""
Sistema de producción completo para trading autónomo seguro.

Diagrama Mermaid — Sistema FINAL integrado:

.. code-block:: text

    flowchart TD
        subgraph M1["M1 Hardware"]
            CAM[Insta360 RTMP] --> OCR[YOLOv8+EasyOCR]
            HID[HIDController] --> KB[Hotkeys Esc/F12]
            ROS2[ROS2 Bridge]
        end

        subgraph M2["M2 Pipeline"]
            WS[Tradier WebSocket] --> CVD[CVD Calculator]
            WS --> IV[IV Rank 30d]
            WS --> TECH[Technical Indicators]
        end

        subgraph M3_4["M3+M4 ML"]
            REGIME[XGBoost+LSTM\nRegime Classifier]
            SIGNAL[Signal Generator\n7-layer gate]
        end

        subgraph M5["M5 Riesgo"]
            KELLY[Kelly Risk Engine\nCircuit Breaker]
        end

        subgraph M6["M6 Healing"]
            HEAL[SelfHealingOrchestrator]
            ERR[ErrorMemoryAgent\nChromaDB]
        end

        subgraph M7["M7 Ejecución"]
            EXEC[SignalExecutor]
            LOOP[LiveLoop 5s cycle\nCPU throttle]
            SWITCH[ModeSwitcher\npaper↔live]
        end

        subgraph M8["M8 Calibración"]
            CALIB[PhysicalCalibrator\nYOLO+circle]
            VOICE[VoiceFeedback\npyttsx3 español]
            TEST[TestRunner\nSharpe+DD+WinRate]
        end

        subgraph M9["M9 Producción"]
            GUARD[ProductionGuard\n2% daily loss\n5% max position]
            TG[TelegramAlerts\nreal-time]
            GRAF[Grafana Dashboard\nPrometheus]
            ACT[LiveActivation\ndouble confirmation]
        end

        CAM --> OCR --> SIGNAL
        WS --> CVD --> SIGNAL
        TECH --> REGIME --> SIGNAL
        SIGNAL --> KELLY --> EXEC
        EXEC --> GUARD
        GUARD -->|preview=true| TRADIER[(Tradier API)]
        GUARD -->|blocked| VOICE

        HEAL --> ERR --> VOICE
        VOICE --> TG
        EXEC --> TG
        EXEC --> GRAF

        TEST -->|ready_for_live| ACT
        ACT --> GUARD
        GUARD -->|double confirm| SWITCH
        SWITCH -->|live| LOOP

        GRAF --> PROMETHEUS[(Prometheus :9090)]
        PROMETHEUS --> GRAFANA[(Grafana :3000)]
"""
from atlas_code_quant.production.production_guard import ProductionGuard
from atlas_code_quant.production.telegram_alerts import TelegramAlerts
from atlas_code_quant.production.grafana_dashboard import GrafanaDashboard

__all__ = ["ProductionGuard", "TelegramAlerts", "GrafanaDashboard"]
