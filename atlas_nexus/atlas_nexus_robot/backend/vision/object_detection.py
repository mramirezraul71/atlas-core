"""
ATLAS NEXUS - Object Detection con YOLOv8
"""

import cv2
import numpy as np
try:
    # Optional dependency. If missing, backend must still boot.
    from ultralytics import YOLO  # type: ignore
except Exception:  # pragma: no cover
    YOLO = None
import logging
from typing import List, Dict, Optional
from pathlib import Path

logger = logging.getLogger(__name__)


class ObjectDetector:
    """Detector de objetos usando YOLOv8"""
    
    def __init__(self, model_name: str = "yolov8n.pt"):
        """
        Args:
            model_name: Modelo YOLO (yolov8n.pt = nano, más rápido)
        """
        self.model_name = model_name
        self.model = None
        self.is_loaded = False
        
        # Clases COCO en español
        self.class_names_es = {
            0: 'persona', 1: 'bicicleta', 2: 'auto', 3: 'moto',
            5: 'bus', 7: 'camion', 15: 'pajaro', 16: 'gato', 
            17: 'perro', 24: 'mochila', 26: 'paraguas', 39: 'botella',
            40: 'copa', 41: 'taza', 42: 'tenedor', 43: 'cuchillo',
            44: 'cuchara', 45: 'bowl', 46: 'banana', 47: 'manzana',
            56: 'silla', 57: 'sofa', 58: 'planta', 59: 'cama',
            62: 'tv', 63: 'laptop', 64: 'mouse', 65: 'teclado',
            66: 'celular', 67: 'microondas', 73: 'libro', 76: 'tijeras'
        }
    
    def load_model(self) -> bool:
        """Carga el modelo YOLO"""
        if YOLO is None:
            logger.warning("YOLO deshabilitado: falta ultralytics. Instala 'ultralytics' para detección.")
            self.is_loaded = False
            self.model = None
            return False
        try:
            logger.info(f"Cargando modelo YOLO {self.model_name}...")
            self.model = YOLO(self.model_name)
            self.is_loaded = True
            logger.info("✓ Modelo YOLO cargado exitosamente")
            return True
        except Exception as e:
            logger.error(f"Error cargando modelo: {e}")
            self.is_loaded = False
            return False
    
    def detect(self, frame: np.ndarray, confidence: float = 0.5) -> List[Dict]:
        """
        Detecta objetos en un frame
        
        Returns:
            Lista de detecciones: [{
                'class': 'persona',
                'confidence': 0.95,
                'bbox': [x1, y1, x2, y2],
                'center': [cx, cy]
            }]
        """
        if not self.is_loaded:
            if not self.load_model():
                return []
        
        try:
            results = self.model(frame, conf=confidence, verbose=False)
            detections = []
            
            for result in results:
                boxes = result.boxes
                
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    
                    class_name = self.class_names_es.get(cls, f"objeto_{cls}")
                    
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    
                    detection = {
                        'class': class_name,
                        'class_id': cls,
                        'confidence': round(conf, 2),
                        'bbox': [int(x1), int(y1), int(x2), int(y2)],
                        'center': [cx, cy],
                        'area': int((x2 - x1) * (y2 - y1))
                    }
                    
                    detections.append(detection)
            
            return detections
            
        except Exception as e:
            logger.error(f"Error en detección: {e}")
            return []
    
    def draw_detections(self, frame: np.ndarray, detections: List[Dict]) -> np.ndarray:
        """Dibuja las detecciones sobre el frame"""
        frame_draw = frame.copy()
        
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            class_name = det['class']
            confidence = det['confidence']
            
            # Color según confianza
            if confidence > 0.8:
                color = (0, 255, 0)  # Verde
            elif confidence > 0.6:
                color = (0, 165, 255)  # Naranja
            else:
                color = (0, 0, 255)  # Rojo
            
            # Dibujar bbox
            cv2.rectangle(frame_draw, (x1, y1), (x2, y2), color, 2)
            
            # Label
            label = f"{class_name} {confidence:.0%}"
            label_size, _ = cv2.getTextSize(label, 0, 0.5, 2)
            
            # Fondo del label
            cv2.rectangle(
                frame_draw,
                (x1, y1 - label_size[1] - 10),
                (x1 + label_size[0], y1),
                color,
                -1
            )
            
            # Texto
            cv2.putText(
                frame_draw,
                label,
                (x1, y1 - 5),
                0,
                0.5,
                (255, 255, 255),
                2
            )
        
        return frame_draw
    
    def get_summary(self, detections: List[Dict]) -> Dict:
        """Genera resumen de detecciones"""
        if not detections:
            return {
                'total': 0,
                'by_class': {},
                'highest_confidence': None
            }
        
        by_class = {}
        for det in detections:
            class_name = det['class']
            by_class[class_name] = by_class.get(class_name, 0) + 1
        
        highest = max(detections, key=lambda x: x['confidence'])
        
        return {
            'total': len(detections),
            'by_class': by_class,
            'highest_confidence': highest
        }


# Singleton
_detector: Optional[ObjectDetector] = None

def get_detector() -> ObjectDetector:
    global _detector
    if _detector is None:
        _detector = ObjectDetector()
    return _detector
