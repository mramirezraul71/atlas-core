#!/usr/bin/env python3
"""
YOLO Object Detection Module for ATLAS NEXUS Robot
Real-time object detection using YOLOv8
"""

import cv2
import numpy as np
from ultralytics import YOLO
import torch
from typing import List, Dict, Tuple, Optional
import logging
from pathlib import Path
import json
from datetime import datetime

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class YOLODetector:
    """YOLO Object Detection System for ATLAS NEXUS"""
    
    def __init__(self, model_path: Optional[str] = None, confidence_threshold: float = 0.5):
        """
        Initialize YOLO Detector
        
        Args:
            model_path: Path to custom YOLO model (optional, uses YOLOv8n by default)
            confidence_threshold: Minimum confidence for detections
        """
        self.confidence_threshold = confidence_threshold
        self.model = None
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.detection_history = []
        self.max_history = 100
        
        # Load model
        self._load_model(model_path)
        
        # COCO class names (default YOLO classes)
        self.class_names = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
            'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
            'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
            'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
            'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
            'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
            'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
            'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
            'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
            'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier',
            'toothbrush'
        ]
        
        logger.info(f"YOLO Detector initialized on {self.device}")
    
    def _load_model(self, model_path: Optional[str] = None):
        """Load YOLO model"""
        try:
            if model_path and Path(model_path).exists():
                self.model = YOLO(model_path)
                logger.info(f"Loaded custom model: {model_path}")
            else:
                # Use default YOLOv8n model (nano version, fastest)
                self.model = YOLO('yolov8n.pt')
                logger.info("Loaded YOLOv8n model (nano)")
                
            # Move model to device
            self.model.to(self.device)
            
        except Exception as e:
            logger.error(f"Failed to load YOLO model: {e}")
            raise
    
    def detect_objects(self, image: np.ndarray) -> List[Dict]:
        """
        Detect objects in image
        
        Args:
            image: Input image (numpy array)
            
        Returns:
            List of detection dictionaries
        """
        if self.model is None:
            logger.error("Model not loaded")
            return []
        
        try:
            # Run YOLO inference
            results = self.model(image, conf=self.confidence_threshold, verbose=False)
            
            detections = []
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Extract detection info
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        confidence = box.conf[0].cpu().numpy()
                        class_id = int(box.cls[0].cpu().numpy())
                        
                        # Get class name
                        class_name = self.class_names[class_id] if class_id < len(self.class_names) else f"class_{class_id}"
                        
                        detection = {
                            'bbox': [int(x1), int(y1), int(x2), int(y2)],
                            'confidence': float(confidence),
                            'class_id': class_id,
                            'class_name': class_name,
                            'center': [int((x1 + x2) / 2), int((y1 + y2) / 2)],
                            'area': int((x2 - x1) * (y2 - y1))
                        }
                        
                        detections.append(detection)
            
            # Store in history
            self._add_to_history(detections)
            
            logger.info(f"Detected {len(detections)} objects")
            return detections
            
        except Exception as e:
            logger.error(f"Detection failed: {e}")
            return []
    
    def draw_detections(self, image: np.ndarray, detections: List[Dict]) -> np.ndarray:
        """
        Draw detection boxes and labels on image
        
        Args:
            image: Input image
            detections: List of detection dictionaries
            
        Returns:
            Image with drawn detections
        """
        annotated_image = image.copy()
        
        for detection in detections:
            bbox = detection['bbox']
            confidence = detection['confidence']
            class_name = detection['class_name']
            
            # Color based on class
            color = self._get_class_color(detection['class_id'])
            
            # Draw bounding box
            cv2.rectangle(annotated_image, 
                         (bbox[0], bbox[1]), 
                         (bbox[2], bbox[3]), 
                         color, 2)
            
            # Draw label
            label = f"{class_name}: {confidence:.2f}"
            label_size = cv2.getTextSize(label, 0, 0.5, 2)[0]
            
            # Background for label
            cv2.rectangle(annotated_image,
                         (bbox[0], bbox[1] - label_size[1] - 10),
                         (bbox[0] + label_size[0], bbox[1]),
                         color, -1)
            
            # Text
            cv2.putText(annotated_image, label,
                       (bbox[0], bbox[1] - 5),
                       0, 0.5, (255, 255, 255), 2)
        
        return annotated_image
    
    def _get_class_color(self, class_id: int) -> Tuple[int, int, int]:
        """Get color for class ID"""
        colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255),
            (0, 255, 255), (128, 0, 128), (255, 165, 0), (0, 128, 128), (128, 128, 0)
        ]
        return colors[class_id % len(colors)]
    
    def _add_to_history(self, detections: List[Dict]):
        """Add detections to history"""
        timestamp = datetime.now().isoformat()
        self.detection_history.append({
            'timestamp': timestamp,
            'detections': detections,
            'count': len(detections)
        })
        
        # Limit history size
        if len(self.detection_history) > self.max_history:
            self.detection_history.pop(0)
    
    def get_detection_stats(self) -> Dict:
        """Get detection statistics"""
        if not self.detection_history:
            return {
                'total_detections': 0,
                'average_confidence': 0.0,
                'most_common_class': None,
                'detection_rate': 0.0
            }
        
        # Calculate stats
        all_detections = []
        for entry in self.detection_history:
            all_detections.extend(entry['detections'])
        
        if not all_detections:
            return {
                'total_detections': 0,
                'average_confidence': 0.0,
                'most_common_class': None,
                'detection_rate': 0.0
            }
        
        # Average confidence
        avg_confidence = np.mean([d['confidence'] for d in all_detections])
        
        # Most common class
        class_counts = {}
        for d in all_detections:
            class_name = d['class_name']
            class_counts[class_name] = class_counts.get(class_name, 0) + 1
        
        most_common = max(class_counts.items(), key=lambda x: x[1]) if class_counts else (None, 0)
        
        # Detection rate (detections per frame)
        detection_rate = len(all_detections) / len(self.detection_history)
        
        return {
            'total_detections': len(all_detections),
            'average_confidence': float(avg_confidence),
            'most_common_class': most_common[0],
            'detection_rate': float(detection_rate),
            'class_distribution': class_counts
        }
    
    def save_detection_history(self, filepath: str):
        """Save detection history to file"""
        try:
            with open(filepath, 'w') as f:
                json.dump(self.detection_history, f, indent=2)
            logger.info(f"Detection history saved to {filepath}")
        except Exception as e:
            logger.error(f"Failed to save history: {e}")
    
    def get_summary(self) -> Dict:
        """Get detector summary"""
        return {
            'model_loaded': self.model is not None,
            'device': str(self.device),
            'confidence_threshold': self.confidence_threshold,
            'total_classes': len(self.class_names),
            'history_size': len(self.detection_history),
            'stats': self.get_detection_stats()
        }

# Global detector instance
_detector = None

def get_detector() -> YOLODetector:
    """Get or create global detector instance"""
    global _detector
    if _detector is None:
        _detector = YOLODetector()
    return _detector

def detect_objects_from_image(image: np.ndarray) -> List[Dict]:
    """Convenience function for object detection"""
    detector = get_detector()
    return detector.detect_objects(image)

def draw_detections_on_image(image: np.ndarray, detections: List[Dict]) -> np.ndarray:
    """Convenience function for drawing detections"""
    detector = get_detector()
    return detector.draw_detections(image, detections)
