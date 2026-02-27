"""
VisionPipeline: Pipeline de visión del lóbulo occipital.

Análogo biológico: Corteza visual (V1, V2, V4, IT)
- Detección de objetos (YOLO)
- Segmentación (SAM)
- Estimación de profundidad
- Detección de humanos y rostros
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)


def _bitacora(msg: str, ok: bool = True) -> None:
    try:
        from modules.humanoid.ans.evolution_bitacora import append_evolution_log
        append_evolution_log(msg, ok=ok, source="cortex.occipital")
    except Exception:
        pass


@dataclass
class BoundingBox:
    """Bounding box para detecciones."""
    x: float  # top-left x
    y: float  # top-left y
    width: float
    height: float
    
    def center(self) -> Tuple[float, float]:
        return (self.x + self.width / 2, self.y + self.height / 2)
    
    def area(self) -> float:
        return self.width * self.height
    
    def to_xyxy(self) -> Tuple[float, float, float, float]:
        return (self.x, self.y, self.x + self.width, self.y + self.height)
    
    def iou(self, other: 'BoundingBox') -> float:
        """Calcula Intersection over Union con otro bbox."""
        x1 = max(self.x, other.x)
        y1 = max(self.y, other.y)
        x2 = min(self.x + self.width, other.x + other.width)
        y2 = min(self.y + self.height, other.y + other.height)
        
        intersection = max(0, x2 - x1) * max(0, y2 - y1)
        union = self.area() + other.area() - intersection
        
        return intersection / union if union > 0 else 0


@dataclass
class Detection:
    """Detección de objeto."""
    class_name: str
    confidence: float
    bbox: BoundingBox
    object_id: Optional[str] = None
    depth: Optional[float] = None  # metros
    pose_3d: Optional[Tuple[float, float, float]] = None  # x, y, z
    mask: Optional[Any] = None  # Máscara de segmentación
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "class_name": self.class_name,
            "confidence": self.confidence,
            "bbox": {
                "x": self.bbox.x,
                "y": self.bbox.y,
                "width": self.bbox.width,
                "height": self.bbox.height,
            },
            "object_id": self.object_id,
            "depth": self.depth,
            "pose_3d": self.pose_3d,
        }


@dataclass
class HumanPose:
    """Pose de humano detectado."""
    keypoints: Dict[str, Tuple[float, float, float]]  # {nombre: (x, y, confidence)}
    bbox: BoundingBox
    confidence: float
    person_id: Optional[str] = None
    
    KEYPOINT_NAMES = [
        "nose", "left_eye", "right_eye", "left_ear", "right_ear",
        "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
        "left_wrist", "right_wrist", "left_hip", "right_hip",
        "left_knee", "right_knee", "left_ankle", "right_ankle",
    ]
    
    def get_face_center(self) -> Optional[Tuple[float, float]]:
        """Obtiene centro de la cara."""
        if "nose" in self.keypoints:
            nose = self.keypoints["nose"]
            return (nose[0], nose[1])
        return None
    
    def is_facing_camera(self) -> bool:
        """Verifica si la persona está mirando hacia la cámara."""
        if "nose" not in self.keypoints:
            return False
        if "left_eye" not in self.keypoints or "right_eye" not in self.keypoints:
            return False
        
        left_eye = self.keypoints["left_eye"]
        right_eye = self.keypoints["right_eye"]
        
        # Ambos ojos deben ser visibles
        return left_eye[2] > 0.5 and right_eye[2] > 0.5


@dataclass
class VisionOutput:
    """Salida del pipeline de visión."""
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    detections: List[Detection] = field(default_factory=list)
    humans: List[HumanPose] = field(default_factory=list)
    depth_map: Optional[Any] = None  # numpy array
    processing_time_ms: float = 0.0
    
    def get_detection_by_class(self, class_name: str) -> List[Detection]:
        """Obtiene detecciones de una clase."""
        return [d for d in self.detections if d.class_name.lower() == class_name.lower()]
    
    def get_nearest_detection(self, class_name: str = None) -> Optional[Detection]:
        """Obtiene la detección más cercana."""
        candidates = self.detections
        if class_name:
            candidates = self.get_detection_by_class(class_name)
        
        if not candidates:
            return None
        
        # Ordenar por profundidad (o tamaño de bbox como proxy)
        def depth_key(d):
            if d.depth is not None:
                return d.depth
            return -d.bbox.area()  # Mayor área = más cerca
        
        return min(candidates, key=depth_key)
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp_ns": self.timestamp_ns,
            "detections": [d.to_dict() for d in self.detections],
            "humans_count": len(self.humans),
            "processing_time_ms": self.processing_time_ms,
        }


class MockDetector:
    """Detector mock para testing sin modelos."""
    
    def detect(self, frame: Any) -> List[Detection]:
        """Simula detecciones."""
        return [
            Detection(
                class_name="cup",
                confidence=0.85,
                bbox=BoundingBox(100, 100, 50, 60),
                depth=0.8,
            )
        ]


class MockPoseEstimator:
    """Estimador de pose mock para testing."""
    
    def detect(self, frame: Any) -> List[HumanPose]:
        """Simula detección de humanos."""
        return []


class MockDepthEstimator:
    """Estimador de profundidad mock."""
    
    def estimate(self, frame: Any) -> Any:
        """Simula mapa de profundidad."""
        return None


class VisionPipeline:
    """
    Pipeline de visión del lóbulo occipital.
    
    Integra múltiples modelos de visión:
    - Detección de objetos (YOLO)
    - Segmentación (SAM)
    - Estimación de profundidad (Depth Anything)
    - Detección de humanos (MediaPipe/YOLO-Pose)
    """
    
    def __init__(self, 
                 detector_model: str = "yolov8n",
                 enable_depth: bool = True,
                 enable_pose: bool = True,
                 enable_segmentation: bool = False,
                 device: str = "auto"):
        """
        Inicializa el pipeline de visión.
        
        Args:
            detector_model: Modelo de detección (yolov8n, yolov8s, etc.)
            enable_depth: Habilitar estimación de profundidad
            enable_pose: Habilitar detección de pose humana
            enable_segmentation: Habilitar segmentación (más lento)
            device: Dispositivo (cpu, cuda, auto)
        """
        self.detector_model = detector_model
        self.enable_depth = enable_depth
        self.enable_pose = enable_pose
        self.enable_segmentation = enable_segmentation
        
        # Auto-detectar GPU
        if device == "auto":
            try:
                import torch
                self.device = "cuda" if torch.cuda.is_available() else "cpu"
                logger.info(f"VisionPipeline using device: {self.device}")
            except ImportError:
                self.device = "cpu"
        else:
            self.device = device
        
        # Modelos (lazy loading)
        self._detector = None
        self._pose_estimator = None
        self._depth_estimator = None
        self._segmenter = None
        
        # Cache de últimas detecciones
        self._last_output: Optional[VisionOutput] = None
        
        # Tracking de objetos
        self._object_tracks: Dict[str, Detection] = {}
        self._next_track_id = 0
    
    def _load_detector(self) -> None:
        """Carga modelo de detección con CUDA si disponible."""
        if self._detector is not None:
            return
        
        try:
            from ultralytics import YOLO
            self._detector = YOLO(f"{self.detector_model}.pt")
            # Mover a GPU si disponible
            if self.device == "cuda":
                self._detector.to("cuda")
            logger.info(f"Loaded YOLO detector: {self.detector_model} on {self.device}")
        except ImportError:
            logger.warning("YOLO not available, using mock detector")
            self._detector = MockDetector()
        except Exception as e:
            logger.error(f"Error loading detector: {e}")
            self._detector = MockDetector()
    
    def _load_pose_estimator(self) -> None:
        """Carga estimador de pose."""
        if self._pose_estimator is not None:
            return
        
        try:
            import mediapipe as mp
            self._pose_estimator = mp.solutions.pose.Pose(
                static_image_mode=False,
                model_complexity=1,
                min_detection_confidence=0.5,
            )
            logger.info("Loaded MediaPipe pose estimator")
        except ImportError:
            logger.warning("MediaPipe not available, using mock pose estimator")
            self._pose_estimator = MockPoseEstimator()
    
    def _load_depth_estimator(self) -> None:
        """Carga estimador de profundidad (MiDaS con CUDA)."""
        if self._depth_estimator is not None:
            return
        
        try:
            from .depth_estimation import DepthEstimation
            self._depth_estimator = DepthEstimation(model_name="midas", model_size="MiDaS_small")
            self._depth_estimator.load()
            logger.info("Loaded MiDaS depth estimator")
        except Exception as e:
            logger.warning(f"Could not load depth estimator: {e}, using mock")
            self._depth_estimator = MockDepthEstimator()
    
    def process_frame(self, frame: Any, 
                     detect_objects: bool = True,
                     detect_humans: bool = True,
                     estimate_depth: bool = True) -> VisionOutput:
        """
        Procesa un frame de imagen.
        
        Args:
            frame: Frame de imagen (numpy array BGR o RGB)
            detect_objects: Ejecutar detección de objetos
            detect_humans: Ejecutar detección de humanos
            estimate_depth: Ejecutar estimación de profundidad
        
        Returns:
            VisionOutput con resultados
        """
        start_time = time.time()
        
        detections = []
        humans = []
        depth_map = None
        
        # Detección de objetos
        if detect_objects:
            self._load_detector()
            detections = self._detect_objects(frame)
        
        # Detección de humanos
        if detect_humans and self.enable_pose:
            self._load_pose_estimator()
            humans = self._detect_humans(frame)
        
        # Estimación de profundidad
        if estimate_depth and self.enable_depth:
            self._load_depth_estimator()
            depth_map = self._estimate_depth(frame)
            
            # Agregar profundidad a detecciones
            if depth_map is not None:
                self._add_depth_to_detections(detections, depth_map)
        
        # Tracking simple
        self._update_tracks(detections)
        
        # Construir output
        processing_time = (time.time() - start_time) * 1000
        
        output = VisionOutput(
            detections=detections,
            humans=humans,
            depth_map=depth_map,
            processing_time_ms=processing_time,
        )
        
        self._last_output = output
        _bitacora(f"Vision: {len(output.detections)} detections, {len(output.humans)} humans")
        return output
    
    def _detect_objects(self, frame: Any) -> List[Detection]:
        """Ejecuta detección de objetos."""
        if isinstance(self._detector, MockDetector):
            return self._detector.detect(frame)
        
        try:
            results = self._detector(frame, verbose=False)
            detections = []
            
            for result in results:
                boxes = result.boxes
                if boxes is None:
                    continue
                
                for i in range(len(boxes)):
                    xyxy = boxes.xyxy[i].cpu().numpy()
                    conf = float(boxes.conf[i])
                    cls_id = int(boxes.cls[i])
                    cls_name = result.names[cls_id]
                    
                    bbox = BoundingBox(
                        x=float(xyxy[0]),
                        y=float(xyxy[1]),
                        width=float(xyxy[2] - xyxy[0]),
                        height=float(xyxy[3] - xyxy[1]),
                    )
                    
                    detections.append(Detection(
                        class_name=cls_name,
                        confidence=conf,
                        bbox=bbox,
                    ))
            
            return detections
            
        except Exception as e:
            logger.error(f"Detection error: {e}")
            return []
    
    def _detect_humans(self, frame: Any) -> List[HumanPose]:
        """Ejecuta detección de humanos y pose."""
        if isinstance(self._pose_estimator, MockPoseEstimator):
            return self._pose_estimator.detect(frame)
        
        try:
            import cv2
            
            # MediaPipe espera RGB
            if len(frame.shape) == 3 and frame.shape[2] == 3:
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            else:
                rgb_frame = frame
            
            results = self._pose_estimator.process(rgb_frame)
            
            if not results.pose_landmarks:
                return []
            
            # Convertir landmarks a keypoints
            h, w = frame.shape[:2]
            keypoints = {}
            
            for idx, landmark in enumerate(results.pose_landmarks.landmark):
                if idx < len(HumanPose.KEYPOINT_NAMES):
                    name = HumanPose.KEYPOINT_NAMES[idx]
                    keypoints[name] = (
                        landmark.x * w,
                        landmark.y * h,
                        landmark.visibility,
                    )
            
            # Calcular bbox desde keypoints
            xs = [kp[0] for kp in keypoints.values()]
            ys = [kp[1] for kp in keypoints.values()]
            
            bbox = BoundingBox(
                x=min(xs),
                y=min(ys),
                width=max(xs) - min(xs),
                height=max(ys) - min(ys),
            )
            
            return [HumanPose(
                keypoints=keypoints,
                bbox=bbox,
                confidence=0.9,
            )]
            
        except Exception as e:
            logger.error(f"Pose detection error: {e}")
            return []
    
    def _estimate_depth(self, frame: Any) -> Any:
        """Ejecuta estimación de profundidad."""
        if isinstance(self._depth_estimator, MockDepthEstimator):
            return self._depth_estimator.estimate(frame)
        
        # Implementación real requiere Depth Anything o similar
        return None
    
    def _add_depth_to_detections(self, detections: List[Detection], 
                                 depth_map: Any) -> None:
        """Agrega información de profundidad a las detecciones."""
        if depth_map is None:
            return
        
        try:
            for detection in detections:
                # Obtener profundidad en el centro del bbox
                cx, cy = detection.bbox.center()
                cx, cy = int(cx), int(cy)
                
                if 0 <= cy < depth_map.shape[0] and 0 <= cx < depth_map.shape[1]:
                    detection.depth = float(depth_map[cy, cx])
                    
                    # Calcular pose 3D aproximada (simplificado)
                    # Requeriría calibración de cámara para ser preciso
                    detection.pose_3d = (cx * detection.depth / 500, 
                                        cy * detection.depth / 500, 
                                        detection.depth)
        except Exception as e:
            logger.error(f"Error adding depth: {e}")
    
    def _update_tracks(self, detections: List[Detection]) -> None:
        """Actualiza tracking de objetos."""
        # Matching simple por IoU
        unmatched = list(detections)
        new_tracks = {}
        
        for track_id, track in self._object_tracks.items():
            best_match = None
            best_iou = 0.3  # Umbral mínimo
            
            for det in unmatched:
                if det.class_name != track.class_name:
                    continue
                
                iou = det.bbox.iou(track.bbox)
                if iou > best_iou:
                    best_iou = iou
                    best_match = det
            
            if best_match:
                best_match.object_id = track_id
                new_tracks[track_id] = best_match
                unmatched.remove(best_match)
        
        # Crear nuevos tracks para detecciones sin match
        for det in unmatched:
            track_id = f"obj_{self._next_track_id}"
            self._next_track_id += 1
            det.object_id = track_id
            new_tracks[track_id] = det
        
        self._object_tracks = new_tracks
    
    def get_last_output(self) -> Optional[VisionOutput]:
        """Obtiene última salida del pipeline."""
        return self._last_output
    
    def find_object(self, class_name: str) -> Optional[Detection]:
        """Busca un objeto específico en las últimas detecciones."""
        if not self._last_output:
            return None
        
        detections = self._last_output.get_detection_by_class(class_name)
        if detections:
            # Retornar el de mayor confianza
            return max(detections, key=lambda d: d.confidence)
        return None
    
    def reset_tracks(self) -> None:
        """Resetea tracking de objetos."""
        self._object_tracks.clear()
        self._next_track_id = 0
