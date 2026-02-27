"""
DepthEstimation: Estimacion de profundidad monocular (V2/V3).

Estima mapas de profundidad a partir de imagenes RGB.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

# Intentar importar numpy
try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False
    np = None


@dataclass
class DepthOutput:
    """Salida del estimador de profundidad."""
    depth_map: Any  # np.ndarray (H, W) con valores en metros
    min_depth: float
    max_depth: float
    mean_depth: float
    width: int
    height: int
    processing_time_ms: float
    confidence: float = 1.0
    model_used: str = "unknown"
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    
    def get_depth_at(self, x: int, y: int) -> Optional[float]:
        """Obtiene profundidad en pixel (x, y)."""
        if self.depth_map is None:
            return None
        try:
            return float(self.depth_map[y, x])
        except (IndexError, TypeError):
            return None
    
    def get_point_cloud(self, fx: float = 500, fy: float = 500, 
                       cx: Optional[float] = None, cy: Optional[float] = None) -> Any:
        """
        Genera point cloud a partir del mapa de profundidad.
        
        Args:
            fx, fy: Focal lengths
            cx, cy: Centro optico (default: centro de imagen)
        
        Returns:
            Array (N, 3) con puntos 3D
        """
        if not HAS_NUMPY or self.depth_map is None:
            return None
        
        h, w = self.depth_map.shape[:2]
        cx = cx or w / 2
        cy = cy or h / 2
        
        # Crear meshgrid
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        
        # Convertir a coordenadas 3D
        z = self.depth_map.astype(np.float32)
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        
        # Stack y reshape
        points = np.stack([x, y, z], axis=-1)
        return points.reshape(-1, 3)


class DepthModel:
    """Interfaz base para modelos de profundidad."""
    
    def __init__(self, model_name: str = "mock"):
        self.model_name = model_name
        self._model = None
        self._loaded = False
    
    def load(self) -> bool:
        """Carga el modelo."""
        raise NotImplementedError
    
    def predict(self, image: Any) -> Any:
        """Predice mapa de profundidad."""
        raise NotImplementedError
    
    def is_loaded(self) -> bool:
        return self._loaded


class MockDepthModel(DepthModel):
    """Modelo de profundidad mock para pruebas."""
    
    def __init__(self):
        super().__init__("mock")
    
    def load(self) -> bool:
        self._loaded = True
        return True
    
    def predict(self, image: Any) -> Any:
        """Genera mapa de profundidad mock."""
        if not HAS_NUMPY:
            return None
        
        # Obtener dimensiones
        if hasattr(image, 'shape'):
            h, w = image.shape[:2]
        else:
            h, w = 480, 640
        
        # Generar gradiente de profundidad (simula perspectiva)
        y_coords = np.linspace(0.5, 3.0, h)
        depth = np.tile(y_coords.reshape(-1, 1), (1, w))
        
        # Agregar algo de ruido
        noise = np.random.normal(0, 0.1, (h, w))
        depth = depth + noise
        
        # Clip a rango valido
        depth = np.clip(depth, 0.1, 10.0)
        
        return depth.astype(np.float32)


class DepthAnythingModel(DepthModel):
    """Wrapper para Depth Anything v2."""
    
    def __init__(self, model_size: str = "small"):
        super().__init__(f"depth_anything_{model_size}")
        self.model_size = model_size
    
    def load(self) -> bool:
        """Carga modelo Depth Anything."""
        try:
            # Intentar cargar Depth Anything
            # En produccion: from depth_anything import DepthAnything
            logger.info(f"Loading Depth Anything {self.model_size}...")
            
            # Mock: simular carga
            self._loaded = True
            logger.info("Depth Anything model loaded (mock mode)")
            return True
            
        except Exception as e:
            logger.warning(f"Could not load Depth Anything: {e}")
            return False
    
    def predict(self, image: Any) -> Any:
        """Predice profundidad con Depth Anything."""
        if not self._loaded:
            return None
        
        # En produccion: return self._model.infer(image)
        # Mock: usar modelo mock
        mock = MockDepthModel()
        return mock.predict(image)


class MiDaSModel(DepthModel):
    """Wrapper para MiDaS depth estimation con GPU."""
    
    def __init__(self, model_type: str = "MiDaS_small"):
        super().__init__(f"midas_{model_type}")
        self.model_type = model_type
        self._transform = None
        self._device = None
    
    def load(self) -> bool:
        """Carga modelo MiDaS con CUDA si disponible."""
        try:
            import torch
            
            logger.info(f"Loading MiDaS {self.model_type}...")
            
            # Determinar device
            self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            logger.info(f"Using device: {self._device}")
            
            # Cargar modelo desde torch hub
            self._model = torch.hub.load("intel-isl/MiDaS", self.model_type, trust_repo=True)
            self._model.to(self._device)
            self._model.eval()
            
            # Cargar transformaciones
            midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)
            if "small" in self.model_type.lower():
                self._transform = midas_transforms.small_transform
            else:
                self._transform = midas_transforms.default_transform
            
            self._loaded = True
            logger.info(f"MiDaS model loaded on {self._device}")
            return True
            
        except Exception as e:
            logger.warning(f"Could not load MiDaS: {e}, falling back to mock")
            return False
    
    def predict(self, image: Any) -> Any:
        """Predice profundidad con MiDaS."""
        if not self._loaded or self._model is None:
            mock = MockDepthModel()
            return mock.predict(image)
        
        try:
            import torch
            import cv2
            
            # Asegurar formato correcto
            if hasattr(image, 'shape') and len(image.shape) == 3:
                if image.shape[2] == 4:  # RGBA -> RGB
                    image = image[:, :, :3]
                if image.dtype != np.uint8:
                    image = (image * 255).astype(np.uint8)
            
            # Aplicar transformacion
            input_batch = self._transform(image).to(self._device)
            
            # Inferencia
            with torch.no_grad():
                prediction = self._model(input_batch)
                prediction = torch.nn.functional.interpolate(
                    prediction.unsqueeze(1),
                    size=image.shape[:2],
                    mode="bicubic",
                    align_corners=False,
                ).squeeze()
            
            # Convertir a numpy y normalizar a metros
            depth = prediction.cpu().numpy()
            
            # MiDaS da profundidad inversa, convertir a metros
            depth = depth - depth.min()
            depth = depth / (depth.max() + 1e-8)
            depth = 0.1 + depth * 9.9  # Rango 0.1 a 10 metros
            
            return depth.astype(np.float32)
            
        except Exception as e:
            logger.error(f"MiDaS prediction error: {e}")
            mock = MockDepthModel()
            return mock.predict(image)


class DepthEstimation:
    """
    Sistema de estimacion de profundidad monocular.
    
    Soporta multiples backends:
    - Depth Anything v2
    - MiDaS
    - Mock (para pruebas)
    """
    
    def __init__(
        self,
        model_name: str = "depth_anything",
        model_size: str = "small",
        min_depth: float = 0.1,
        max_depth: float = 10.0,
        scale_factor: float = 1.0,
    ):
        """
        Inicializa el estimador de profundidad.
        
        Args:
            model_name: Nombre del modelo ("depth_anything", "midas", "mock")
            model_size: Tamano del modelo ("small", "base", "large")
            min_depth: Profundidad minima en metros
            max_depth: Profundidad maxima en metros
            scale_factor: Factor de escala para la profundidad
        """
        self.model_name = model_name
        self.model_size = model_size
        self.min_depth = min_depth
        self.max_depth = max_depth
        self.scale_factor = scale_factor
        
        # Seleccionar modelo
        if model_name == "depth_anything":
            self._model = DepthAnythingModel(model_size)
        elif model_name == "midas":
            self._model = MiDaSModel(model_size)
        else:
            self._model = MockDepthModel()
        
        # Estadisticas
        self._frame_count = 0
        self._total_time_ms = 0
        
        # Cache
        self._last_output: Optional[DepthOutput] = None
        
        # Callbacks
        self._callbacks: List[Callable[[DepthOutput], None]] = []
    
    def load(self) -> bool:
        """Carga el modelo."""
        return self._model.load()
    
    def estimate(self, image: Any) -> DepthOutput:
        """
        Estima mapa de profundidad de imagen RGB.
        
        Args:
            image: Imagen RGB (numpy array HxWx3)
        
        Returns:
            DepthOutput con mapa de profundidad
        """
        start_time = time.time()
        
        # Cargar modelo si no esta cargado
        if not self._model.is_loaded():
            self._model.load()
        
        # Obtener dimensiones
        if hasattr(image, 'shape'):
            h, w = image.shape[:2]
        else:
            h, w = 480, 640
        
        # Predecir
        raw_depth = self._model.predict(image)
        
        if raw_depth is None:
            # Fallback
            if HAS_NUMPY:
                raw_depth = np.ones((h, w), dtype=np.float32) * 2.0
            else:
                raw_depth = [[2.0] * w for _ in range(h)]
        
        # Aplicar escala y limites
        if HAS_NUMPY:
            depth_map = raw_depth * self.scale_factor
            depth_map = np.clip(depth_map, self.min_depth, self.max_depth)
            
            min_d = float(np.min(depth_map))
            max_d = float(np.max(depth_map))
            mean_d = float(np.mean(depth_map))
        else:
            depth_map = raw_depth
            min_d = self.min_depth
            max_d = self.max_depth
            mean_d = (min_d + max_d) / 2
        
        processing_time = (time.time() - start_time) * 1000
        
        output = DepthOutput(
            depth_map=depth_map,
            min_depth=min_d,
            max_depth=max_d,
            mean_depth=mean_d,
            width=w,
            height=h,
            processing_time_ms=processing_time,
            model_used=self._model.model_name,
        )
        
        # Actualizar estadisticas
        self._frame_count += 1
        self._total_time_ms += processing_time
        self._last_output = output
        
        # Callbacks
        for callback in self._callbacks:
            try:
                callback(output)
            except Exception as e:
                logger.error(f"Error in depth callback: {e}")
        
        return output
    
    def estimate_distance_to_object(
        self,
        image: Any,
        bbox: Tuple[int, int, int, int],
    ) -> float:
        """
        Estima distancia a objeto dado su bounding box.
        
        Args:
            image: Imagen RGB
            bbox: (x1, y1, x2, y2) del objeto
        
        Returns:
            Distancia estimada en metros
        """
        output = self.estimate(image)
        
        x1, y1, x2, y2 = bbox
        
        if output.depth_map is None:
            return self.max_depth / 2
        
        # Extraer region del mapa de profundidad
        if HAS_NUMPY:
            region = output.depth_map[y1:y2, x1:x2]
            if region.size > 0:
                # Usar mediana para robustez
                return float(np.median(region))
        
        return self.max_depth / 2
    
    def on_depth(self, callback: Callable[[DepthOutput], None]) -> None:
        """Registra callback para nuevos mapas de profundidad."""
        self._callbacks.append(callback)
    
    def get_last_output(self) -> Optional[DepthOutput]:
        """Obtiene ultimo output."""
        return self._last_output
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas."""
        return {
            "model_name": self._model.model_name,
            "model_loaded": self._model.is_loaded(),
            "frame_count": self._frame_count,
            "avg_processing_time_ms": self._total_time_ms / max(1, self._frame_count),
            "min_depth": self.min_depth,
            "max_depth": self.max_depth,
        }
