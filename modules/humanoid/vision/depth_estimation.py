"""
Fase 2 - Monocular depth estimation (MiDaS-style).
Salida: depth map relativo; opcional calibración métrica.
"""
from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

_log = logging.getLogger(__name__)

_model = None


def _get_model():
    """Lazy load MiDaS via torch hub. Fallback: None si torch no disponible."""
    global _model
    if _model is not None:
        return _model
    try:
        import torch
        # MiDaS small para CPU; DPT-Hybrid más preciso pero más pesado
        model = torch.hub.load("intel-isl/MiDaS", "MiDaS_small", trust_repo=True)
        model.eval()
        _model = model
        return _model
    except Exception as e:
        _log.warning("Depth model (MiDaS) no disponible: %s", e)
        return None


def estimate_depth(frame: np.ndarray) -> np.ndarray:
    """
    Estima mapa de profundidad relativo (0 = cerca, 1 = lejos normalizado).
    frame: BGR o RGB, cualquier tamaño; se redimensiona internamente.
    """
    model = _get_model()
    if model is None:
        h, w = frame.shape[:2]
        return np.zeros((h, w), dtype=np.float32)

    try:
        import torch
        import cv2
        from torchvision import transforms

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) if len(frame.shape) == 3 else frame
        h_orig, w_orig = rgb.shape[:2]
        size = 384
        rgb_small = cv2.resize(rgb, (size, size))
        tensor = transforms.ToTensor()(rgb_small).unsqueeze(0)
        normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        tensor = normalize(tensor)

        with torch.no_grad():
            pred = model(tensor)
            pred = torch.nn.functional.interpolate(
                pred.unsqueeze(1), size=(h_orig, w_orig), mode="bicubic", align_corners=False
            )
            depth = pred.squeeze().cpu().numpy()

        # Normalizar a [0, 1]: cerca = bajo, lejos = alto (invertir si MiDaS da alto=cerca)
        d_min, d_max = depth.min(), depth.max()
        if d_max > d_min:
            depth = (depth - d_min) / (d_max - d_min)
        return depth.astype(np.float32)
    except Exception as e:
        _log.exception("estimate_depth: %s", e)
        h, w = frame.shape[:2]
        return np.zeros((h, w), dtype=np.float32)


def get_distance_at_point(depth_map: np.ndarray, x: int, y: int) -> float:
    """Valor de profundidad en (x,y). 0 = cerca, 1 = lejos (relativo)."""
    h, w = depth_map.shape[:2]
    x = max(0, min(x, w - 1))
    y = max(0, min(y, h - 1))
    return float(depth_map[y, x])


def get_distance_in_bbox(
    depth_map: np.ndarray, bbox: List[int]
) -> Dict[str, float]:
    """bbox = [x1, y1, x2, y2]. Devuelve min, max, mean, median en la región."""
    x1, y1, x2, y2 = bbox[:4]
    h, w = depth_map.shape[:2]
    x1, x2 = max(0, x1), min(w, x2)
    y1, y2 = max(0, y1), min(h, y2)
    region = depth_map[y1:y2, x1:x2]
    if region.size == 0:
        return {"min": 0.0, "max": 0.0, "mean": 0.0, "median": 0.0}
    return {
        "min": float(np.min(region)),
        "max": float(np.max(region)),
        "mean": float(np.mean(region)),
        "median": float(np.median(region)),
    }


def depth_map_to_colored(depth_map: np.ndarray) -> np.ndarray:
    """Convierte depth float [0,1] a imagen BGR para visualización (cerca=rojo, lejos=azul)."""
    import cv2
    d = np.clip(depth_map * 255, 0, 255).astype(np.uint8)
    colored = cv2.applyColorMap(d, cv2.COLORMAP_INFERNO)
    return colored
