"""
ObjectRecognition: Reconocimiento de objetos (Corteza Inferotemporal).

Identifica y clasifica objetos especificos mediante embeddings y matching.
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
class ObjectIdentity:
    """Identidad de un objeto reconocido."""
    id: str                     # ID unico del objeto
    name: str                   # Nombre del objeto
    category: str               # Categoria (e.g., "cup", "bottle")
    instance_name: Optional[str] = None  # Nombre de instancia (e.g., "mi_taza_azul")
    
    # Confianza
    confidence: float = 0.0
    recognition_method: str = "unknown"  # "embedding", "template", "learned"
    
    # Ubicacion en imagen
    bbox: Optional[Tuple[int, int, int, int]] = None  # x1, y1, x2, y2
    
    # Embedding
    embedding: Optional[List[float]] = None
    
    # Atributos visuales
    color: Optional[str] = None
    size: Optional[str] = None  # "small", "medium", "large"
    material: Optional[str] = None
    
    # Metadata
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "category": self.category,
            "instance_name": self.instance_name,
            "confidence": self.confidence,
            "color": self.color,
            "bbox": self.bbox,
        }


@dataclass
class RecognitionOutput:
    """Salida del reconocedor de objetos."""
    objects: List[ObjectIdentity]
    processing_time_ms: float
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    
    def get_by_name(self, name: str) -> Optional[ObjectIdentity]:
        """Busca objeto por nombre."""
        for obj in self.objects:
            if obj.name.lower() == name.lower():
                return obj
        return None
    
    def get_by_category(self, category: str) -> List[ObjectIdentity]:
        """Filtra por categoria."""
        return [o for o in self.objects if o.category.lower() == category.lower()]


@dataclass
class KnownObject:
    """Objeto conocido en la base de datos."""
    id: str
    name: str
    category: str
    instance_name: Optional[str] = None
    embeddings: List[List[float]] = field(default_factory=list)  # Multiples vistas
    templates: List[Any] = field(default_factory=list)  # Templates de imagen
    attributes: Dict[str, Any] = field(default_factory=dict)
    times_seen: int = 0
    last_seen_ns: Optional[int] = None


class EmbeddingExtractor:
    """Extractor de embeddings visuales usando CLIP."""
    
    def __init__(self, model_name: str = "clip"):
        self.model_name = model_name
        self._model = None
        self._preprocess = None
        self._loaded = False
        self._device = None
        self.embedding_dim = 512
        self._use_real_clip = False
    
    def load(self) -> bool:
        """Carga el modelo CLIP con CUDA si disponible."""
        try:
            import torch
            
            logger.info(f"Loading CLIP model...")
            
            self._device = "cuda" if torch.cuda.is_available() else "cpu"
            
            # Usar transformers CLIP con safetensors (más seguro y estable)
            try:
                from transformers import CLIPProcessor, CLIPModel
                import os
                os.environ['HF_HUB_DISABLE_SYMLINKS_WARNING'] = '1'
                
                self._model = CLIPModel.from_pretrained(
                    "openai/clip-vit-base-patch32",
                    use_safetensors=True
                )
                self._preprocess = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")
                self._model.to(self._device)
                self._model.eval()
                self._use_real_clip = True
                self.embedding_dim = 512
                logger.info(f"CLIP (transformers) loaded on {self._device}")
            except Exception as e:
                logger.warning(f"CLIP not available: {e}, using mock embeddings")
                self._use_real_clip = False
            
            self._loaded = True
            return True
            
        except Exception as e:
            logger.warning(f"Could not load embedding model: {e}")
            self._loaded = True  # Mark as loaded but use mock
            return False
    
    def extract(self, image: Any, bbox: Optional[Tuple[int, int, int, int]] = None) -> List[float]:
        """
        Extrae embedding de imagen o region usando CLIP.
        
        Args:
            image: Imagen completa (numpy array HxWx3)
            bbox: Region de interes opcional
        
        Returns:
            Embedding como lista de floats
        """
        if not self._loaded:
            self.load()
        
        # Crop si hay bbox
        if bbox is not None and hasattr(image, '__getitem__'):
            x1, y1, x2, y2 = bbox
            try:
                image = image[y1:y2, x1:x2]
            except:
                pass
        
        # Usar CLIP real si disponible
        if self._use_real_clip and self._model is not None:
            try:
                import torch
                from PIL import Image
                
                # Convertir a PIL
                if hasattr(image, 'shape'):
                    if image.dtype != np.uint8:
                        image = (image * 255).astype(np.uint8)
                    pil_image = Image.fromarray(image)
                else:
                    pil_image = image
                
                # Extraer embedding
                with torch.no_grad():
                    if hasattr(self._preprocess, '__call__'):
                        # openai-clip style
                        image_input = self._preprocess(pil_image).unsqueeze(0).to(self._device)
                        embedding = self._model.encode_image(image_input)
                    else:
                        # transformers style
                        inputs = self._preprocess(images=pil_image, return_tensors="pt").to(self._device)
                        embedding = self._model.get_image_features(**inputs)
                    
                    # Normalizar
                    embedding = embedding / embedding.norm(dim=-1, keepdim=True)
                    return embedding.cpu().numpy().flatten().tolist()
                    
            except Exception as e:
                logger.warning(f"CLIP extraction failed: {e}, using mock")
        
        # Mock: generar embedding aleatorio normalizado
        if HAS_NUMPY:
            embedding = np.random.randn(self.embedding_dim)
            embedding = embedding / np.linalg.norm(embedding)
            return embedding.tolist()
        else:
            import random
            return [random.gauss(0, 1) for _ in range(self.embedding_dim)]
    
    def is_loaded(self) -> bool:
        return self._loaded


class ObjectRecognition:
    """
    Sistema de reconocimiento de objetos especificos.
    
    Funcionalidades:
    - Reconocer objetos conocidos
    - Aprender nuevos objetos
    - Extraer atributos visuales
    """
    
    def __init__(
        self,
        embedding_model: str = "clip",
        similarity_threshold: float = 0.7,
        storage_path: Optional[str] = None,
    ):
        """
        Inicializa el reconocedor.
        
        Args:
            embedding_model: Modelo para embeddings
            similarity_threshold: Umbral de similitud para matching
            storage_path: Ruta para persistencia
        """
        self.similarity_threshold = similarity_threshold
        self.storage_path = storage_path
        
        # Extractor de embeddings
        self._extractor = EmbeddingExtractor(embedding_model)
        
        # Base de datos de objetos conocidos
        self._known_objects: Dict[str, KnownObject] = {}
        
        # Cache de embeddings
        self._embedding_cache: Dict[str, List[float]] = {}
        
        # Estadisticas
        self._recognition_count = 0
        self._learning_count = 0
        self._total_time_ms = 0
        
        # Callbacks
        self._callbacks: List[Callable[[RecognitionOutput], None]] = []
        
        # Inicializar con objetos comunes
        self._init_common_objects()
    
    def _init_common_objects(self) -> None:
        """Inicializa objetos comunes."""
        common = [
            ("cup", "container"),
            ("bottle", "container"),
            ("phone", "electronics"),
            ("keyboard", "electronics"),
            ("mouse", "electronics"),
            ("book", "object"),
            ("pen", "tool"),
            ("scissors", "tool"),
            ("remote", "electronics"),
            ("apple", "food"),
            ("banana", "food"),
            ("chair", "furniture"),
            ("table", "furniture"),
        ]
        
        for name, category in common:
            obj_id = f"generic_{name}"
            self._known_objects[obj_id] = KnownObject(
                id=obj_id,
                name=name,
                category=category,
            )
    
    def recognize(
        self,
        image: Any,
        detections: List[Dict[str, Any]] = None,
    ) -> RecognitionOutput:
        """
        Reconoce objetos en imagen.
        
        Args:
            image: Imagen RGB
            detections: Lista de detecciones previas (de VisionPipeline)
        
        Returns:
            RecognitionOutput con objetos identificados
        """
        start_time = time.time()
        recognized = []
        
        # Cargar extractor si es necesario
        if not self._extractor.is_loaded():
            self._extractor.load()
        
        # Procesar cada deteccion
        if detections:
            for det in detections:
                identity = self._recognize_detection(image, det)
                if identity:
                    recognized.append(identity)
        else:
            # Sin detecciones, intentar reconocer imagen completa
            identity = self._recognize_full_image(image)
            if identity:
                recognized.append(identity)
        
        processing_time = (time.time() - start_time) * 1000
        
        output = RecognitionOutput(
            objects=recognized,
            processing_time_ms=processing_time,
        )
        
        # Actualizar estadisticas
        self._recognition_count += 1
        self._total_time_ms += processing_time
        
        # Callbacks
        for callback in self._callbacks:
            try:
                callback(output)
            except Exception as e:
                logger.error(f"Error in recognition callback: {e}")
        
        return output
    
    def _recognize_detection(
        self,
        image: Any,
        detection: Dict[str, Any],
    ) -> Optional[ObjectIdentity]:
        """Reconoce una deteccion especifica."""
        bbox = detection.get("bbox") or detection.get("box")
        class_name = detection.get("class") or detection.get("class_name", "unknown")
        confidence = detection.get("confidence", 0.5)
        
        # Extraer embedding
        embedding = self._extractor.extract(image, bbox)
        
        # Buscar match en objetos conocidos
        best_match = None
        best_similarity = 0.0
        
        for obj_id, known in self._known_objects.items():
            if known.embeddings:
                similarity = self._compute_similarity(embedding, known.embeddings)
                if similarity > best_similarity and similarity > self.similarity_threshold:
                    best_similarity = similarity
                    best_match = known
            elif known.name.lower() == class_name.lower():
                # Match por nombre de clase
                best_similarity = confidence
                best_match = known
        
        if best_match:
            # Actualizar objeto conocido
            best_match.times_seen += 1
            best_match.last_seen_ns = time.time_ns()
            
            return ObjectIdentity(
                id=best_match.id,
                name=best_match.name,
                category=best_match.category,
                instance_name=best_match.instance_name,
                confidence=best_similarity,
                recognition_method="embedding" if best_match.embeddings else "class_match",
                bbox=bbox if isinstance(bbox, tuple) else tuple(bbox) if bbox else None,
                embedding=embedding,
                color=best_match.attributes.get("color"),
            )
        
        # Objeto no reconocido pero detectado
        return ObjectIdentity(
            id=f"unknown_{time.time_ns()}",
            name=class_name,
            category="unknown",
            confidence=confidence * 0.5,
            recognition_method="detection_only",
            bbox=bbox if isinstance(bbox, tuple) else tuple(bbox) if bbox else None,
            embedding=embedding,
        )
    
    def _recognize_full_image(self, image: Any) -> Optional[ObjectIdentity]:
        """Reconoce objeto dominante en imagen."""
        embedding = self._extractor.extract(image)
        
        best_match = None
        best_similarity = 0.0
        
        for obj_id, known in self._known_objects.items():
            if known.embeddings:
                similarity = self._compute_similarity(embedding, known.embeddings)
                if similarity > best_similarity and similarity > self.similarity_threshold:
                    best_similarity = similarity
                    best_match = known
        
        if best_match:
            return ObjectIdentity(
                id=best_match.id,
                name=best_match.name,
                category=best_match.category,
                instance_name=best_match.instance_name,
                confidence=best_similarity,
                recognition_method="embedding",
                embedding=embedding,
            )
        
        return None
    
    def _compute_similarity(
        self,
        embedding: List[float],
        known_embeddings: List[List[float]],
    ) -> float:
        """Computa similitud maxima con embeddings conocidos."""
        if not known_embeddings:
            return 0.0
        
        if HAS_NUMPY:
            e = np.array(embedding)
            similarities = []
            for known in known_embeddings:
                k = np.array(known)
                sim = np.dot(e, k) / (np.linalg.norm(e) * np.linalg.norm(k) + 1e-8)
                similarities.append(sim)
            return float(max(similarities))
        else:
            # Fallback sin numpy
            return 0.5
    
    def learn_object(
        self,
        name: str,
        category: str,
        image: Any,
        bbox: Optional[Tuple[int, int, int, int]] = None,
        instance_name: Optional[str] = None,
        attributes: Dict[str, Any] = None,
    ) -> str:
        """
        Aprende nuevo objeto o agrega vista a objeto existente.
        
        Args:
            name: Nombre del objeto
            category: Categoria
            image: Imagen del objeto
            bbox: Region en la imagen
            instance_name: Nombre de instancia especifico
            attributes: Atributos adicionales
        
        Returns:
            ID del objeto
        """
        # Generar embedding
        embedding = self._extractor.extract(image, bbox)
        
        # Buscar si ya existe
        obj_id = None
        for oid, known in self._known_objects.items():
            if known.name == name and known.instance_name == instance_name:
                obj_id = oid
                break
        
        if obj_id:
            # Agregar embedding a objeto existente
            self._known_objects[obj_id].embeddings.append(embedding)
            self._known_objects[obj_id].times_seen += 1
            if attributes:
                self._known_objects[obj_id].attributes.update(attributes)
        else:
            # Crear nuevo objeto
            import uuid
            obj_id = f"learned_{uuid.uuid4().hex[:8]}"
            
            self._known_objects[obj_id] = KnownObject(
                id=obj_id,
                name=name,
                category=category,
                instance_name=instance_name,
                embeddings=[embedding],
                attributes=attributes or {},
                times_seen=1,
                last_seen_ns=time.time_ns(),
            )
        
        self._learning_count += 1
        logger.info(f"Learned object: {name} (id={obj_id})")
        
        return obj_id
    
    def forget_object(self, obj_id: str) -> bool:
        """Elimina objeto de la base de datos."""
        if obj_id in self._known_objects:
            del self._known_objects[obj_id]
            return True
        return False
    
    def find_object(self, query: str) -> Optional[KnownObject]:
        """Busca objeto por nombre o instancia."""
        query_lower = query.lower()
        
        for obj in self._known_objects.values():
            if obj.name.lower() == query_lower:
                return obj
            if obj.instance_name and obj.instance_name.lower() == query_lower:
                return obj
        
        return None
    
    def get_objects_by_category(self, category: str) -> List[KnownObject]:
        """Obtiene objetos por categoria."""
        return [
            obj for obj in self._known_objects.values()
            if obj.category.lower() == category.lower()
        ]
    
    def on_recognition(self, callback: Callable[[RecognitionOutput], None]) -> None:
        """Registra callback para reconocimientos."""
        self._callbacks.append(callback)
    
    def get_known_objects(self) -> List[Dict[str, Any]]:
        """Lista objetos conocidos."""
        return [
            {
                "id": obj.id,
                "name": obj.name,
                "category": obj.category,
                "instance_name": obj.instance_name,
                "num_embeddings": len(obj.embeddings),
                "times_seen": obj.times_seen,
            }
            for obj in self._known_objects.values()
        ]
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas."""
        return {
            "known_objects": len(self._known_objects),
            "recognition_count": self._recognition_count,
            "learning_count": self._learning_count,
            "avg_processing_time_ms": self._total_time_ms / max(1, self._recognition_count),
            "similarity_threshold": self.similarity_threshold,
        }
