"""
Face Recognition System for ATLAS NEXUS Personal Assistant
Handles face detection, encoding, and owner identification using OpenCV and DeepFace
"""

import cv2
import numpy as np
import pickle
import os
import logging
from datetime import datetime
from typing import List, Tuple, Optional, Dict
from deepface import DeepFace
import base64
import io
from PIL import Image

logger = logging.getLogger('ATLAS_FACE_RECOGNITION')

class FaceRecognitionSystem:
    """Advanced face recognition system with emotion detection using OpenCV and DeepFace"""
    
    def __init__(self, data_dir: str = "data"):
        self.data_dir = data_dir
        self.face_encodings_file = os.path.join(data_dir, "face_encodings.pkl")
        self.owner_encodings = []
        self.confidence_threshold = 0.6
        self.emotion_threshold = 0.7
        
        # Create data directory if not exists
        os.makedirs(data_dir, exist_ok=True)
        
        # Load existing encodings
        self.load_encodings()
        
        logger.info("✅ Face Recognition System initialized (using OpenCV + DeepFace)")
    
    def load_encodings(self):
        """Load face encodings from file"""
        try:
            if os.path.exists(self.face_encodings_file):
                with open(self.face_encodings_file, 'rb') as f:
                    data = pickle.load(f)
                    self.owner_encodings = data.get('encodings', [])
                    logger.info(f"✅ Loaded {len(self.owner_encodings)} face encodings")
            else:
                logger.info("ℹ️ No existing face encodings found")
        except Exception as e:
            logger.error(f"❌ Error loading face encodings: {e}")
            self.owner_encodings = []
    
    def save_encodings(self):
        """Save face encodings to file"""
        try:
            data = {
                'encodings': self.owner_encodings,
                'updated_at': datetime.now().isoformat()
            }
            with open(self.face_encodings_file, 'wb') as f:
                pickle.dump(data, f)
            logger.info(f"✅ Saved {len(self.owner_encodings)} face encodings")
        except Exception as e:
            logger.error(f"❌ Error saving face encodings: {e}")
    
    def detect_faces(self, image: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """Detect faces in image using OpenCV Haar cascade"""
        try:
            # Convert to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Load face cascade
            face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
            
            # Detect faces
            faces = face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30, 30)
            )
            
            return faces.tolist()
        except Exception as e:
            logger.error(f"❌ Error detecting faces: {e}")
            return []
    
    def extract_face_embedding(self, image: np.ndarray, face_location: Tuple[int, int, int, int]) -> Optional[np.ndarray]:
        """Extract face embedding using DeepFace"""
        try:
            x, y, w, h = face_location
            
            # Extract face ROI with some padding
            padding = 20
            x1 = max(0, x - padding)
            y1 = max(0, y - padding)
            x2 = min(image.shape[1], x + w + padding)
            y2 = min(image.shape[0], y + h + padding)
            
            face_roi = image[y1:y2, x1:x2]
            
            # Convert to PIL Image
            face_pil = Image.fromarray(face_roi)
            
            # Extract embedding using DeepFace
            try:
                embedding = DeepFace.represent(
                    face_pil,
                    model_name='VGG-Face',
                    enforce_detection=False,
                    detector_backend='opencv'
                )
                
                if embedding and len(embedding) > 0:
                    return np.array(embedding[0]['embedding'])
                    
            except Exception as e:
                logger.warning(f"DeepFace embedding extraction failed: {e}")
                # Fallback: use face ROI as simple embedding
                face_gray = cv2.cvtColor(face_roi, cv2.COLOR_BGR2GRAY)
                face_resized = cv2.resize(face_gray, (64, 64))
                return face_resized.flatten().astype(np.float32)
            
            return None
        except Exception as e:
            logger.error(f"❌ Error extracting face embedding: {e}")
            return None
    
    def register_face_from_image(self, image_data: str) -> Dict:
        """Register face from base64 image data"""
        try:
            # Decode base64 image
            image_bytes = base64.b64decode(image_data.split(',')[1])
            image = Image.open(io.BytesIO(image_bytes))
            image_np = np.array(image)
            
            # Convert BGR for OpenCV processing
            if len(image_np.shape) == 3 and image_np.shape[2] == 3:
                image_bgr = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)
            else:
                image_bgr = image_np
            
            # Detect faces
            faces = self.detect_faces(image_bgr)
            
            if not faces:
                return {
                    "success": False,
                    "message": "No face detected in image",
                    "faces_detected": 0
                }
            
            if len(faces) > 1:
                return {
                    "success": False,
                    "message": "Multiple faces detected. Please show only one face.",
                    "faces_detected": len(faces)
                }
            
            # Extract embedding from the detected face
            face_location = faces[0]
            embedding = self.extract_face_embedding(image_bgr, face_location)
            
            if embedding is None:
                return {
                    "success": False,
                    "message": "Failed to extract face embedding",
                    "faces_detected": 1
                }
            
            # Add to encodings
            self.owner_encodings.append(embedding)
            
            # Save encodings
            self.save_encodings()
            
            return {
                "success": True,
                "message": "Face registered successfully",
                "total_encodings": len(self.owner_encodings),
                "face_location": face_location
            }
            
        except Exception as e:
            logger.error(f"❌ Error registering face: {e}")
            return {
                "success": False,
                "message": f"Error registering face: {str(e)}"
            }
    
    def is_owner_present(self, image_data: str) -> Dict:
        """Check if owner is present in image and detect emotion"""
        try:
            # Decode base64 image
            image_bytes = base64.b64decode(image_data.split(',')[1])
            image = Image.open(io.BytesIO(image_bytes))
            image_np = np.array(image)
            
            # Convert BGR for OpenCV processing
            if len(image_np.shape) == 3 and image_np.shape[2] == 3:
                image_bgr = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)
            else:
                image_bgr = image_np
            
            # Detect faces
            faces = self.detect_faces(image_bgr)
            
            if not faces:
                return {
                    "is_owner": False,
                    "confidence": 0.0,
                    "emotion": "none",
                    "message": "No face detected"
                }
            
            # Check each detected face
            for face_location in faces:
                # Extract embedding
                embedding = self.extract_face_embedding(image_bgr, face_location)
                
                if embedding is not None and self.owner_encodings:
                    # Compare with known embeddings using cosine similarity
                    similarities = []
                    for registered_embedding in self.owner_encodings:
                        # Normalize embeddings
                        embedding_norm = embedding / np.linalg.norm(embedding)
                        registered_norm = registered_embedding / np.linalg.norm(registered_embedding)
                        
                        # Calculate cosine similarity
                        similarity = np.dot(embedding_norm, registered_norm)
                        similarities.append(similarity)
                    
                    if similarities:
                        max_similarity = max(similarities)
                        confidence = float(max_similarity)
                        
                        if confidence >= self.confidence_threshold:
                            # Detect emotion
                            emotion = self.detect_emotion(image_np, face_location)
                            
                            return {
                                "is_owner": True,
                                "confidence": confidence,
                                "emotion": emotion,
                                "face_location": face_location.tolist(),
                                "message": f"Owner detected with {confidence:.2f} confidence"
                            }
            
            return {
                "is_owner": False,
                "confidence": 0.0,
                "emotion": "none",
                "message": "Face detected but not recognized as owner"
            }
            
        except Exception as e:
            logger.error(f"❌ Error checking owner presence: {e}")
            return {
                "is_owner": False,
                "confidence": 0.0,
                "emotion": "none",
                "message": f"Error: {str(e)}"
            }
    
    def detect_emotion(self, image: np.ndarray, face_location: Tuple[int, int, int, int]) -> str:
        """Detect emotion from face using DeepFace"""
        try:
            x, y, w, h = face_location
            
            # Extract face ROI with padding
            padding = 20
            x1 = max(0, x - padding)
            y1 = max(0, y - padding)
            x2 = min(image.shape[1], x + w + padding)
            y2 = min(image.shape[0], y + h + padding)
            
            face_roi = image[y1:y2, x1:x2]
            
            # Convert to PIL Image
            face_pil = Image.fromarray(face_roi)
            
            # Analyze emotion with DeepFace
            analysis = DeepFace.analyze(
                face_pil,
                actions=['emotion'],
                enforce_detection=False
            )
            
            if isinstance(analysis, list):
                analysis = analysis[0]
            
            emotions = analysis.get('emotion', {})
            
            if emotions:
                # Get dominant emotion
                dominant_emotion = max(emotions, key=emotions.get)
                confidence = emotions[dominant_emotion]
                
                if confidence >= self.emotion_threshold:
                    return dominant_emotion
            
            return "neutral"
            
        except Exception as e:
            logger.warning(f"⚠️ Error detecting emotion: {e}")
            return "neutral"
    
    def get_registration_status(self) -> Dict:
        """Get current registration status"""
        return {
            "face_registered": len(self.owner_encodings) > 0,
            "total_encodings": len(self.owner_encodings),
            "confidence_threshold": self.confidence_threshold,
            "minimum_encodings": 3,
            "ready_for_identification": len(self.owner_encodings) >= 3
        }
    
    def clear_encodings(self):
        """Clear all face encodings"""
        self.owner_encodings = []
        self.save_encodings()
        logger.info("✅ All face encodings cleared")

# Global instance
face_system = FaceRecognitionSystem()

def get_face_system() -> FaceRecognitionSystem:
    """Get global face recognition system instance"""
    return face_system
