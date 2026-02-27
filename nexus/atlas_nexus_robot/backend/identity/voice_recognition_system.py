"""
Voice Recognition System for ATLAS NEXUS Personal Assistant
Handles voice registration, speaker identification, and voice analysis using librosa
"""

import librosa
import numpy as np
import pickle
import os
import logging
from datetime import datetime
from typing import List, Optional, Dict
import soundfile as sf
import base64
import io
import tempfile

logger = logging.getLogger('ATLAS_VOICE_RECOGNITION')

class VoiceRecognitionSystem:
    """Advanced voice recognition system with speaker identification using librosa"""
    
    def __init__(self, data_dir: str = "data"):
        self.data_dir = data_dir
        self.voice_embeddings_file = os.path.join(data_dir, "voice_embeddings.pkl")
        self.owner_embeddings = []
        self.confidence_threshold = 0.75
        self.sample_rate = 16000
        
        # Create data directory if not exists
        os.makedirs(data_dir, exist_ok=True)
        
        # Load existing embeddings
        self.load_embeddings()
        
        logger.info("✅ Voice Recognition System initialized (using librosa)")
    
    def load_embeddings(self):
        """Load voice embeddings from file"""
        try:
            if os.path.exists(self.voice_embeddings_file):
                with open(self.voice_embeddings_file, 'rb') as f:
                    data = pickle.load(f)
                    self.owner_embeddings = data.get('embeddings', [])
                    logger.info(f"✅ Loaded {len(self.owner_embeddings)} voice embeddings")
            else:
                logger.info("ℹ️ No existing voice embeddings found")
        except Exception as e:
            logger.error(f"❌ Error loading voice embeddings: {e}")
            self.owner_embeddings = []
    
    def save_embeddings(self):
        """Save voice embeddings to file"""
        try:
            data = {
                'embeddings': self.owner_embeddings,
                'updated_at': datetime.now().isoformat()
            }
            with open(self.voice_embeddings_file, 'wb') as f:
                pickle.dump(data, f)
            logger.info(f"✅ Saved {len(self.owner_embeddings)} voice embeddings")
        except Exception as e:
            logger.error(f"❌ Error saving voice embeddings: {e}")
    
    def decode_base64_audio(self, audio_data: str) -> np.ndarray:
        """Decode base64 audio data to numpy array"""
        try:
            # Remove data URL prefix if present
            if ',' in audio_data:
                audio_data = audio_data.split(',')[1]
            
            # Decode base64
            audio_bytes = base64.b64decode(audio_data)
            
            # Create temporary file
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp_file:
                tmp_file.write(audio_bytes)
                tmp_file.flush()
                
                # Load audio with librosa
                audio, sr = librosa.load(tmp_file.name, sr=self.sample_rate)
                
                # Clean up temporary file
                os.unlink(tmp_file.name)
                
                return audio
        except Exception as e:
            logger.error(f"❌ Error decoding audio: {e}")
            return None
    
    def preprocess_audio(self, audio: np.ndarray) -> np.ndarray:
        """Preprocess audio for voice recognition"""
        try:
            # Ensure correct sample rate
            if len(audio) == 0:
                return None
            
            # Normalize audio
            audio = librosa.util.normalize(audio)
            
            # Remove silence
            audio, _ = librosa.effects.trim(audio, top_db=20)
            
            # Ensure minimum length (at least 1 second)
            min_length = self.sample_rate
            if len(audio) < min_length:
                # Pad with zeros
                audio = np.pad(audio, (0, min_length - len(audio)), mode='constant')
            
            return audio
        except Exception as e:
            logger.error(f"❌ Error preprocessing audio: {e}")
            return None
    
    def extract_voice_embedding(self, audio: np.ndarray) -> Optional[np.ndarray]:
        """Extract voice embedding using MFCC features"""
        try:
            # Preprocess audio
            audio = self.preprocess_audio(audio)
            if audio is None:
                return None
            
            # Extract MFCC features
            mfccs = librosa.feature.mfcc(
                y=audio, 
                sr=self.sample_rate, 
                n_mfcc=13,
                n_fft=2048,
                hop_length=512
            )
            
            # Extract additional features
            # Chroma features
            chroma = librosa.feature.chroma(y=audio, sr=self.sample_rate)
            
            # Spectral contrast
            spectral_contrast = librosa.feature.spectral_contrast(y=audio, sr=self.sample_rate)
            
            # Tonnetz (harmonic)
            tonnetz = librosa.feature.tonnetz(y=audio, sr=self.sample_rate)
            
            # Combine features
            features = np.concatenate([
                np.mean(mfccs, axis=1),
                np.std(mfccs, axis=1),
                np.mean(chroma, axis=1),
                np.mean(spectral_contrast, axis=1),
                np.mean(tonnetz, axis=1)
            ])
            
            return features
            
        except Exception as e:
            logger.error(f"❌ Error extracting voice embedding: {e}")
            return None
    
    def register_voice_from_audio(self, audio_data: str) -> Dict:
        """Register voice from base64 audio data"""
        try:
            # Decode audio
            audio = self.decode_base64_audio(audio_data)
            if audio is None:
                return {
                    "success": False,
                    "message": "Failed to decode audio data"
                }
            
            # Extract embedding
            embedding = self.extract_voice_embedding(audio)
            if embedding is None:
                return {
                    "success": False,
                    "message": "Failed to extract voice embedding"
                }
            
            # Add to embeddings
            self.owner_embeddings.append(embedding)
            
            # Save embeddings
            self.save_embeddings()
            
            return {
                "success": True,
                "message": "Voice registered successfully",
                "total_embeddings": len(self.owner_embeddings),
                "audio_length": len(audio) / self.sample_rate
            }
            
        except Exception as e:
            logger.error(f"❌ Error registering voice: {e}")
            return {
                "success": False,
                "message": f"Error registering voice: {str(e)}"
            }
    
    def verify_speaker(self, audio_data: str) -> Dict:
        """Verify if speaker is the registered owner"""
        try:
            # Decode audio
            audio = self.decode_base64_audio(audio_data)
            if audio is None:
                return {
                    "is_owner": False,
                    "confidence": 0.0,
                    "message": "Failed to decode audio data"
                }
            
            # Extract embedding
            embedding = self.extract_voice_embedding(audio)
            if embedding is None:
                return {
                    "is_owner": False,
                    "confidence": 0.0,
                    "message": "Failed to extract voice embedding"
                }
            
            if not self.owner_embeddings:
                return {
                    "is_owner": False,
                    "confidence": 0.0,
                    "message": "No registered voice embeddings found"
                }
            
            # Compare with registered embeddings using cosine similarity
            similarities = []
            for registered_embedding in self.owner_embeddings:
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
                    return {
                        "is_owner": True,
                        "confidence": confidence,
                        "message": f"Speaker verified with {confidence:.2f} confidence"
                    }
                else:
                    return {
                        "is_owner": False,
                        "confidence": confidence,
                        "message": f"Speaker not recognized (confidence: {confidence:.2f})"
                    }
            
            return {
                "is_owner": False,
                "confidence": 0.0,
                "message": "Unable to verify speaker"
            }
            
        except Exception as e:
            logger.error(f"❌ Error verifying speaker: {e}")
            return {
                "is_owner": False,
                "confidence": 0.0,
                "message": f"Error: {str(e)}"
            }
    
    def get_registration_status(self) -> Dict:
        """Get current registration status"""
        return {
            "voice_registered": len(self.owner_embeddings) > 0,
            "total_embeddings": len(self.owner_embeddings),
            "confidence_threshold": self.confidence_threshold,
            "minimum_embeddings": 3,
            "ready_for_identification": len(self.owner_embeddings) >= 3,
            "encoder_available": True  # Always available with librosa
        }
    
    def clear_embeddings(self):
        """Clear all voice embeddings"""
        self.owner_embeddings = []
        self.save_embeddings()
        logger.info("✅ All voice embeddings cleared")
    
    def analyze_voice_characteristics(self, audio_data: str) -> Dict:
        """Analyze voice characteristics for additional insights"""
        try:
            # Decode audio
            audio = self.decode_base64_audio(audio_data)
            if audio is None:
                return {
                    "success": False,
                    "message": "Failed to decode audio"
                }
            
            # Basic audio analysis
            characteristics = {}
            
            # Duration
            characteristics["duration"] = len(audio) / self.sample_rate
            
            # RMS Energy (loudness)
            characteristics["rms_energy"] = float(np.sqrt(np.mean(audio**2)))
            
            # Zero Crossing Rate (speech activity)
            characteristics["zero_crossing_rate"] = float(np.mean(librosa.feature.zero_crossing_rate(audio)[0]))
            
            # Spectral centroid (brightness)
            characteristics["spectral_centroid"] = float(np.mean(librosa.feature.spectral_centroid(audio)[0]))
            
            # MFCC features
            mfccs = librosa.feature.mfcc(audio, sr=self.sample_rate, n_mfcc=13)
            characteristics["mfcc_mean"] = mfccs.mean(axis=1).tolist()
            
            # Pitch (fundamental frequency)
            pitches, magnitudes = librosa.piptrack(audio, sr=self.sample_rate)
            pitch_values = []
            for t in range(pitches.shape[1]):
                index = magnitudes[:, t].argmax()
                pitch = pitches[index, t]
                if pitch > 0:
                    pitch_values.append(pitch)
            
            if pitch_values:
                characteristics["pitch_mean"] = float(np.mean(pitch_values))
                characteristics["pitch_std"] = float(np.std(pitch_values))
            else:
                characteristics["pitch_mean"] = 0.0
                characteristics["pitch_std"] = 0.0
            
            return {
                "success": True,
                "characteristics": characteristics
            }
            
        except Exception as e:
            logger.error(f"❌ Error analyzing voice characteristics: {e}")
            return {
                "success": False,
                "message": f"Error: {str(e)}"
            }

# Global instance
voice_system = VoiceRecognitionSystem()

def get_voice_system() -> VoiceRecognitionSystem:
    """Get global voice recognition system instance"""
    return voice_system
