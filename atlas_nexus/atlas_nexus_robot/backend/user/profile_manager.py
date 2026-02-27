"""
Profile Manager for ATLAS NEXUS Personal Assistant
Handles user profile data, preferences, and interaction history
"""

import logging
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Any
from sqlalchemy.orm import Session
from database.models import UserProfile, Interaction, UserPreference, get_db

logger = logging.getLogger('ATLAS_PROFILE_MANAGER')

class ProfileManager:
    """Manages user profiles, preferences, and interaction history"""
    
    def __init__(self):
        self.current_user = None
        self.session_context = {}
        logger.info("✅ Profile Manager initialized")
    
    def create_user_profile(self, profile_data: Dict) -> Dict:
        """Create a new user profile"""
        try:
            db = next(get_db())
            
            # Check if user already exists
            existing_user = db.query(UserProfile).filter(UserProfile.name == profile_data.get('name')).first()
            if existing_user:
                return {
                    "success": False,
                    "message": "User with this name already exists",
                    "user_id": existing_user.id
                }
            
            # Create new user profile
            user = UserProfile(
                name=profile_data.get('name'),
                nickname=profile_data.get('nickname'),
                birthdate=profile_data.get('birthdate'),
                profession=profile_data.get('profession'),
                preferred_temperature=profile_data.get('preferred_temperature', 22.0),
                preferred_formality=profile_data.get('preferred_formality', 'casual'),
                wake_up_time=profile_data.get('wake_up_time'),
                sleep_time=profile_data.get('sleep_time')
            )
            
            # Set work hours if provided
            if 'work_hours' in profile_data:
                user.set_work_hours(profile_data['work_hours'])
            
            db.add(user)
            db.commit()
            db.refresh(user)
            
            self.current_user = user
            
            logger.info(f"✅ Created user profile: {user.name}")
            
            return {
                "success": True,
                "message": "User profile created successfully",
                "user_id": user.id,
                "user_data": {
                    "name": user.name,
                    "nickname": user.nickname,
                    "preferred_formality": user.preferred_formality
                }
            }
            
        except Exception as e:
            logger.error(f"❌ Error creating user profile: {e}")
            return {
                "success": False,
                "message": f"Error creating user profile: {str(e)}"
            }
        finally:
            db.close()
    
    def get_user_profile(self, user_id: Optional[int] = None) -> Dict:
        """Get user profile information"""
        try:
            db = next(get_db())
            
            if user_id:
                user = db.query(UserProfile).filter(UserProfile.id == user_id).first()
            elif self.current_user:
                user = db.query(UserProfile).filter(UserProfile.id == self.current_user.id).first()
            else:
                # Get first user (for single-user system)
                user = db.query(UserProfile).first()
            
            if not user:
                return {
                    "success": False,
                    "message": "User profile not found"
                }
            
            self.current_user = user
            
            return {
                "success": True,
                "user_data": {
                    "id": user.id,
                    "name": user.name,
                    "nickname": user.nickname,
                    "birthdate": user.birthdate.isoformat() if user.birthdate else None,
                    "profession": user.profession,
                    "preferred_temperature": user.preferred_temperature,
                    "preferred_formality": user.preferred_formality,
                    "wake_up_time": user.wake_up_time,
                    "sleep_time": user.sleep_time,
                    "work_hours": user.get_work_hours(),
                    "face_registered": user.face_registered,
                    "voice_registered": user.voice_registered,
                    "created_at": user.created_at.isoformat()
                }
            }
            
        except Exception as e:
            logger.error(f"❌ Error getting user profile: {e}")
            return {
                "success": False,
                "message": f"Error getting user profile: {str(e)}"
            }
        finally:
            db.close()
    
    def update_user_profile(self, updates: Dict) -> Dict:
        """Update user profile information"""
        try:
            if not self.current_user:
                return {
                    "success": False,
                    "message": "No current user set"
                }
            
            db = next(get_db())
            user = db.query(UserProfile).filter(UserProfile.id == self.current_user.id).first()
            
            if not user:
                return {
                    "success": False,
                    "message": "User not found"
                }
            
            # Update allowed fields
            updatable_fields = [
                'nickname', 'birthdate', 'profession', 'preferred_temperature',
                'preferred_formality', 'wake_up_time', 'sleep_time', 'face_registered', 'voice_registered'
            ]
            
            for field in updatable_fields:
                if field in updates:
                    setattr(user, field, updates[field])
            
            # Handle work hours separately
            if 'work_hours' in updates:
                user.set_work_hours(updates['work_hours'])
            
            user.updated_at = datetime.utcnow()
            db.commit()
            db.refresh(user)
            
            logger.info(f"✅ Updated user profile: {user.name}")
            
            return {
                "success": True,
                "message": "User profile updated successfully",
                "updated_fields": list(updates.keys())
            }
            
        except Exception as e:
            logger.error(f"❌ Error updating user profile: {e}")
            return {
                "success": False,
                "message": f"Error updating user profile: {str(e)}"
            }
        finally:
            db.close()
    
    def record_interaction(self, interaction_data: Dict) -> Dict:
        """Record a user interaction"""
        try:
            db = next(get_db())
            
            interaction = Interaction(
                interaction_type=interaction_data.get('interaction_type', 'chat'),
                user_input=interaction_data.get('user_input'),
                robot_response=interaction_data.get('robot_response'),
                user_emotion=interaction_data.get('user_emotion'),
                session_id=interaction_data.get('session_id', 'default'),
                successful=interaction_data.get('successful', True)
            )
            
            # Set context if provided
            if 'context' in interaction_data:
                interaction.set_context(interaction_data['context'])
            
            db.add(interaction)
            db.commit()
            db.refresh(interaction)
            
            logger.info(f"✅ Recorded interaction: {interaction.interaction_type}")
            
            return {
                "success": True,
                "message": "Interaction recorded successfully",
                "interaction_id": interaction.id
            }
            
        except Exception as e:
            logger.error(f"❌ Error recording interaction: {e}")
            return {
                "success": False,
                "message": f"Error recording interaction: {str(e)}"
            }
        finally:
            db.close()
    
    def get_interaction_history(self, limit: int = 50, interaction_type: Optional[str] = None) -> Dict:
        """Get interaction history"""
        try:
            db = next(get_db())
            
            query = db.query(Interaction).order_by(Interaction.timestamp.desc())
            
            if interaction_type:
                query = query.filter(Interaction.interaction_type == interaction_type)
            
            interactions = query.limit(limit).all()
            
            history = []
            for interaction in interactions:
                history.append({
                    "id": interaction.id,
                    "timestamp": interaction.timestamp.isoformat(),
                    "interaction_type": interaction.interaction_type,
                    "user_input": interaction.user_input,
                    "robot_response": interaction.robot_response,
                    "user_emotion": interaction.user_emotion,
                    "context": interaction.get_context(),
                    "session_id": interaction.session_id,
                    "successful": interaction.successful
                })
            
            return {
                "success": True,
                "history": history,
                "total_count": len(history)
            }
            
        except Exception as e:
            logger.error(f"❌ Error getting interaction history: {e}")
            return {
                "success": False,
                "message": f"Error getting interaction history: {str(e)}"
            }
        finally:
            db.close()
    
    def update_preference(self, category: str, item: str, preference_score: float, context: Optional[Dict] = None) -> Dict:
        """Update or create a user preference"""
        try:
            db = next(get_db())
            
            # Check if preference exists
            preference = db.query(UserPreference).filter(
                UserPreference.category == category,
                UserPreference.item == item
            ).first()
            
            if preference:
                # Update existing preference
                preference.preference_score = preference_score
                preference.frequency += 1
                preference.last_mentioned = datetime.utcnow()
                
                # Update context if provided
                if context:
                    current_context = preference.get_context()
                    current_context.update(context)
                    preference.set_context(current_context)
            else:
                # Create new preference
                preference = UserPreference(
                    category=category,
                    item=item,
                    preference_score=preference_score,
                    frequency=1
                )
                
                if context:
                    preference.set_context(context)
                
                db.add(preference)
            
            db.commit()
            db.refresh(preference)
            
            logger.info(f"✅ Updated preference: {category}/{item} = {preference_score}")
            
            return {
                "success": True,
                "message": "Preference updated successfully",
                "preference_id": preference.id,
                "current_score": preference.preference_score,
                "frequency": preference.frequency
            }
            
        except Exception as e:
            logger.error(f"❌ Error updating preference: {e}")
            return {
                "success": False,
                "message": f"Error updating preference: {str(e)}"
            }
        finally:
            db.close()
    
    def get_preferences(self, category: Optional[str] = None) -> Dict:
        """Get user preferences"""
        try:
            db = next(get_db())
            
            query = db.query(UserPreference).order_by(UserPreference.preference_score.desc())
            
            if category:
                query = query.filter(UserPreference.category == category)
            
            preferences = query.all()
            
            result = {}
            for pref in preferences:
                if pref.category not in result:
                    result[pref.category] = []
                
                result[pref.category].append({
                    "item": pref.item,
                    "preference_score": pref.preference_score,
                    "frequency": pref.frequency,
                    "last_mentioned": pref.last_mentioned.isoformat(),
                    "context": pref.get_context()
                })
            
            return {
                "success": True,
                "preferences": result,
                "total_items": len(preferences)
            }
            
        except Exception as e:
            logger.error(f"❌ Error getting preferences: {e}")
            return {
                "success": False,
                "message": f"Error getting preferences: {str(e)}"
            }
        finally:
            db.close()
    
    def get_user_context(self) -> Dict:
        """Get comprehensive user context for AI responses"""
        try:
            if not self.current_user:
                return {"context": {}, "message": "No current user"}
            
            # Get user profile
            profile_result = self.get_user_profile()
            if not profile_result["success"]:
                return {"context": {}, "message": "Failed to get user profile"}
            
            user_data = profile_result["user_data"]
            
            # Get recent interactions
            recent_interactions = self.get_interaction_history(limit=10)
            
            # Get preferences
            preferences_result = self.get_preferences()
            
            # Get current time context
            now = datetime.now()
            current_hour = now.hour
            
            # Determine time of day
            if 5 <= current_hour < 12:
                time_of_day = "morning"
            elif 12 <= current_hour < 17:
                time_of_day = "afternoon"
            elif 17 <= current_hour < 22:
                time_of_day = "evening"
            else:
                time_of_day = "night"
            
            # Build comprehensive context
            context = {
                "user": user_data,
                "time_context": {
                    "current_time": now.isoformat(),
                    "hour": current_hour,
                    "time_of_day": time_of_day,
                    "day_of_week": now.strftime("%A")
                },
                "recent_interactions": recent_interactions.get("history", [])[:5],
                "preferences": preferences_result.get("preferences", {}),
                "session_context": self.session_context
            }
            
            return {
                "success": True,
                "context": context
            }
            
        except Exception as e:
            logger.error(f"❌ Error getting user context: {e}")
            return {
                "success": False,
                "message": f"Error getting user context: {str(e)}"
            }
    
    def set_session_context(self, key: str, value: Any):
        """Set session context variable"""
        self.session_context[key] = value
    
    def get_session_context(self, key: str) -> Any:
        """Get session context variable"""
        return self.session_context.get(key)
    
    def clear_session_context(self):
        """Clear session context"""
        self.session_context.clear()

# Global instance
profile_manager = ProfileManager()

def get_profile_manager() -> ProfileManager:
    """Get global profile manager instance"""
    return profile_manager
