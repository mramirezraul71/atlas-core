"""
Database models for ATLAS NEXUS Personal Assistant
User profiles, interactions, and preferences
"""

from sqlalchemy import create_engine, Column, Integer, String, Float, DateTime, Text, Boolean
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from datetime import datetime
import json

Base = declarative_base()

class UserProfile(Base):
    """User profile information"""
    __tablename__ = 'user_profiles'
    
    id = Column(Integer, primary_key=True)
    name = Column(String(100), nullable=False)
    nickname = Column(String(50))
    birthdate = Column(DateTime)
    profession = Column(String(100))
    preferred_temperature = Column(Float, default=22.0)
    preferred_formality = Column(String(20), default='casual')  # formal/casual/friendly
    wake_up_time = Column(String(10))  # HH:MM format
    sleep_time = Column(String(10))   # HH:MM format
    work_hours = Column(Text)  # JSON string
    face_registered = Column(Boolean, default=False)
    voice_registered = Column(Boolean, default=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    def get_work_hours(self):
        """Parse work hours JSON"""
        if self.work_hours:
            try:
                return json.loads(self.work_hours)
            except:
                return {}
        return {}
    
    def set_work_hours(self, hours_dict):
        """Set work hours as JSON"""
        self.work_hours = json.dumps(hours_dict)

class Interaction(Base):
    """User interaction history"""
    __tablename__ = 'interactions'
    
    id = Column(Integer, primary_key=True)
    timestamp = Column(DateTime, default=datetime.utcnow)
    interaction_type = Column(String(50))  # chat, command, voice, gesture
    user_input = Column(Text)
    robot_response = Column(Text)
    user_emotion = Column(String(20))  # happy, sad, angry, neutral, stressed, etc.
    context = Column(Text)  # JSON string with additional context
    session_id = Column(String(50))
    successful = Column(Boolean, default=True)
    
    def get_context(self):
        """Parse context JSON"""
        if self.context:
            try:
                return json.loads(self.context)
            except:
                return {}
        return {}
    
    def set_context(self, context_dict):
        """Set context as JSON"""
        self.context = json.dumps(context_dict)

class UserPreference(Base):
    """User preferences and learning"""
    __tablename__ = 'user_preferences'
    
    id = Column(Integer, primary_key=True)
    category = Column(String(50))  # music, food, activities, topics, etc.
    item = Column(String(100))    # specific item within category
    preference_score = Column(Float, default=0.0)  # -1.0 (dislike) to 1.0 (like)
    frequency = Column(Integer, default=0)  # how often mentioned
    last_mentioned = Column(DateTime, default=datetime.utcnow)
    context = Column(Text)  # JSON string with additional context
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    
    def get_context(self):
        """Parse context JSON"""
        if self.context:
            try:
                return json.loads(self.context)
            except:
                return {}
        return {}
    
    def set_context(self, context_dict):
        """Set context as JSON"""
        self.context = json.dumps(context_dict)

# Database setup
DATABASE_URL = "sqlite:///./atlas_nexus.db"

engine = create_engine(DATABASE_URL, echo=False)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

def create_tables():
    """Create all database tables"""
    Base.metadata.create_all(bind=engine)

def get_db():
    """Get database session"""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

if __name__ == "__main__":
    # Create tables when run directly
    create_tables()
    print("✅ Database tables created successfully")
