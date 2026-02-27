"""
Database module initialization
"""

from .models import create_tables, get_db, UserProfile, Interaction, UserPreference

__all__ = ['create_tables', 'get_db', 'UserProfile', 'Interaction', 'UserPreference']
