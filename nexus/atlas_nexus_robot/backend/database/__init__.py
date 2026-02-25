"""
Database module initialization
"""

from .models import (Interaction, UserPreference, UserProfile, create_tables,
                     get_db)

__all__ = ["create_tables", "get_db", "UserProfile", "Interaction", "UserPreference"]
