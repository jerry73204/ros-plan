"""
Visitor pattern for extracting metadata from launch entities
"""

from .entity import visit_entity
from .session import CollectionSession

__all__ = ["visit_entity", "CollectionSession"]
