"""This module contains the types used by the shiboken module."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QObject

ShibokenType = type(QObject)


def validateClass(QClass: type) -> bool:
  """Validate that the given class is a ShibokenType"""
  return isinstance(QClass, ShibokenType)
