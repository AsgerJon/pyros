"""TESTER"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QWidget

from morevistutils.metas import AbstractMetaclass


class SomeClass(metaclass=AbstractMetaclass):
  """YOLO"""

  a: int
  b: int

  def __init__(self, *args, **kwargs) -> None:
    pass
