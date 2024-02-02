"""AbstractPaintPath provides an abstract baseclass for painter paths."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from PySide6.QtGui import QPainterPath
from vistutils.fields import Field


class AbstractPaintPath(QPainterPath):
  """AbstractPaintPath provides an abstract baseclass for painter paths."""

  def __init__(self, *paths, **kwargs) -> None:
    QPainterPath.__init__(self)
    pathArgs = []
    for path in paths:
      if isinstance(path, (AbstractPaintPath, QPainterPath)):
        pathArgs.append(path)
       