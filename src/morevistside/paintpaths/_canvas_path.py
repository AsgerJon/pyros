"""CanvasPath is intended to collect multiple smaller and simpler paths on
it."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QPainterPath
from vistutils.fields import Field


class CanvasPath(QPainterPath):
  """CanvasPath is intended to collect multiple smaller and simpler paths on
  it."""

  width = Field()
  height = Field()

  @width.GET
  def _getWidth(self) -> int:
    """Getter-function for width of bounding rectangle"""
    return self.boundingRect().width()

  @height.GET
  def _getHeight(self) -> int:
    """Getter-function for height of bounding rectangle"""
    return self.boundingRect().height()

  def __init__(self, *args, **kwargs) -> None:
    QPainterPath.__init__(self, )
