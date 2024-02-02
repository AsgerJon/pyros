"""BaseWidget is a blank subclass of QWidget."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QSize
from PySide6.QtGui import QResizeEvent
from PySide6.QtWidgets import QWidget
from vistutils.fields import Field

from morevistside import parseParent
from morevistutils.fields import TypedField


class BaseWidget(QWidget):
  """BaseWidget is a blank subclass of QWidget."""

  minWidth = TypedField(int, 32)
  minHeight = TypedField(int, 32)
  minSize = Field()

  @minSize.GET
  def _getMinSize(self) -> QSize:
    """Getter-function for minimum-size"""
    return QSize(self.minWidth, self.minHeight)

  def resizeEvent(self, event: QResizeEvent) -> None:
    """Implementation of resizing"""
    oldSize = self.size()
    newWidth = min(event.size().width(), self.minWidth)
    newHeight = min(event.size().height(), self.minHeight)
    newSize = QSize(newWidth, newHeight)
    newEvent = QResizeEvent(newSize, oldSize)
    QWidget.resizeEvent(self, newEvent)

  def __init__(self, *args, **kwargs) -> None:
    parent = parseParent(*args)
    QWidget.__init__(self, parent)
