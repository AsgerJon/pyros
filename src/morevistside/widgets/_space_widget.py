"""SpaceWidget provides a widget with space awareness"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QSize
from PySide6.QtWidgets import QWidget

from morevistside import parseParent


class SpaceWidget(QWidget):
  """SpaceWidget provides a widget with space awareness"""

  def __init__(self, *args, **kwargs) -> None:
    parent = parseParent(*args)
    QWidget.__init__(self, parent, )
    self.minHeight = kwargs.get('minHeight', 64)
    self.maxHeight = kwargs.get('minHeight', 144)
    self.minWidth = kwargs.get('minHeight', 64)
    self.maxWidth = kwargs.get('minHeight', 256)
    self.setMinimumSize(QSize(self.minWidth, self.minWidth))
    self.setMaximumSize(QSize(self.maxWidth, self.minWidth))
