"""BaseWidget is a blank subclass of QWidget."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os

from PySide6.QtCore import QSize, QRect
from PySide6.QtWidgets import QWidget
from icecream import ic
from vistutils import maybe
from vistutils.fields import Field

from morevistside import parseParent


class BaseWidget(QWidget):
  """BaseWidget is a blank subclass of QWidget."""

  __min_widget_size__ = '32, 32'

  minSize = Field()

  @classmethod
  @minSize.GET
  def _getMinSize(cls, *args, **kwargs) -> QSize:
    """Getter-function for minimum size"""
    envVal = os.environ.get('MIN_WIDGET_SIZE', None)
    fallbackVal = cls.__min_widget_size__
    val = maybe(envVal, fallbackVal)
    width, height = val.split(',')
    width, height = int(width.strip()), int(height.strip())
    return QSize(32, 32)

  def __init__(self, *args, **kwargs) -> None:
    parent = parseParent(*args)
    QWidget.__init__(self, parent, )
    self.setMinimumSize(self.minSize)
