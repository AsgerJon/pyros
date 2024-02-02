"""LabelWidget subclasses AbstractPaintWidget and implements a simple
label widget"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QColor, QFont

from morevistside.paintlayers import TextLayer, SolidLayer
from morevistside.widgets import AbstractPaintWidget
from morevistutils.fields import TypedField


class LabelWidget(AbstractPaintWidget):
  """LabelWidget subclasses AbstractPaintWidget and implements a simple
  label widget"""

  text = TypedField(str, 'Hello World')

  fill = SolidLayer(QColor(255, 255, 191, 255))
  label = TextLayer(font=QFont('Courier', 24))

  def __init__(self, *args, **kwargs) -> None:
    AbstractPaintWidget.__init__(self, *args, **kwargs)
