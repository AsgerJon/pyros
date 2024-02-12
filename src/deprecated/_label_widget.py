"""LabelWidget subclasses AbstractPaintWidget and implements a simple
label widget"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Signal, QSize
from PySide6.QtGui import QColor, QFont
from icecream import ic
from vistutils import maybe
from vistutils.fields import Field

from deprecated.paintlayers import TextLayer, SolidLayer
from deprecated.widgets import AbstractPaintWidget
from morevistutils.waitaminute import typeMsg

ic.configureOutput(includeContext=True)


class LabelWidget(AbstractPaintWidget):
  """LabelWidget subclasses AbstractPaintWidget and implements a simple
  label widget"""

  __default_text__ = 'Hello World!'

  text = Field()
  textChanged = Signal(str)

  @text.GET
  def _getText(self, ) -> str:
    """Getter-function for inner text"""
    return self.__inner_text__

  @text.SET
  def _setText(self, text: str) -> None:
    """Setter-function for inner text"""
    self.__inner_text__ = text
    self.textChanged.emit(text)

  fill = SolidLayer(QColor(255, 255, 191, 255))
  label = TextLayer(font=QFont('Courier', 24))

  def _getMinSize(self) -> QSize:
    """Reimplementation of minimum size getter to include a requirement
    that the size be not smaller than the bounding rect on the text"""
    if self.label is not None:
      boundSize = self.label.getBoundingSize()
      boundWidth, boundHeight = boundSize.width(), boundSize.height()
      if boundWidth < self.minWidth and boundHeight < self.minHeight:
        return AbstractPaintWidget._getMinSize(self)
      newWidth = max(boundWidth, self.minWidth)
      newHeight = max(boundHeight, self.minHeight)
      return QSize(newWidth, newHeight)
    raise RuntimeError

  def __init__(self, *args, **kwargs) -> None:
    textArg = None
    for arg in args:
      if isinstance(arg, str) and textArg is None:
        textArg = arg
    textVal = maybe(textArg, self.__default_text__)
    if isinstance(textVal, str):
      self.__inner_text__ = textVal
    else:
      e = typeMsg('textVal', textVal, str)
      raise TypeError(e)
    AbstractPaintWidget.__init__(self, *args, **kwargs)
    ic(self.label)
