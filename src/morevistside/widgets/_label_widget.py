"""LabelWidget provides a text containing widget"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import Qt, QMargins
from PySide6.QtGui import QColor, QPen, QPaintEvent, QPainter
from icecream import ic
from vistutils.fields import Field

from morevistside.factories import parseColor, parsePen, emptyBrush
from morevistside.widgets import FillWidget
from morevistutils.waitaminute import typeMsg

ic.configureOutput(includeContext=True)


class LabelWidget(FillWidget):
  """LabelWidget provides a text containing widget"""

  __default_text__ = 'Text'
  __text_color__ = parseColor('black')
  __text_width__ = 1
  __text_style__ = Qt.PenStyle.SolidLine
  __text_margin__ = QMargins(8, 8, 8, 8, )
  __text_padding__ = QMargins(4, 4, 4, 4, )
  __text_border__ = QMargins(2, 2, 2, 2, )

  text = Field()
  textPen = Field()

  @text.GET
  def getText(self, **kwargs) -> str:
    """Getter-function for text"""
    if hasattr(self, '_text'):
      val = getattr(self, '_text')
      if isinstance(val, str):
        return val
      e = typeMsg('val', val, str)
      raise TypeError(e)
    if kwargs.get('_recursion', False):
      raise RecursionError
    setattr(self, '_text', self.__default_text__)
    return self.getText(_recursion=True)

  @text.SET
  def setText(self, newText: str) -> None:
    """Setter-function for text"""
    if isinstance(newText, str):
      setattr(self, '_text', newText)
    else:
      e = typeMsg('newText', newText, str)
      raise TypeError(e)

  @textPen.GET
  def getTextPen(self, **kwargs) -> QPen:
    """Getter-function for pen drawing the text"""
    if hasattr(self, '_textPen'):
      if isinstance(self._textPen, QPen):
        return self._textPen
      e = typeMsg('_textPen', self._textPen, QPen)
      raise TypeError(e)
    if kwargs.get('_recursion', False):
      raise RecursionError
    textColor = self.__text_color__
    textWidth = self.__text_width__
    textStyle = self.__text_style__
    textPen = parsePen(textColor, textWidth, textStyle)
    setattr(self, '_textPen', textPen)
    return self.getTextPen(_recursion=True)

  def __init__(self, *args, **kwargs) -> None:
    nextArgs = []
    textArg = None
    for arg in args:
      if isinstance(arg, str) and textArg is None:
        textArg = arg
      else:
        nextArgs.append(arg)
    self.text = textArg
    FillWidget.__init__(self, *nextArgs, **kwargs)
    if 'textColor' in kwargs:
      color = parseColor(kwargs['textColor'])
      if isinstance(color, QColor):
        self.__text_color__ = color
    if 'textWidth' in kwargs:
      width = kwargs['textWidth']
      if isinstance(width, int):
        self.__text_width__ = width
      elif isinstance(width, float):
        self.__text_width__ = int(width)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Paints the widget"""
    FillWidget.paintEvent(self, event)
    painter = QPainter()
    painter.begin(self)
    viewRect = painter.viewport()
    borderRect = viewRect.marginsRemoved(self.__text_margin__)
    paddingRect = borderRect.marginsRemoved(self.__text_border__)
    textRect = paddingRect.marginsRemoved(self.__text_padding__)
    painter.setPen(self.borderPen)
    painter.setBrush(emptyBrush())
    painter.drawRect(borderRect)
    painter.drawText(textRect, Qt.AlignCenter, self.text)
    painter.end()
