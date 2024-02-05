"""TextLayer provides painting of text. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtCore import Qt, QRect, QSize, QPoint
from PySide6.QtGui import QPainter, QPen, QColor, QFontMetrics
from vistutils import maybeType, maybe
from vistutils.waitaminute import typeMsg

from morevistside.paintlayers import AbstractPaintLayer


class TextLayer(AbstractPaintLayer):
  """TextLayer provides painting of text. """

  def __init__(self, *args, **kwargs) -> None:
    AbstractPaintLayer.__init__(self, *args, **kwargs)
    self.__font_metrics__ = None
    self._font = kwargs.get('font', None)
    if self._font is None:
      self._font = self._parseFont(*args)
    self._pen = self._parsePen(*args)
    flagsArg = self._parseTextFlags()
    flagsDefault = Qt.TextFlag.TextWordWrap
    flags = maybe(flagsArg, flagsDefault)
    if isinstance(flags, Qt.TextFlag):
      self._flags = flags
    else:
      e = typeMsg('flags', flags, Qt.TextFlag)
      raise TypeError(e)
    textArg = maybeType(str, *args)
    textDefault = 'TextLayer'
    text = maybe(textArg, textDefault)
    if isinstance(text, str):
      self._text = text
    else:
      e = typeMsg('text', text, str)
      raise TypeError(e)

  def paintMeLike(self, painter: QPainter) -> None:
    """Specifies the painting operation. """
    fontPen = QPen()
    fontPen.setColor(QColor(0, 0, 0, 255))
    fontPen.setWidthF(1)
    fontPen.setStyle(Qt.PenStyle.SolidLine)

    instance = painter.device()
    painter.setFont(self._font)
    painter.setPen(fontPen)
    viewRect = painter.viewport()
    flags = self._flags
    text = self._text
    textRect = painter.boundingRect(viewRect, flags, text)
    painter.drawText(textRect, flags, text)

  def __set__(self, instance: Any, value: Any) -> None:
    """Setter function changes the text shown"""
    if isinstance(value, str):
      instance.text = value

  def _createFontMetrics(self) -> None:
    """Creator-function for font metrics"""
    self.__font_metrics__ = QFontMetrics(self._font)

  def _getFontMetrics(self, **kwargs) -> QFontMetrics:
    """Getter-function for font metrics"""
    if self.__font_metrics__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createFontMetrics()
      return self._getFontMetrics(_recursion=True)
    return self.__font_metrics__

  def getBoundingRect(self) -> QRect:
    """Getter-function for the bounding rectangle required to bound the
    given text"""
    fontMetrics = self._getFontMetrics()
    return fontMetrics.boundingRect(self._text, )

  def getBoundingSize(self) -> QSize:
    """Getter-function for the size of the bounding rectangle required to
    bound the given text."""
    return self.getBoundingRect().size()
