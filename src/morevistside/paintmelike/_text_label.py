"""TextLabel provides a label with underlying fields 'text' and 'font'. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from PySide6.QtCore import Qt, QRect, QPoint, QMargins
from PySide6.QtGui import QPainter, QPaintEvent, QFont, QPen, QColor, \
  QFontMetrics
from icecream import ic
from vistutils import maybe

from morevistside.paintmelike import AbstractStencil, TextDescriptor, \
  FontDescriptor

ic.configureOutput(includeContext=True)


class TextLabel(AbstractStencil):
  """TextLabel provides a label with underlying fields 'text' and 'font'. """

  def __init__(self, *args, **kwargs) -> None:
    AbstractStencil.__init__(self, *args, **kwargs)
    defaultText = None
    defaultFont = None
    fontFamily = None
    fontSize = None
    for arg in args:
      if isinstance(arg, str):
        if defaultText is None:
          defaultText = arg
        elif fontFamily is None:
          fontFamily = arg
      if isinstance(arg, QFont) and defaultFont is None:
        defaultFont = arg
      if isinstance(arg, int) and fontSize is None:
        fontSize = arg
    if defaultText is None:
      defaultText = TextDescriptor.__default_value__
    if defaultFont is None:
      font = QFont()
      family = maybe(fontFamily, FontDescriptor.__default_family__)
      size = maybe(fontSize, FontDescriptor.__default_size__)
      if isinstance(family, str):
        font.setFamily(family)
      if isinstance(fontSize, int):
        font.setPointSize(fontSize)
      defaultFont = font
    self.__default_text__ = defaultText
    self.__default_font__ = defaultFont

  def _getPrivateTextName(self) -> str:
    """Getter-function for the private name of the text"""
    return '__%s_text__' % self._getFieldName()

  def _getPrivateFontName(self) -> str:
    """Getter-function for the private name of the font"""
    return '__%s_font__' % self._getFieldName()

  def __prepare_owner__(self, owner: type) -> type:
    """Prepares the owner by adding fields"""
    owner = AbstractStencil.__prepare_owner__(self, owner)
    fieldName = self.__field_name__
    pvtTextName = self._getPrivateTextName()
    setattr(owner, pvtTextName, TextDescriptor(self.__default_text__))
    pvtFontName = self._getPrivateFontName()
    setattr(owner, pvtFontName, FontDescriptor(self.__default_font__))
    return owner

  def _getTextDescriptor(self) -> TextDescriptor:
    """Getter-function for the text descriptor on the owner"""
    owner = self._getFieldOwner()
    pvtName = self._getPrivateTextName()
    if not hasattr(owner, pvtName, ):
      raise AttributeError(pvtName)
    return getattr(owner, pvtName)

  def _getFontDescriptor(self) -> FontDescriptor:
    """Getter-function for the font descriptor on the owner"""
    owner = self._getFieldOwner()
    pvtName = self._getPrivateFontName()
    if not hasattr(owner, pvtName, ):
      raise AttributeError(pvtName)
    return getattr(owner, pvtName)

  def __get__(self, instance: Any, owner: type) -> str:
    """Returns the text"""
    pvtName = self._getPrivateTextName()
    if instance is None:
      return getattr(owner, pvtName)
    return getattr(instance, pvtName, )

  def __set__(self, instance: Any, value: str) -> None:
    """Sets the text"""
    pvtName = self._getPrivateTextName()
    setattr(instance, pvtName, value)

  def __delete__(self, instance: Any, ) -> None:
    """Deletes the text"""
    pvtName = self._getPrivateTextName()
    delattr(instance, pvtName)

  def paintMeLike(self, painter: QPainter, event: QPaintEvent) -> None:
    """Painting contribution"""
    AbstractStencil.paintMeLike(self, painter, event)
    instance = painter.device()
    owner = self._getFieldOwner()
    font = getattr(instance, self._getPrivateFontName())
    text = getattr(instance, self._getPrivateTextName())
    pen = QPen()
    pen.setColor(QColor(0, 0, 0, 255))
    pen.setWidth(1)
    pen.setStyle(Qt.PenStyle.SolidLine)
    painter.setPen(pen)
    painter.setFont(font)
    viewRect = painter.viewport()
    fontMetrics = QFontMetrics(font)
    tightSize = fontMetrics.tightBoundingRect(text).size()
    textSize = fontMetrics.boundingRect(text).size()
    textRect = QRect(QPoint(0, 0), textSize)
    margins = QMargins(2, 2, 2, 2, )
    # textRect = textRect + margins
    textRect.moveCenter(viewRect.center())
    painter.drawText(textRect, text)
