"""PushButton subclasses LabelWidget and provides a push button widget. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QEvent, Qt
from PySide6.QtGui import QPaintEvent, QMouseEvent, QPainter, QColor, \
  QEnterEvent, QBrush, QFont, QGuiApplication
from icecream import ic
from vistutils.fields import TypedField

from morevistside.factories import solidBrush, parseFont
from morevistside.widgets import LabelWidget, SpaceWidget

ic.configureOutput(includeContext=True)


class PushButton(SpaceWidget):
  """PushButton subclasses FillWidget and provides a push button widget. """

  @staticmethod
  def mouseButtons() -> int:
    """Getter-function for the mouse buttons."""
    return QGuiApplication.mouseButtons()

  underMouse = TypedField(bool, False)
  text = TypedField(str, 'Button', )

  def __init__(self, *args, **kwargs) -> None:
    SpaceWidget.__init__(self, *args, 32, 256, 32, 256)
    for arg in args:
      if isinstance(arg, str):
        self.text = arg

  def enterEvent(self, event: QEnterEvent) -> None:
    """Reimplementation to handle mouse enter events. """
    self.underMouse = True
    self.update()

  def leaveEvent(self, event: QEvent) -> None:
    """Reimplementation to handle mouse leave events. """
    self.underMouse = False
    self.update()

  def mousePressEvent(self, event: QMouseEvent) -> None:
    """Reimplementation to handle mouse press events. """

  def mouseReleaseEvent(self, event: QMouseEvent) -> None:
    """Reimplementation to handle mouse release events. """

  def getFillColor(self) -> QColor:
    """Getter-function for the fill color."""

    if self.underMouse:
      return QColor(255, 0, 144, 255)
    return QColor(0, 255, 0, 255)

  def getFillBrush(self) -> QBrush:
    """Getter-function for the fill brush."""
    return solidBrush(self.getFillColor())

  def getTextPen(self) -> QColor:
    """Getter-function for the text pen."""
    if self.underMouse:
      return QColor(0, 0, 0, 255)
    return QColor(95, 95, 95, 255)

  def paintEvent(self, event: QPaintEvent) -> None:
    """Reimplementation to handle paint events. """
    textPen = self.getTextPen()
    fillBrush = self.getFillBrush()
    alignFlag = Qt.AlignmentFlag.AlignCenter
    painter = QPainter()
    painter.begin(self)
    viewRect = painter.viewport()
    painter.setBrush(fillBrush)
    painter.setPen(textPen)
    painter.setFont(parseFont('Courier', 24))
    painter.drawRoundedRect(viewRect, 4, 4)
    painter.drawText(viewRect, alignFlag, self.text, )
    painter.end()
