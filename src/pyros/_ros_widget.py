"""RosWidget"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QPaintEvent, QPainter, QMouseEvent
from PySide6.QtWidgets import QWidget


class RosWidget(QWidget):
  """RosWidget"""

  def paintEvent(self, event: QPaintEvent) -> None:
    """Bla"""

    p = QPainter()
    p.begin(self)

    p.end()

  def mousePressEvent(self, event: QMouseEvent) -> None:
    """Mouse event"""
