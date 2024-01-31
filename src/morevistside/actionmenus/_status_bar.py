"""StatusBar subclasses QStatusBar """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QStatusBar, QLabel

from morevistside import parseParent
from morevistutils.fields import SpecialField


class StatusBar(QStatusBar):
  """StatusBar subclasses QStatusBar """

  label = SpecialField(QLabel)

  @label.CREATE
  def _createLabel(self, *args, **kwargs) -> QLabel:
    """Creator function for status bar label widget"""
    widget = QLabel(self, )
    widget.setText('Welcome!')
    return widget

  def __init__(self, *args, **kwargs) -> None:
    parent = parseParent(*args)
    QStatusBar.__init__(self, parent, )

  def initUI(self, ) -> None:
    """Initializes the UI by setting up widgets and layouts. Subclasses
    must implement this method."""
    self.addPermanentWidget(self.label)

  def show(self) -> None:
    """Implements initUI before show"""
    self.initUI()
    return QStatusBar.show(self)
