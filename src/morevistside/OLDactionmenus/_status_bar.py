"""StatusBar subclasses QStatusBar """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QStatusBar

from morevistside import parseParent
from morevistside.widgets import LabelWidget, WidgetField


class StatusBar(QStatusBar):
  """StatusBar subclasses QStatusBar """

  label = WidgetField(LabelWidget, 'Welcome to Qt!', '#666666FF', )

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
