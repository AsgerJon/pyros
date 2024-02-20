"""TestWindow"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QMainWindow, QWidget, QGridLayout, QLabel

from morevistside.widgets import FillWidget, BaseWidget, LabelWidget
from morevistutils.fields import ClassField


class TestWindow(QMainWindow):
  """Tester """

  baseWidget = ClassField(BaseWidget, )
  baseLayout = ClassField(QGridLayout, )
  welcomeBanner = ClassField(LabelWidget, 'Welcome!')
  goodbyeBanner = ClassField(LabelWidget, 'Goodbye!')

  def __init__(self, *args, **kwargs) -> None:
    QMainWindow.__init__(self, )

  def initUI(self) -> None:
    """Sets up the layout and widgets."""
    self.baseLayout.addWidget(self.welcomeBanner, 0, 0)
    self.baseLayout.addWidget(self.goodbyeBanner, 1, 0)
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)

  def show(self) -> None:
    """Shows the window."""
    self.initUI()
    QMainWindow.show(self)
