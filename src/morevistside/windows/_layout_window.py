"""LayoutWindow subclasses BaseWindow and provides the layout management
for the main application window. This class is responsible for creating
widgets and layouts in the main application window. It is not responsible
for connecting any signals and slots to and from the visual elements. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QWidget, QGridLayout, QLabel, QLCDNumber

from morevistside.widgets import WidgetField, LabelWidget, PlotWidget
from morevistside.windows import BaseWindow


class LayoutWindow(BaseWindow):
  """LayoutWindow subclasses BaseWindow and provides the layout management
  for the main application window. This class is responsible for creating
  widgets and layouts in the main application window. It is not responsible
  for connecting any signals and slots to and from the visual elements. """

  helloWorld = WidgetField(LabelWidget, 'yolo', 128, 64)
  clock = WidgetField(QLCDNumber, )
  plot = WidgetField(PlotWidget, 256, 128, 512, 256)

  def __init__(self, *args, **kwargs) -> None:
    BaseWindow.__init__(self, *args, **kwargs)
    self.baseWidget = QWidget()
    self.baseLayout = QGridLayout()

  def initUI(self, ) -> None:
    """Sets up the widgets"""
    self.baseLayout.addWidget(self.helloWorld, 0, 0)
    self.baseLayout.addWidget(self.clock, 0, 1)
    self.baseLayout.addWidget(self.plot, 1, 0, 1, 2)
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)
