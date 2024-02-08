"""LayoutWindow subclasses BaseWindow and provides the layout management
for the main application window. This class is responsible for creating
widgets and layouts in the main application window. It is not responsible
for connecting any signals and slots to and from the visual elements. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

import rospy
from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import QWidget, QGridLayout, QLabel, QLCDNumber
from icecream import ic
from rospy import Subscriber
from std_msgs.msg import Float64

from morevistside import ResizeFilter
from morevistside.widgets import WidgetField, LabelWidget, PlotWidget
from morevistside.windows import BaseWindow
from yolomsg import Float32Stamped

ic.configureOutput(includeContext=True)


class LayoutWindow(BaseWindow):
  """LayoutWindow subclasses BaseWindow and provides the layout management
  for the main application window. This class is responsible for creating
  widgets and layouts in the main application window. It is not responsible
  for connecting any signals and slots to and from the visual elements. """

  helloWorld = WidgetField(LabelWidget, 'yolo', 128, 64)
  plot = WidgetField(PlotWidget, 256, 128, 512, 256)
  debug = WidgetField(LabelWidget, 'DEBUG', 256, 64)

  def __init__(self, *args, **kwargs) -> None:
    BaseWindow.__init__(self, *args, **kwargs)
    self.baseWidget = QWidget()
    self.baseLayout = QGridLayout()

  def initUI(self, ) -> None:
    """Sets up the widgets"""
    self.baseLayout.addWidget(self.helloWorld, 0, 0, 1, 2)
    self.baseLayout.addWidget(self.plot, 1, 0, 1, 2)
    self.baseLayout.addWidget(self.debug, 2, 0, 1, 2)
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)
