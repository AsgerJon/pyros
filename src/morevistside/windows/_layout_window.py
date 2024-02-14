"""LayoutWindow subclasses BaseWindow and provides the layout management
for the main application window. This class is responsible for creating
widgets and layouts in the main application window. It is not responsible
for connecting any signals and slots to and from the visual elements. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QWidget, QGridLayout, QPushButton
from icecream import ic

from morevistside.widgets import PlotWidget, LabelWidget, WidgetField
from morevistside.windows import BaseWindow

ic.configureOutput(includeContext=True)


class LayoutWindow(BaseWindow):
  """LayoutWindow subclasses BaseWindow and provides the layout management
  for the main application window. This class is responsible for creating
  widgets and layouts in the main application window. It is not responsible
  for connecting any signals and slots to and from the visual elements. """

  baseWidget = WidgetField(QWidget)
  helloWorld = WidgetField(LabelWidget, 'yolo', 128, 64)
  debug = WidgetField(LabelWidget, 'DEBUG', 256, 64)
  button = WidgetField(QPushButton, 'LMAO')
  timePlot = WidgetField(PlotWidget, )
  specPlot = WidgetField(PlotWidget, )

  def __init__(self, *args, **kwargs) -> None:
    BaseWindow.__init__(self, *args, **kwargs)
    self.baseLayout = QGridLayout()
    self._plot = None
    self._data = None

  def initUI(self, ) -> None:
    """Sets up the widgets"""
    self.baseLayout.addWidget(self.helloWorld, 0, 0, 1, 1)
    self.baseLayout.addWidget(self.button, 0, 1, 1, 1)
    self.baseLayout.addWidget(self.timePlot, 1, 0, 1, 2)
    self.baseLayout.addWidget(self.specPlot, 2, 0, 1, 2)
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)

  def show(self) -> None:
    """Implements initUI before show"""
    self.initUI()
    return BaseWindow.show(self)
