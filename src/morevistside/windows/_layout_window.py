"""LayoutWindow subclasses BaseWindow and provides the layout management
for the main application window. This class is responsible for creating
widgets and layouts in the main application window. It is not responsible
for connecting any signals and slots to and from the visual elements. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QWidget, QGridLayout
from icecream import ic
from vistutils.fields import Field

from morevistside.widgets import WidgetField, LabelWidget, PlotWidget
from morevistside.windows import BaseWindow
from morevistutils import DataArray
from morevistutils.waitaminute import typeMsg

ic.configureOutput(includeContext=True)


class LayoutWindow(BaseWindow):
  """LayoutWindow subclasses BaseWindow and provides the layout management
  for the main application window. This class is responsible for creating
  widgets and layouts in the main application window. It is not responsible
  for connecting any signals and slots to and from the visual elements. """

  helloWorld = WidgetField(LabelWidget, 'yolo', 128, 64)
  debug = WidgetField(LabelWidget, 'DEBUG', 256, 64)
  plot = Field()

  def _createPlot(self, ) -> None:
    """Creator-function for plot widget"""
    self._plot = PlotWidget(self)

  @plot.GET
  def _getPlot(self, **kwargs) -> PlotWidget:
    """Getter-function for plot widget"""
    if self._plot is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createPlot()
      return self._getPlot(_recursion=True)
    if isinstance(self._plot, PlotWidget):
      return self._plot
    e = typeMsg('_plot', self._plot, PlotWidget)
    raise TypeError(e)

  def __init__(self, *args, **kwargs) -> None:
    BaseWindow.__init__(self, *args, **kwargs)
    self.baseWidget = QWidget()
    self.baseLayout = QGridLayout()
    self._plot = None
    self._data = None

  def initUI(self, ) -> None:
    """Sets up the widgets"""
    self.baseLayout.addWidget(self.helloWorld, 0, 0, 1, 1)
    self.baseLayout.addWidget(self.plot, 1, 0, 1, 2)
    self.baseLayout.addWidget(self.debug, 2, 1, 1, 1)
    self.baseWidget.setLayout(self.baseLayout)
    self.setCentralWidget(self.baseWidget)
