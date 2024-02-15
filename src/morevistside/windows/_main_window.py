"""MainWindow subclasses LayoutWindow and provides the business logic."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import rospy
from PySide6.QtCore import QTimer, Qt, QRect, QMargins
from icecream import ic
from vistutils.fields import Field

from morevistside import ThreadField
from morevistside.windows import LayoutWindow
from morevistutils.waitaminute import typeMsg
from pyros import BaseThread


class MainWindow(LayoutWindow):
  """MainWindow subclasses LayoutWindow and provides the business logic."""

  __paint_timer__ = None

  paintTimer = Field()
  ROS = ThreadField(BaseThread, )

  def __init__(self, *args, **kwargs) -> None:
    LayoutWindow.__init__(self, *args, **kwargs)

  def _createPaintTimer(self, ) -> None:
    """Creates the update timer"""
    self.__paint_timer__ = QTimer()
    self.__paint_timer__.setInterval(20)
    self.__paint_timer__.setTimerType(Qt.TimerType.PreciseTimer)
    self.__paint_timer__.setSingleShot(True)
    self.__paint_timer__.timeout.connect(self.timedPaint)

  @paintTimer.GET
  def _getPaintTimer(self, **kwargs) -> QTimer:
    """Getter-function for paint timer"""
    if self.__paint_timer__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createPaintTimer()
      return self._getPaintTimer(_recursion=True)
    if isinstance(self.__paint_timer__, QTimer):
      return self.__paint_timer__
    e = typeMsg('_paintTimer', self.__paint_timer__, QTimer)
    raise TypeError(e)

  def debugFunc01(self, ) -> None:
    """Start paint timer"""
    ic('debug 01')

  def debugFunc02(self) -> None:
    """Explicitly repaint plot widget"""
    self.paintTimer.start()

  def debugFunc03(self) -> None:
    """Explicitly repaint plot widget"""
    self.timePlot.repaint()

  def timedPaint(self) -> None:
    """Handles timeout event"""
    if rospy.is_shutdown():
      ic('Rospy was shut down')
      return
    self.timePlot.update()
    self.paintTimer.start()

  def initUI(self) -> None:
    """Reimplementation"""
    LayoutWindow.initUI(self)
    self.button.clicked.connect(self.buttonFunc)
    self.ROS.pumpSignal.connect(self.timePlot.receiveValues)
    self.paintTimer.start()
    self.ROS.start()

  def buttonFunc(self) -> None:
    """Called by button press"""

  def debugFunc04(self, ) -> None:
    """Test of rectangles"""
    outer = QRect(0, 0, 100, 100)
    margins = QMargins(10, 10, 10, 10)
    inner = outer.marginsRemoved(margins)
    ic(outer, inner, outer - margins)

  def show(self) -> None:
    """Reimplementation"""
    LayoutWindow.show(self)
    # self.subscriber
