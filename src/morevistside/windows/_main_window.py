"""MainWindow subclasses LayoutWindow and provides the business logic."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

import rospy
from PySide6.QtCore import QTimer, Qt
from icecream import ic
from rospy import Subscriber
from std_msgs.msg import Float64
from vistutils.fields import Field

from morevistside.windows import LayoutWindow
from morevistutils import DataArray
from morevistutils.waitaminute import typeMsg


class MainWindow(LayoutWindow):
  """MainWindow subclasses LayoutWindow and provides the business logic."""

  paintTimer = Field()
  subscriber = Field()
  data = Field()

  def _createData(self) -> None:
    """Creator function for data array"""
    self._data = DataArray()

  @data.GET
  def _getData(self, **kwargs) -> DataArray:
    """Getter-function for data array"""
    if self._data is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createData()
      return self._getData(_recursion=True)
    if isinstance(self._data, DataArray):
      return self._data
    e = typeMsg('_data', self._data, DataArray)
    raise TypeError(e)

  def __init__(self, *args, **kwargs) -> None:
    LayoutWindow.__init__(self, *args, **kwargs)
    self._paintTimer = None
    self._subscriber = None
    self._startTime = None

  def _createPaintTimer(self, ) -> None:
    """Creates the update timer"""
    self._paintTimer = QTimer()
    self._paintTimer.setInterval(33)
    self._paintTimer.setTimerType(Qt.TimerType.VeryCoarseTimer)
    self._paintTimer.setSingleShot(True)
    self._paintTimer.timeout.connect(self.timedPaint)

  @paintTimer.GET
  def _getPaintTimer(self, **kwargs) -> QTimer:
    """Getter-function for paint timer"""
    if self._paintTimer is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createPaintTimer()
      return self._getPaintTimer(_recursion=True)
    if isinstance(self._paintTimer, QTimer):
      return self._paintTimer
    e = typeMsg('_paintTimer', self._paintTimer, QTimer)
    raise TypeError(e)

  def debugFunc01(self, ) -> None:
    """Start paint timer"""
    ic('debug 01')
    print(self.subscriber)
    self.paintTimer.start()

  def debugFunc02(self) -> None:
    """Explicitly repaint plot widget"""
    self.plot.repaint()
    ic('debug 02')

  def timedPaint(self) -> None:
    """Handles timeout event"""
    self.plot.update()
    if rospy.is_shutdown():
      ic('rospy was shutdown, L')
      return
    self.paintTimer.start()

  def _createSubscriber(self) -> None:
    """Creator function for publisher"""
    if self._subscriber is not None:
      raise ValueError
    rospy.init_node('Client', anonymous=False)
    self._subscriber = Subscriber(
      '/tool/pump_current', Float64, self.subscriberCallback, )

  @subscriber.GET
  def _getSubscriber(self, **kwargs) -> Subscriber:
    """Getter-function for subscriber"""
    if self._subscriber is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createSubscriber()
      return self._getSubscriber(_recursion=True)
    if isinstance(self._subscriber, Subscriber):
      return self._subscriber
    e = typeMsg('_subscriber', self._subscriber, Subscriber)
    raise TypeError(e)

  def subscriberCallback(self, data: Any) -> None:
    """This method wraps the callback from the subscriber. Must be kept
    separate from the paint events."""
    value = data.data
    self.data.callback(value)
