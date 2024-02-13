"""MainWindow subclasses LayoutWindow and provides the business logic."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from typing import Any

import rospy
from PySide6.QtCore import QTimer, Qt, Slot
from icecream import ic
from rospy import Subscriber, Publisher
from std_msgs.msg import Float64, String
from vistutils.fields import Field

from morevistside.windows import LayoutWindow
from morevistutils import DataArray
from morevistutils.waitaminute import typeMsg
from yolomsg import AuxCommand


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
    self._publisher = None
    self._startTime = None
    rospy.init_node('Client', anonymous=False)

  def _createPublisher(self) -> None:
    """Creator function for publisher"""
    topicName = 'lmao'
    msgType = Float64
    self._publisher = Publisher(topicName, String, queue_size=1)

  def _getPublisher(self, **kwargs) -> Publisher:
    """Getter function for publisher"""
    if self._publisher is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createPublisher()
      return self._getPublisher(_recursion=True)
    if isinstance(self._publisher, Publisher):
      return self._publisher
    e = typeMsg('_publisher', self._publisher, Publisher)
    raise TypeError(e)

  @Slot()
  def publishMessage(self, ) -> None:
    """Publishes the message"""
    self._getPublisher().publish(time.ctime())

  def _createPaintTimer(self, ) -> None:
    """Creates the update timer"""
    self._paintTimer = QTimer()
    self._paintTimer.setInterval(20)
    self._paintTimer.setTimerType(Qt.TimerType.PreciseTimer)
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

  def debugFunc02(self) -> None:
    """Explicitly repaint plot widget"""
    self.paintTimer.start()

  def debugFunc03(self) -> None:
    """Explicitly repaint plot widget"""
    self.plot.repaint()

  def timedPaint(self) -> None:
    """Handles timeout event"""
    if rospy.is_shutdown():
      ic('Rospy was shut down')
      return
    self.plot.update()
    self.paintTimer.start()

  def _createSubscriber(self) -> None:
    """Creator function for publisher"""
    if self._subscriber is not None:
      raise ValueError

    self._subscriber = Subscriber(
      '/tool/pump_current', Float64, self.subscriberCallback, queue_size=1)

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

  def initUI(self) -> None:
    """Reimplementation"""
    LayoutWindow.initUI(self)
    self.button.clicked.connect(self.buttonFunc)

  def buttonFunc(self) -> None:
    """Called by button press"""
    self.publishMessage()
    ic('button triggered')
