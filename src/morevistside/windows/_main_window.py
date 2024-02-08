"""MainWindow subclasses LayoutWindow and provides the business logic."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time
from typing import Any

import rospy
from PySide6.QtCore import QTimer, Qt
from rospy import Subscriber
from std_msgs.msg import Float64

from morevistside.windows import LayoutWindow


class MainWindow(LayoutWindow):
  """MainWindow subclasses LayoutWindow and provides the business logic."""

  def __init__(self, *args, **kwargs) -> None:
    LayoutWindow.__init__(self, *args, **kwargs)
    self._paintTimer = None
    self._subscriber = None
    self._startTime = None

  def _createPaintTimer(self, ) -> None:
    """Creates the update timer"""
    self._paintTimer = QTimer()
    self._paintTimer.setInterval(1000)
    self._paintTimer.setTimerType(Qt.TimerType.VeryCoarseTimer)
    self._paintTimer.setSingleShot(True)
    self._paintTimer.timeout.connect(self.timedPaint)

  def callback(self, data: Any) -> None:
    """Callback function used by the Subscriber. This method should not
    cause any paint related events!"""

  def timedPaint(self) -> None:
    """Handles timeout event"""
    self.update()
    if rospy.is_shutdown():
      return
    self._paintTimer.start()

  def _createSubscriber(self) -> None:
    """Creator function for publisher"""
    if self._subscriber is not None:
      raise ValueError
    rospy.init_node('Client', anonymous=False)
    self._subscriber = Subscriber(
      '/tool/pump_current', Float64, self.callback, )
