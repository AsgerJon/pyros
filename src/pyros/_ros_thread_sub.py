"""RosThread subclasses QThread providing a separate thread for receiving
data from ROS than the thread used for painting the GUI. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
from typing import Callable, Any

from PySide6.QtCore import QThread, Signal, QObject, Slot
from rospy import init_node, Subscriber, spin
from rospy.core import is_initialized
from rospy.topics import Topic
from std_msgs.msg import Float64
from vistutils import maybe
from vistutils.fields import Field

from morevistutils.fields import TypedField


class RosControlThread(QObject):
  """Control signals for ROS subscriber thread."""
  startSignal = Signal()
  stopSignal = Signal()


class RosThreadSub(QThread):
  """RosThread subclasses QThread providing a separate thread for receiving
  data from ROS than the thread used for painting the GUI. """

  dataRes = Signal(float)
  pumpCurrentData = Signal(float)

  nodeName = Field()
  uri = Field()

  @Slot()
  def startRos(self) -> None:
    """This slot starts the ros loop"""
    self.__run_ros_run__ = True

  @Slot()
  def stopRos(self) -> None:
    """This slot stops the ros loop"""
    self.__run_ros_run__ = False

  @nodeName.GET
  def getNodeName(self) -> str:
    """Getter-function for node name"""
    defKey = 'PYROS_DEFAULT_NODE_NAME'
    return maybe(self.__node_name__, os.environ.get(defKey, 'LOL'))

  @uri.GET
  def getURI(self) -> str:
    """Getter-function for the ROS MASTER URI. Please note that the first
    initialization of ROS will be using an URI returned by this method. """
    return self.__ros_master_uri__

  @staticmethod
  def getPumpCurrentTopicName() -> str:
    """Getter-function for pump current topic name"""
    return '/pump/current'

  @staticmethod
  def getPumpCurrentTopicClass() -> type:
    """Getter-function for the class of the message"""
    return Float64

  def getPumpCurrentCallback(self, ) -> Callable:
    """Callback for pump current callback"""

    def callback(data: Any) -> None:
      """Callback function created by the factory"""
      self.pumpCurrentData.emit(data.data)

    return callback

  def _createPumpCurrentTopic(self, ) -> None:
    """Creator-function for pump current topic"""
    callback = self.getPumpCurrentCallback()
    topicName = self.getPumpCurrentTopicName()
    topicClass = self.getPumpCurrentTopicClass()
    self.__pump_current__ = Subscriber(topicName, topicClass, callback)

  def _getPumpCurrentTopic(self, **kwargs) -> Topic:
    """Getter-function for the topic named"""
    if self.__pump_current__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createPumpCurrentTopic()
      return self._getPumpCurrentTopic(_recursion=True)

  def __init__(self, *args, **kwargs) -> None:
    self.__run_ros_run__ = False
    self.control = RosControlThread()
    self.control.startSignal.connect(self.startRos)
    self.control.stopSignal.connect(self.stopRos)
    self.__node_name__ = kwargs.get('nodeName', None)
    self.__ros_master_uri__ = None
    self.__pump_current__ = None

  def run(self) -> None:
    """In the parent class, this method executes 'exec' and when it ends
    the method and the thread ends. Here instead is the Ros Subscriber
    loop. """
    if not is_initialized():
      init_node(self.nodeName, anonymous=False)
    self._getPumpCurrentTopic()
    spin()
