"""AbstractRosThread provides a subclass of QThreads allowing ROS-related
threads to run externally to the GUI thread."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QThread, Signal, Slot
from rospy.rostime import wallsleep
from vistutils.fields import Field
from vistutils.waitaminute import typeMsg


class AbstractRosThread(QThread):
  """AbstractRosThread provides a subclass of QThreads allowing ROS-related
  threads to run externally to the GUI thread."""

  __named_topics__ = {}
  __named_fields__ = {}
  __allow_run__ = None
  __node_name__ = None
  nodeName = Field()
  allowRun = Field()

  startedRun = Signal()
  stoppedRun = Signal()

  @allowRun.GET
  def _getAllowRun(self) -> bool:
    """Getter-function for allow run flag"""
    return True if self.__allow_run__ else False

  @Slot()
  def _stopRun(self) -> None:
    """Stop the run"""
    if self.__allow_run__:
      self.stoppedRun.emit()
      self.__allow_run__ = False

  @Slot()
  def _startRun(self) -> None:
    """Start the run"""
    if not self.__allow_run__:
      self.startedRun.emit()
      self.__allow_run__ = True

  @nodeName.GET
  def _getNodeName(self) -> str:
    if self.__node_name__ is None:
      self.__node_name__ = 'Test'
      return self._getNodeName(_recursive=True)
    if isinstance(self.__node_name__, str):
      return self.__node_name__
    e = typeMsg('nodeName', self.__node_name__, str)
    raise TypeError(e)

  @nodeName.SET
  def _setNodeName(self, value: str) -> None:
    """Run-once setter for nodeName"""
    if self.__node_name__ is not None:
      e = typeMsg('nodeName', value, str, )
      raise TypeError(e)
    self.__node_name__ = value

  def __init__(self, *args, **kwargs) -> None:
    parent = None
    nodeName = None
    for arg in args:
      if isinstance(arg, QThread) and parent is None:
        parent = arg
      if isinstance(arg, str) and nodeName is None:
        nodeName = arg
    nodeName = 'Test' if nodeName is None else nodeName
    QThread.__init__(self, parent)

  def _initFields(self) -> None:
    for (name, field) in self.__named_fields__.items():
      field.build()

  def run(self) -> None:
    """Run the thread"""
    self._initFields()
    while True:
      self._startRun()
      wallsleep(0.02)
      if not self.__allow_run__:
        return
