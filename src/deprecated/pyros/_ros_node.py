"""RosNode encapsulates the logic relating to nodes in the ROS framework."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os

import rospy
from rospy import init_node
from vistutils import maybe
from vistutils.fields import Field
from vistutils.waitaminute import typeMsg

from morevistutils.fields import TypedField
from pyros import MetaRos


class RosNode(metaclass=MetaRos):
  """RosNode encapsulates the logic relating to nodes in the ROS
  framework."""
  nodeName = TypedField(str, 'Test')
  spinTime = Field()

  @classmethod
  @spinTime.GET
  def _getSpinTime(cls, *args, **kwargs) -> int:
    """Getter-function for spin time. This is the amount of time measured
    in ms that the spinning method waits between each state check."""
    envTime = os.environ.get('ROS_SPIN_TIME', None)
    fallBackTime = cls.getFallbackSpinTime()
    out = maybe(envTime, fallBackTime)
    if isinstance(out, int):
      return out
    e = typeMsg('spinTime', out, int)
    raise TypeError(e)

  def __init__(self, name: str, *args, **kwargs) -> None:
    self.nodeName = name

  def _initializeNode(self) -> None:
    """This method invokes the underlying 'init_node' assuming that the
    '__init__ ' method has populated the necessary values"""
    init_node(self.nodeName)

  def spin(self, **kwargs) -> None:
    """This method keeps the node running"""
    rospy.rostime.wallsleep(self.spinTime / 1000)
