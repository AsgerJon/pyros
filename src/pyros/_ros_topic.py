"""RosTopic encapsulates the logic relating to topics in the ROS
framework."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time

# import rospy
# from rospy import Publisher, Rate, is_shutdown

from pyros import RosDispatcher
from std_msgs___.msg import String


@RosDispatcher()
class Talker:
  """RosTopic encapsulates the logic relating to topics in the ROS
  framework."""

  def __init__(self, nodeName: str, **kwargs) -> None:
    self._nodeName = nodeName
    self._chatter = Publisher('chatter', String, queue_size=10)
    self._rate = Rate(1)
    self.__start_time__ = None
    self.__time_limit__ = kwargs.get('timeLimit', 30)

  def begin(self) -> None:
    """Starts up the chatting"""
    if self.__start_time__ is None:
      self.__start_time__ = time.time()
    elif time.time() - self.__start_time__ > self.__time_limit__:
      return
    message = 'Hello world!'
    self._chatter.publish(message)
    self._rate.sleep()
    if is_shutdown():
      return
    return self.begin()
