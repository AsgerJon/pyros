"""RosTopic encapsulates the logic relating to topics in the ROS
framework."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import time
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
  from rospy import Publisher, Rate, init_node
from vistutils import maybe

from pyros import RosDispatcher


@RosDispatcher()
class Talker:
  """RosTopic encapsulates the logic relating to topics in the ROS
  framework."""

  __node_name_fallback__ = 'test_node'
  __topic_name_fallback__ = 'test_topic'

  @staticmethod
  def _parseKwargs(**kwargs) -> dict[str, Optional[str]]:
    """Parser for keyword arguments"""
    nodeNameKwarg = kwargs.get('nodeName', None)
    topicNameKwarg = kwargs.get('topicName', None)
    return dict(nodeName=nodeNameKwarg, topicName=topicNameKwarg)

  @staticmethod
  def _parseArgs(*args) -> dict[str, Optional[str]]:
    """Parser for positional arguments"""
    nodeNameArg, topicNameArg = None, None
    for arg in args:
      if isinstance(arg, bytes):
        arg = arg.decode('utf-8')
      if isinstance(arg, str):
        if nodeNameArg is None:
          nodeNameArg = arg
        elif topicNameArg is None:
          topicNameArg = arg
    return dict(nodeName=nodeNameArg, topicName=topicNameArg)

  @classmethod
  def _defaultValues(cls) -> dict[str, Optional[str]]:
    """Collects the default values."""
    nodeNameFallback = cls.__node_name_fallback__
    topicNameFallback = cls.__topic_name_fallback__
    nodeNameDefault = os.environ.get('NODE_NAME', nodeNameFallback)
    topicNameDefault = os.environ.get('TOPIC_NAME', topicNameFallback)
    return dict(nodeName=nodeNameDefault, topicName=topicNameDefault)

  @classmethod
  def _parse(cls, *args, **kwargs) -> dict[str, Optional[str]]:
    """Parses arguments"""
    parsedKwargs = cls._parseKwargs(**kwargs)
    parsedArgs = cls._parseArgs(*args)
    parsedDefaults = cls._defaultValues()
    nodeNameKwarg = parsedKwargs.get('nodeName', None)
    topicNameKwarg = parsedKwargs.get('topicName', None)
    nodeNameArg = parsedArgs.get('nodeName', None)
    topicNameArg = parsedArgs.get('topicName', None)
    nodeNameDefault = parsedDefaults.get('nodeName', None)
    topicNameDefault = parsedDefaults.get('topicName', None)
    nodeName = maybe(nodeNameKwarg, nodeNameArg, nodeNameDefault)
    topicName = maybe(topicNameKwarg, topicNameArg, topicNameDefault)
    return dict(nodeName=nodeName, topicName=topicName)

  def __init__(self, *args, **kwargs) -> None:
    parsed = self._parse(*args, **kwargs)
    self._nodeName = parsed.get('nodeName', )
    self._topicName = parsed.get('topicName', )
    self._pub = Publisher(self._topicName, String, queue_size=10)
    self._rate = Rate(1)
    self.__start_time__ = None
    self.__time_limit__ = kwargs.get('timeLimit', 30)

  def _createPublisher(self, ) -> None:
    """Creator-function for the rospy publisher"""
    init_node(self._nodeName, )
    self._pub = Publisher(self._nodeName, )

  def _getPublisher(self, **kwargs) -> Publisher:
    """Getter-function for the rospy publisher"""
    if self._pub is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createPublisher()
      return self._getPublisher(_recursion=True)

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
