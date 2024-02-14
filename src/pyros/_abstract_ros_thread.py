"""AbstractRosThread subclasses QThread and provides a base class for ROS
threads. It provides a common interface for starting and stopping the ROS
loop. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
from abc import abstractmethod
from typing import Never, Any, Callable
from warnings import warn

from PySide6.QtCore import QThread
from roslib.message import get_message_class
from rospy import init_node, get_published_topics
from rospy.core import is_initialized, logwarn
from rospy.topics import Topic, Subscriber
from vistutils import maybe
from vistutils.fields import Field
from vistutils.waitaminute import typeMsg

from morevistside import parseParent


class AbstractRosThread(QThread):
  """AbstractRosThread subclasses QThread and provides a base class for ROS
threads. It provides a common interface for starting and stopping the ROS
loop. """

  def __new__(cls, *args, **kwargs) -> AbstractRosThread:
    """Constructor for AbstractRosThread"""
    self = super().__new__(cls)
    cls.__threaded_nodes__.append(self)
    return self

  __threaded_nodes__ = []
  __subscribed_topics__ = {}
  __default_node_name__ = 'Test'
  __default_master_uri__ = 'http://localhost:11311'

  nodeName = Field()
  uri = Field()
  initialized = Field()

  @nodeName.GET
  def getNodeName(self) -> str:
    """Getter-function for node name"""
    if hasattr(self, '__node_name__'):
      nodeName = getattr(self, '__node_name__')
      if isinstance(nodeName, str):
        return nodeName
      e = typeMsg('nodeName', nodeName, str)
      raise TypeError(e)
    return self.__default_node_name__

  @nodeName.SET
  def setNodeName(self, nodeName: str) -> None:
    """Setter-function for node name"""
    if isinstance(nodeName, str):
      setattr(self, '__node_name__', nodeName)
    else:
      e = typeMsg('nodeName', nodeName, str)
      raise TypeError(e)

  @uri.GET
  def getURI(self) -> str:
    """Getter-function for the ROS MASTER URI. Please note that the first
    initialization of ROS will be using an URI returned by this method. """
    if hasattr(self, '__ros_master_uri__'):
      uri = getattr(self, '__ros_master_uri__')
      if isinstance(uri, str):
        return uri
      e = typeMsg('uri', uri, str)
      raise TypeError(e)
    return self.__default_master_uri__

  @uri.SET
  def setURI(self, uri: str) -> None:
    """Setter-function for the ROS MASTER URI. """
    if isinstance(uri, str):
      setattr(self, '__ros_master_uri__', uri)
    else:
      e = typeMsg('uri', uri, str)
      raise TypeError(e)

  @initialized.GET
  def getInitialized(self) -> bool:
    """Getter-function for the initialized flag"""
    return is_initialized()

  @initialized.SET
  def setInitialized(self) -> Never:
    """Illegal setter function"""
    e = """The initialized flag is read-only. """
    raise TypeError(e)

  @staticmethod
  def getNamedTopicMessageType(topicName: str) -> type:
    """Locates the topic type of the given name"""
    topicType = None
    topics = get_published_topics()
    for topic, type_ in topics:
      if topic == topicName:
        topicType = type_
        break
    else:
      raise NameError(topicName)
    return get_message_class(topicType)

  def __init__(self, nodeName: str = None, *args, **kwargs) -> None:
    parentArg = parseParent(*args)
    parentKwarg = kwargs.get('parent', None)
    parent = maybe(parentKwarg, parentArg)
    QThread.__init__(self, parent)
    if nodeName is not None:
      self.nodeName = nodeName
    uri = kwargs.get('uri', None)
    if uri is not None:
      self.uri = uri

  def initializeNode(self, **kwargs) -> None:
    """Initializes the ROS node"""
    if not is_initialized():
      if kwargs.get('_recursion', False):
        raise RecursionError
      if 'uri' in kwargs:
        uri = kwargs.get('uri')
        if isinstance(uri, str):
          os.environ['ROS_MASTER_URI'] = uri
      else:
        os.environ['ROS_MASTER_URI'] = self.uri
      init_node(self.nodeName, anonymous=False, )
      return self.initializeNode(_recursion=True)
    val = kwargs.get('_recursion', None)
    if val is None:
      warn("""The ROS node is already initialized from elsewhere!""")

  def createSubscriber(self, topicName: str, **kwargs) -> Subscriber:
    """Creates a subscriber"""
    if not is_initialized():
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.initializeNode()
      return self.createSubscriber(topicName, _recursion=True)
    if topicName in self.__subscribed_topics__:
      e = """Subscriber already exists for topic: '%s'!""" % topicName
      raise AttributeError(e)
    msgType = self.getNamedTopicMessageType(topicName)
    callback = self.getCallback(topicName)
    subscriber = Subscriber(topicName, msgType, callback)
    self.__subscribed_topics__[topicName] = subscriber
    return subscriber

  def getSubscriber(self, topicName, **kwargs) -> Subscriber:
    """Getter-function for the subscriber"""
    if topicName in self.__subscribed_topics__:
      return self.__subscribed_topics__[topicName]
    if kwargs.get('_recursion', False):
      raise RecursionError
    self.createSubscriber(topicName)
    return self.getSubscriber(topicName, _recursion=True)

  @abstractmethod
  def getCallback(self, topicName: str, ) -> Callable:
    """Getter-function for the callback. Subclasses must implement this."""

  @abstractmethod
  def run(self) -> None:
    """Subclasses must implement this method"""
