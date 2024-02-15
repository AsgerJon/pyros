"""RosField provides a descriptor class for ros topics. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Callable, Never

from rospy import Subscriber, Publisher
from vistutils import monoSpace
from vistutils.fields import AbstractField, Field

from morevistutils.fields import QuickField
from morevistutils.waitaminute import typeMsg
from pyros import MessageTypeError
from rosutils import subscriberFactory, publisherFactory


class RosField(AbstractField):
  """RosField provides a descriptor class for ros topics."""

  __subscriber_callback__ = None
  __topic_subscriber__ = None
  __topic_publisher__ = None

  topicName = QuickField(str, 'unnamed')
  nodeName = QuickField(str, 'Test')

  subscriber = Field()
  publisher = Field()
  owner = Field()

  @owner.GET
  def _getFieldOwner(self) -> type:
    """Getter-function for the owner."""
    return AbstractField._getFieldOwner(self)

  @owner.DEL
  @owner.SET
  def _setFieldOwner(self, *_) -> Never:
    """Illegal setter/deleter function"""
    e = """The owner of a RosField is immutable!"""
    raise AttributeError(monoSpace(e))

  def __prepare_owner__(self, owner: type) -> type:
    """Prepare the owner for the RosField."""
    existingTopics = getattr(owner, '__named_topics__', {})
    existingFields = getattr(owner, '__named_fields__', {})
    if self.topicName in existingTopics:
      e = """Name: '%s' is already in use!""" % self.topicName
      raise AttributeError(monoSpace(e))
    existingTopics[self.topicName] = self
    existingFields[self._getFieldName()] = self
    setattr(owner, '__named_topics__', existingTopics)
    setattr(owner, '__named_fields__', existingFields)
    if hasattr(owner, 'nodeName'):
      name = getattr(owner, 'nodeName')
      if isinstance(name, str):
        self.nodeName = name
      else:
        e = typeMsg('nodeName', name, str)
        raise TypeError(e)
    else:
      raise AttributeError('nodeName')
    return owner

  def __init__(self, topicName: str, ) -> None:
    AbstractField.__init__(self)
    self.topicName = topicName

  def getCallback(self, ) -> Callable:
    """Getter-function for the callback function"""
    if self.__subscriber_callback__ is None:
      e = """The callback function is not set!"""
      raise MessageTypeError(monoSpace(e))
    if callable(self.__subscriber_callback__):
      return self.__subscriber_callback__
    e = typeMsg('callback', self.__subscriber_callback__, Callable)
    raise TypeError(e)

  def setCallback(self, callMeMaybe: Callable) -> Callable:
    """Setter-function for the callback function"""
    if self.__subscriber_callback__ is not None:
      e = """The callback function is already set!"""
      raise AttributeError(monoSpace(e))
    if not callable(callMeMaybe):
      e = typeMsg('callMeMaybe', callMeMaybe, Callable)
      raise TypeError(e)
    self.__subscriber_callback__ = callMeMaybe
    return callMeMaybe

  def CALL(self, callMeMaybe: Callable) -> Any:
    """Alias for setCallback."""
    return self.setCallback(callMeMaybe)

  def _createSubscriber(self, ) -> None:
    """Create a subscriber for the topic."""
    self.__topic_subscriber__ = subscriberFactory(
      self.topicName, self.getCallback(), self.nodeName)

  @subscriber.GET
  def _getSubscriber(self, **kwargs) -> Subscriber:
    """Getter-function for the subscriber."""
    if self.__topic_subscriber__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createSubscriber()
      return self._getSubscriber(_recursion=True)
    if isinstance(self.__topic_subscriber__, Subscriber):
      return self.__topic_subscriber__
    e = typeMsg('subscriber', self.__topic_subscriber__, Subscriber)
    raise TypeError(e)

  def _createPublisher(self, ) -> None:
    """Create a publisher for the topic."""
    publisher = publisherFactory(self.topicName, self.nodeName)
    self.__topic_publisher__ = publisher

  @publisher.GET
  def _getPublisher(self, **kwargs) -> Publisher:
    """Getter-function for the publisher."""
    if self.__topic_publisher__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createPublisher()
      return self._getPublisher(_recursion=True)
    if isinstance(self.__topic_publisher__, Publisher):
      return self.__topic_publisher__
    e = typeMsg('publisher', self.__topic_publisher__, Publisher)
    raise TypeError(e)

  def build(self) -> bool:
    """Build the subscriber and publisher."""
    self._createSubscriber()
    self._createPublisher()
    return True
