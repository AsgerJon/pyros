"""TopicField provides a descriptor class for the ros topics."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Never, Any, Callable

from PySide6.QtCore import QThread
from rospy import Subscriber
from vistutils import monoSpace
from vistutils.fields import AbstractField, Field
from vistutils.waitaminute import typeMsg


class TopicField(AbstractField):
  """TopicField provides a descriptor class for the ros topics."""

  __topic_name__ = None
  __topic_subscriber__ = None
  __topic_publisher__ = None
  __subscriber_callback__ = None

  ownerThread = Field()
  fieldName = Field()
  topicName = Field()
  subscriber = Field()
  publisher = Field()

  @ownerThread.GET
  def ownerThread(self) -> type:
    """Getter-function for the ownerThread"""
    owner = AbstractField._getFieldOwner(self)
    if issubclass(owner, QThread):
      return owner
    e = """Expected '%s' to be a subclass of '%s'!"""
    raise TypeError(monoSpace(e % (owner, QThread)))

  @ownerThread.SET
  def ownerThread(self, *_) -> Never:
    """Setter-function for the ownerThread"""
    raise AttributeError('Cannot set the owner thread!')

  @fieldName.GET
  def fieldName(self) -> str:
    """Getter-function for the fieldName"""
    name = AbstractField._getFieldName(self)
    if isinstance(name, str):
      return name
    e = typeMsg('fieldName', name, str)
    raise TypeError(e)

  @fieldName.SET
  def fieldName(self, *_) -> Never:
    """Setter-function for the fieldName"""
    raise AttributeError('Cannot set the field name!')

  @topicName.GET
  def topicName(self) -> str:
    """Getter-function for the topicName"""
    return self.__topic_name__

  @topicName.SET
  def topicName(self, name: str) -> None:
    """Setter-function for the topicName"""
    if self.__topic_name__ is None:
      self.__topic_name__ = name
    else:
      raise AttributeError('Cannot set the topic name!')

  def __init__(self, topicName: str, *args, **kwargs) -> None:
    AbstractField.__init__(self, *args, **kwargs)
    self.topicName = topicName

  def __prepare_owner__(self, owner: type) -> type:
    """Implementation of the abstract method"""
    existingTopics = getattr(owner, '__topic_fields__', {})
    existingTopics |= {self.__name__: self}
    setattr(owner, '__topic_fields__', existingTopics)
    return owner

  def __get__(self, instance: Any, owner: QThread) -> TopicField:
    """Implementation of the abstract method"""
    return self

  def __set__(self, instance: Any, message: Any) -> None:
    """Implementation of the abstract method"""
    self.publisher.publish(message)

  def __delete__(self, *_) -> Never:
    """Implementation of the abstract method"""
    raise AttributeError('Cannot delete a topic field!')

  def getCallback(self) -> Callable:
    """Getter-function for the callback"""
    callMeMaybe = self.__subscriber_callback__
    if callable(callMeMaybe):
      return callMeMaybe
    if callMeMaybe is None:
      e = """No callback function has been defined!"""
      raise AttributeError(e)
    e = typeMsg('callback', callMeMaybe, Callable)
    raise TypeError(e)

  def setCallback(self, callMeMaybe: Callable) -> None:
    """Setter-function for the callback"""
    if self.__subscriber_callback__ is not None:
      e = """The callback function has already been defined!"""
      raise AttributeError(e)
    if callable(callMeMaybe):
      self.__subscriber_callback__ = callMeMaybe
    else:
      e = typeMsg('callback', callMeMaybe, Callable)
      raise TypeError(e)

  def CALL(self, callMeMaybe: Callable) -> None:
    """Alias for setCallback. Intended for use as decorator."""
    return self.setCallback(callMeMaybe)

  def createSubscriber(self) -> None:
    """Creates a subscriber"""
    if self.__topic_subscriber__ is not None:
      e = """Subscriber already exists for topic: '%s'!""" % self.topicName
      raise AttributeError(e)
    callback = self.getCallback()
    msgType = self.getNamedTopicMessageType(self.topicName)
    subscriber = Subscriber(self.topicName, msgType, callback)
    self.__topic_subscriber__ = subscriber

  @subscriber.GET
  def subscriber(self, **kwargs) -> Subscriber:
    """Getter-function for the subscriber"""
    if self.__topic_subscriber__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createSubscriber()
      return self.subscriber(_recursion=True)
    if isinstance(self.__topic_subscriber__, Subscriber):
      return self.__topic_subscriber__
    e = typeMsg('subscriber', self.__topic_subscriber__, Subscriber)
    raise TypeError(e)
