"""RosField provides a descriptor class for ros topics. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Callable

from rospy import Subscriber
from vistutils import monoSpace
from vistutils.fields import AbstractField, Field

from morevistutils.waitaminute import typeMsg
from pyros import AbstractRosThread


class RosField(AbstractField):
  """RosField provides a descriptor class for ros topics."""

  __topic_name__ = None
  __topic_subscriber__ = None
  __topic_publisher__ = None
  __topic_callback__ = None

  topicName = Field()
  subscriber = Field()
  publisher = Field()

  def getSubscriberCallback(self, ) -> Callable:
    """Callback for the subscriber"""
    callback = self.__topic_callback__
    if callback is None:
      e = """No callback set!"""
      raise AttributeError(e)
    callbackName = callback.__name__
    owner = self._getFieldOwner()
    if not hasattr(owner, callbackName):
      raise AttributeError(callbackName)
    if getattr(owner, callbackName) is callback:
      return callback
    e = """Failed to confirm identity of callback function at name: '%s'!"""
    raise AttributeError(monoSpace(e % callbackName))

  def _createSubscriber(self, instance: Any) -> None:
    """Creator-function for the subscriber"""
    name = self.topicName
    msgType = AbstractRosThread.getNamedTopicMessageType(name)
    callback = self.getSubscriberCallback()

  @subscriber.GET
  def _getSubscriber(self, instance: Any, **kwargs) -> Subscriber:
    """Getter-function for the subscriber"""
    if self.__topic_subscriber__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createSubscriber(instance)
      return self._getSubscriber(instance, _recursion=True)
    if isinstance(self.__topic_subscriber__, Subscriber):
      return self.__topic_subscriber__
    e = typeMsg('subscriber', self.__topic_subscriber__, Subscriber)
    raise TypeError(e)

  def __prepare_owner__(self, owner: type) -> type:
    """Ensures that at the owning class has an attribute called:
      __topic_fields__ listing the instances of this class owned by the
      owner. """
    if not issubclass(owner, AbstractRosThread):
      e = """Expected owner to be a subclass of AbstractRosThread!"""
      raise TypeError(e)
    existingFields = getattr(owner, '__topic_fields__', [])
    setattr(owner, '__topic_fields__', (*existingFields, self))
    return owner

  def __init__(self, topicName: str, *args, **kwargs) -> None:
    AbstractField.__init__(self)
    self.__topic_name__ = topicName

  def __get__(self, instance: Any, owner: type) -> Any:
    """Getter-function for the descriptor"""
    subscriber = self._getSubscriber(instance)

  def CALLBACK(self, callMeMaybe: Callable) -> Callable:
    """Decorator setting the callback for the subscriber"""
    if self.__topic_callback__ is not None:
      raise AttributeError('Callback already set!')
    if not callable(callMeMaybe):
      e = typeMsg('callMeMaybe', callMeMaybe, Callable)
      raise TypeError(e)
    self.__topic_callback__ = callMeMaybe
    return callMeMaybe
