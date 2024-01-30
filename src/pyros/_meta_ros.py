"""MetaRos provides a common metaclass shared by classes wrapping the ros
framework. This metaclass is certain to invoke its __prepare__ method
before any of its derived classes can be created. Thus, it allows for a
convenient hook ensuring initialization."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from rospy import MasterProxy, get_published_topics
from vistutils.fields import Field
from vistutils.metas import AbstractMetaclass, Bases
from vistutils.metas import BaseNamespace as BNS

from morevistutils import applyEnv
from morevistutils.fields import SpecialField, EnvField
from pyros import RosMaster, TopicType


class MetaRos(AbstractMetaclass):
  """MetaRos provides a common metaclass shared by classes wrapping the ros
  framework. This metaclass is certain to invoke its __prepare__ method
  before any of its derived classes can be created. Thus, it allows for a
  convenient hook ensuring initialization."""
  URI = EnvField('ROS_MASTER_URI')
  master = SpecialField(RosMaster)
  topics = Field()

  @master.CREATE
  def _createRosMaster(cls, *args, **kwargs) -> MasterProxy:
    """Creator function for ros master"""
    return RosMaster(cls.URI)

  @topics.GET
  def _getTopics(cls) -> list:
    """Getter-function for published topics"""
    data = get_published_topics()
    out = []
    for item in data:
      topicName = item[0]
      dataName = item[1]
      if isinstance(topicName, str) and isinstance(dataName, str):
        out.append(TopicType(topicName, dataName))
    return out

  @classmethod
  def __prepare__(mcls, name: str, bases: Bases, **kwargs) -> BNS:
    applyEnv()
    if mcls.master is None:
      raise RuntimeError
    namespace = BNS(name, bases, **kwargs)
    namespace.setMetaclass(mcls)
    return namespace
