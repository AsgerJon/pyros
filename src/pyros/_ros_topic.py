"""RosTopic provides an abstraction for ros topics."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from genpy import Message
from vistutils.fields import Field

from morevistutils.fields import TypedField
from pyros import MetaRos


class RosTopic(metaclass=MetaRos):
  """RosTopic provides an abstraction for ros topics."""
  topicName = TypedField(str, 'topic_test')
  dataClass = Field()

  @dataClass.GET
  def _getDataClass(self) -> Message:
    """Getter-function for data class"""

  def __init__(self, *args, **kwargs) -> None:
    pass
