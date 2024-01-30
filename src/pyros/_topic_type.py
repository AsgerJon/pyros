"""TopicType is a dataclass representing a topic by name and dataclass"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from morevistutils.fields import TypedField


class TopicType:
  """TopicType is a dataclass representing a topic by name and dataclass"""

  name = TypedField(str, 'topicName')
  data = TypedField(str, 'bool')

  def __init__(self, *args, **kwargs) -> None:
    pass
