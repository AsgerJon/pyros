"""SubscriberField provides a descriptor class for ros topics."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from vistutils.fields import AbstractField

from morevistutils.fields import QuickField
from pyros import RosField
from rosutils import getTopicType


class SubscriberField(AbstractField):
  """SubscriberField provides a descriptor class for ros topics."""

  topicName = QuickField(str, 'unnamed')

  def __prepare_owner__(self, owner: type) -> type:
    """Prepare the owner for the SubscriberField."""
    if owner is RosField:
      return owner
    raise ValueError

  def _getFieldOwner(self) -> type:
    """Get the owner of the field."""
    if AbstractField._getFieldOwner() is RosField:
      return RosField
    raise ValueError

  def __init__(self, topicName: str, ) -> None:
    AbstractField.__init__(self)
    self.topicName = topicName

  def _createSubscriber(self, owner: type) -> None:
    """Create the subscriber."""
