"""WrapMsg provides a wrapping class on existing custom msg types."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations


class WrapMsg:
  """WrapMsg provides a wrapping class on existing custom msg types."""

  def __init__(self, *args, **kwargs) -> None:
    pass

  def _setMsgType(self, msgType: type) -> None:
    """Sets the msg type"""
