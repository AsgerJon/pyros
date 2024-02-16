"""DebugMenu provides a menu of customizable actions"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from morevistside.actionmenus import AbstractMenu


class DebugMenu(AbstractMenu):
  """DebugMenu provides a menu of customizable actions"""

  __action_names__ = ['debug%02d' % i for i in range(13) if i]
  __action_keys__ = ['F%d' % i for i in range(13) if i]
  __action_text__ = ['--<DEBUG>-- %02d' % i for i in range(13) if i]

  @classmethod
  def getNames(cls) -> list:
    """Getter-function for list of names"""
    return cls.__action_names__

  @classmethod
  def getShortcuts(cls) -> list:
    """Getter-function for list of keyboard shortcuts"""
    return cls.__action_keys__

  @classmethod
  def getText(cls) -> list[str]:
    """Getter-function for the list of descriptions"""
    return cls.__action_text__

  def __init__(self, *args, **kwargs) -> None:
    AbstractMenu.__init__(self, *args, icon='debug', **kwargs)
