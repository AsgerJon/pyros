"""The FilesMenu subclasses AbstractMenu."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from vistutils import stringList

from morevistside.actionmenus import AbstractMenu, actionFactory


class FilesMenu(AbstractMenu):
  """The FilesMenu subclasses AbstractMenu."""

  __action_names__ = stringList("""new, open, save, saveAs, exit""")
  __action_keys__ = stringList("""CTRL+N, CTRL+O, CTRL+S, CTRL+SHIFT+S, 
  ALT+F4""")

  @classmethod
  def _getNames(cls) -> list:
    """Getter-function for list of names"""
    return cls.__action_names__

  @classmethod
  def _getKeys(cls) -> list:
    """Getter-function for list of keyboard shortcuts"""
    return cls.__action_keys__

  def __init__(self, *args, **kwargs) -> None:
    AbstractMenu.__init__(self, *args, **kwargs)
    self.__menu_actions__ = []
    for (name, keys) in zip(self._getNames(), self._getKeys()):
      self.__menu_actions__.append(actionFactory(name, self, keys))

  def getActions(self) -> list:
    """Getter-function for actions in this menu"""
    return self.__menu_actions__
