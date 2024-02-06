"""The FilesMenu subclasses AbstractMenu."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from vistutils import stringList

from morevistside.actionmenus import AbstractMenu


class FilesMenu(AbstractMenu):
  """The FilesMenu subclasses AbstractMenu."""

  __action_names__ = stringList("""new, open, save, save_as, exit""")
  __action_keys__ = stringList("""CTRL+N, CTRL+O, CTRL+S, CTRL+SHIFT+S, 
  ALT+F4""")
  __action_text__ = stringList("""New, Open, Save, Save As, EXIT""")

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
    AbstractMenu.__init__(self, *args, **kwargs)
