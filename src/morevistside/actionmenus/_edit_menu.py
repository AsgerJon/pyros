"""EditMenu provides the edit menu with actions such as cut, copy and
paste"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QAction
from vistutils import stringList

from morevistside.actionmenus import AbstractMenu


class EditMenu(AbstractMenu):
  """EditMenu provides the edit menu with actions such as cut, copy and
  paste"""

  __action_names__ = stringList("""cut, copy, paste, undo, redo""")
  __action_keys__ = stringList("""CTRL+X, CTRL+C, CTRL+V, CTRL+Z, CTRL+Y""")
  __action_text__ = stringList("""Cut, Copy, Paste, Undo, Redo""")

  @classmethod
  def getNames(cls) -> list[str]:
    """Getter-function for list of names"""
    return cls.__action_names__

  @classmethod
  def getShortcuts(cls) -> list[str]:
    """Getter-function for list of keyboard shortcuts"""
    return cls.__action_keys__

  @classmethod
  def getText(cls) -> list[str]:
    """Getter-function for the list of action descriptions"""
    return cls.__action_text__
