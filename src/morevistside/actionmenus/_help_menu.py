"""HelpMenu provides the help menu. At first, it contains only About QT"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from vistutils import stringList
from PySide6.QtGui import QAction

from morevistside.actionmenus import AbstractMenu


class HelpMenu(AbstractMenu):
  """HelpMenu provides the help menu. At first, it contains only About QT"""

  __action_names__ = stringList("""about_qt""")
  __action_keys__ = stringList("""F12""")
  __action_text__ = stringList("""About Qt""")

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
