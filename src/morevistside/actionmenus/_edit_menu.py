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

  def getActions(self) -> list:
    """Getter-function still needing some implementation details"""
    return stringList("""cut, copy, paste, settings""")
