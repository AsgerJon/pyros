"""HelpMenu provides the help menu. At first, it contains only About QT"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QAction

from morevistside.actionmenus import AbstractMenu


class HelpMenu(AbstractMenu):
  """HelpMenu provides the help menu. At first, it contains only About QT"""

  def getActions(self) -> list[QAction]:
    """Getter-function for the ordered list of actions in this menu."""
    return [QAction('About Qt', )]
