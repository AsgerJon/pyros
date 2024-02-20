"""HelpMenu provides the help menu. At first, it contains only About QT"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from morevistside.actionmenus import AbstractMenu, ActionField


class HelpMenu(AbstractMenu):
  """HelpMenu provides the help menu. At first, it contains only About QT"""

  aboutQtAction = ActionField("About &Qt",
                              "aboutQt",
                              "Show the Qt library's about box")
