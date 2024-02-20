"""DebugMenu provides a menu of customizable actions"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from morevistside.actionmenus import AbstractMenu, ActionField


class DebugMenu(AbstractMenu):
  """DebugMenu provides a menu of customizable actions"""

  debugAction1 = ActionField("Debug Action 1", 'F1', 'DEBUG')
  debugAction2 = ActionField("Debug Action 2", 'F2', 'DEBUG')
  debugAction3 = ActionField("Debug Action 3", 'F3', 'DEBUG')
  debugAction4 = ActionField("Debug Action 4", 'F4', 'DEBUG')
  debugAction5 = ActionField("Debug Action 5", 'F5', 'DEBUG')
  debugAction6 = ActionField("Debug Action 6", 'F6', 'DEBUG')
  debugAction7 = ActionField("Debug Action 7", 'F7', 'DEBUG')
  debugAction8 = ActionField("Debug Action 8", 'F8', 'DEBUG')
  debugAction9 = ActionField("Debug Action 9", 'F9', 'DEBUG')
  debugAction10 = ActionField("Debug Action 10", 'F10', 'DEBUG')
  debugAction11 = ActionField("Debug Action 11", 'F11', 'DEBUG')
  debugAction12 = ActionField("Debug Action 12", 'F12', 'DEBUG')
