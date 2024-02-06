"""The actionFactory creates named instances of QAction"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QAction, QKeySequence

from morevistside.actionmenus import getIcon
from morevistutils.waitaminute import typeMsg


def actionFactory(name: str, shortCut: str = None) -> QAction:
  """The actionFactory creates named instances of QAction"""

  icon = getIcon(name, )
  keyCombination = None
  if shortCut is not None:
    if isinstance(shortCut, str):
      keyCombination = QKeySequence(shortCut)
    else:
      e = typeMsg('shortCut', shortCut, str)
      raise TypeError(e)
  action = QAction(icon, name, )
  action.setShortcut(keyCombination)
  return action
