"""The actionFactory function creates a named action including icon"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtGui import QAction, QKeySequence, QPixmap, QIcon

from morevistside import parseParent
from morevistside.actionmenus import getFids


def _parseIcon(*args) -> tuple:
  """Parses icon"""
  names = getFids()
  for arg in args:
    if isinstance(arg, QPixmap):
      return QIcon(arg)
    if isinstance(arg, QIcon):
      return arg
    if arg in names:
      return arg, QIcon(QPixmap(names.get(arg)))
  return None, None


def _parseShortCut(*args) -> tuple:
  """Parses keyboard shortcut"""
  for arg in args:
    if isinstance(arg, QKeySequence):
      return arg
    if isinstance(arg, str):
      test = QKeySequence(arg)
      if test.count() and test.toString():
        return arg, test
  return None, None


def actionFactory(*args) -> QAction:
  """The actionFactory function creates a named action including icon"""
  parent = parseParent(*args)
  strArgs = [arg for arg in args if isinstance(arg, str)]
  iconArg, icon = _parseIcon(*strArgs)
  keyArg, shortCut = _parseIcon
  text = iconArg
  for arg in strArgs:
    if arg not in [iconArg, keyArg]:
      text = arg
  action = QAction(icon, text, parent)
  if shortCut is not None:
    action.setShortcut(shortCut)
  return action
