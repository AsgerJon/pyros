"""EditMenu provides the edit menu with actions such as cut, copy and
paste"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from morevistside.actionmenus import AbstractMenu, ActionField


class EditMenu(AbstractMenu):
  """EditMenu provides the edit menu with actions such as cut, copy and
  paste"""

  cutAction = ActionField('Cut', 'Ctrl+X', 'Cut the selected text')
  copyAction = ActionField('Copy', 'Ctrl+C', 'Copy the selected text')
  pasteAction = ActionField('Paste', 'Ctrl+V', 'Paste the selected text')
  selectAllAction = ActionField('Select All', 'Ctrl+A', 'Select all text')
  undoAction = ActionField('Undo', 'Ctrl+Z', 'Undo the last action')
  redoAction = ActionField('Redo', 'Ctrl+Y', 'Redo the last action')
