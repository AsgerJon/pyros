"""The FilesMenu subclasses AbstractMenu."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QMainWindow

from morevistside.actionmenus import AbstractMenu, ActionField


class FilesMenu(AbstractMenu):
  """The FilesMenu subclasses AbstractMenu."""

  newAction = ActionField('New', 'Ctrl+N', 'Create a new file')
  openAction = ActionField('Open', 'Ctrl+O', 'Open a file')
  saveAction = ActionField('Save', 'Ctrl+S', 'Save the file')
  saveAsAction = ActionField('Save As',
                             'Ctrl+Shift+S',
                             'Save the file as...')
  exitAction = ActionField('Exit', 'ATL+F4', 'EXIT')

  def __init__(self, mainWindow: QMainWindow) -> None:
    AbstractMenu.__init__(self, mainWindow)
