"""The MenuBar class subclasses QMenuBar and organizes the menus typically
found at the top of the main application window."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QMenuBar

from morevistside import parseParent
from morevistutils.fields import SpecialField
from morevistside.actionmenus import FilesMenu, EditMenu, HelpMenu


class MenuBar(QMenuBar):
  """The MenuBar class subclasses QMenuBar and organizes the menus typically
  found at the top of the main application window."""

  files = SpecialField(FilesMenu)
  edit = SpecialField(EditMenu)
  help = SpecialField(HelpMenu)

  @files.CREATE
  def _createFilesMenu(self, *args, **kwargs) -> FilesMenu:
    return FilesMenu(self, )

  @edit.CREATE
  def _createEditMenu(self, *args, **kwargs) -> EditMenu:
    return EditMenu(self, )

  @help.CREATE
  def _createHelpMenu(self, *args, **kwargs) -> HelpMenu:
    return HelpMenu(self, )

  def __init__(self, *args, **kwargs) -> None:
    parent = parseParent(*args)
    QMenuBar.__init__(self, parent)
    self.addMenu(self.files)
    self.addMenu(self.edit)
    self.addMenu(self.help)
