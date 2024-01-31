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

  @classmethod
  @files.CREATE
  def _createFilesMenu(cls, *args, **kwargs) -> FilesMenu:
    return FilesMenu(cls, )

  @classmethod
  @edit.CREATE
  def _createEditMenu(cls, *args, **kwargs) -> EditMenu:
    return EditMenu(cls, )

  @classmethod
  @help.CREATE
  def _createHelpMenu(cls, *args, **kwargs) -> HelpMenu:
    return HelpMenu(cls, )

  def __init__(self, *args, **kwargs) -> None:
    parent = parseParent(*args)
    QMenuBar.__init__(self, parent)
    self.addMenu(self.files)
    self.addMenu(self.edit)
    self.addMenu(self.help)
