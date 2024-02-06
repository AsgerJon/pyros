"""The MenuBar class subclasses QMenuBar and organizes the menus typically
found at the top of the main application window."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QMenuBar
from icecream import ic
from vistutils import stringList

from morevistside import parseParent
from morevistside.actionmenus import FilesMenu, EditMenu, HelpMenu

ic.configureOutput(includeContext=True)


class MenuBar(QMenuBar):
  """The MenuBar class subclasses QMenuBar and organizes the menus typically
  found at the top of the main application window."""

  __menu_titles__ = stringList("""Files, Edit, Help""")
  __menu_classes__ = [FilesMenu, EditMenu, HelpMenu]
  __menu_icons__ = stringList("""files_menu, edit_menu, help_menu""")

  def __init__(self, *args, **kwargs) -> None:
    parent = parseParent(*args)
    QMenuBar.__init__(self, parent)
    self.setupMenus()

  @classmethod
  def getMenuTitles(cls) -> list[str]:
    """Getter-function for list of menu titles"""
    return cls.__menu_titles__

  @classmethod
  def getMenuClasses(cls) -> list[type]:
    """Getter-function for the list of menu classes"""
    return cls.__menu_classes__

  @classmethod
  def getMenuIcons(cls) -> list[str]:
    """Getter-function for the list of menu icons"""
    return cls.__menu_icons__

  def setupMenus(self) -> None:
    """Sets up the menus"""
    filesMenu = FilesMenu('Files', self)
    self.addMenu(filesMenu)
    editMenu = EditMenu('Edit', self)
    self.addMenu(editMenu)
    helpMenu = HelpMenu('Help', self)
    self.addMenu(helpMenu)
