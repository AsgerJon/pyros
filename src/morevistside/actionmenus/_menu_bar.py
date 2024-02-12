"""The MenuBar class subclasses QMenuBar and organizes the menus typically
found at the top of the main application window."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QMenuBar
from icecream import ic
from vistutils import stringList

from morevistside import parseParent
from morevistside.actionmenus import FilesMenu, EditMenu, HelpMenu, \
  DebugMenu, AbstractMenu

ic.configureOutput(includeContext=True)


class MenuBar(QMenuBar):
  """The MenuBar class subclasses QMenuBar and organizes the menus typically
  found at the top of the main application window."""

  __menu_titles__ = stringList("""Files, Edit, Help, DEBUG""")
  __menu_classes__ = [FilesMenu, EditMenu, HelpMenu, DebugMenu]
  __menu_icons__ = stringList("""files_menu, edit_menu, help_menu, 
    risitas""")

  def __init__(self, *args, **kwargs) -> None:
    parent = parseParent(*args)
    QMenuBar.__init__(self, parent)
    self.__files_menu__ = None
    self.__edit_menu__ = None
    self.__help_menu__ = None
    self.__debug_menu__ = None
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
    self.__files_menu__ = FilesMenu('Files', self)
    self.addMenu(self.__files_menu__)
    self.__edit_menu__ = EditMenu('Edit', self)
    self.addMenu(self.__edit_menu__)
    self.__help_menu__ = HelpMenu('Help', self)
    self.addMenu(self.__help_menu__)
    self.__debug_menu__ = DebugMenu('DEBUG', self)
    self.addMenu(self.__debug_menu__)

  def getMenus(self) -> list[AbstractMenu]:
    """Getter-function for list of menus"""
    out = []
    for menu in self.children():
      if isinstance(menu, AbstractMenu):
        out.append(menu)
    return out
