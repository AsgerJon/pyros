"""The MenuBar class subclasses QMenuBar and organizes the menus typically
found at the top of the main application window."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QMenuBar, QMainWindow
from icecream import ic
from vistutils.fields import Field
from vistutils.waitaminute import typeMsg
from morevistside.actionmenus import MenuField

from morevistside.actionmenus import FilesMenu, EditMenu, HelpMenu, DebugMenu

ic.configureOutput(includeContext=True)


class MenuBar(QMenuBar):
  """The MenuBar class subclasses QMenuBar and organizes the menus typically
  found at the top of the main application window."""

  __main_window__ = None

  mainWindow = Field()
  __menu_fields__ = []
  files = MenuField(FilesMenu)
  edit = MenuField(EditMenu)
  aboutQt = MenuField(HelpMenu)
  debug = MenuField(DebugMenu)

  @mainWindow.GET
  def _getMainWindow(self) -> QMainWindow:
    """Explicit getter for the main window."""
    return self.__main_window__

  def __init__(self, mainWindow: QMainWindow, *args, **kwargs) -> None:
    self.__main_window__ = mainWindow
    QMenuBar.__init__(self, mainWindow)

  def setupMenus(self) -> None:
    """Sets up the menus."""
    menuFieldsName = MenuField.getOwnerListName()
    if hasattr(self, menuFieldsName):
      menuFields = getattr(self, menuFieldsName)
      if isinstance(menuFields, list):
        for field in menuFields:
          self.addMenu(field.__get__(self, self.__class__))
      else:
        e = typeMsg('menuFields', menuFields, list)
        raise TypeError(e)
    else:
      e = """MenuBar has no attribute: '%s'""" % menuFieldsName
      raise AttributeError(e)
