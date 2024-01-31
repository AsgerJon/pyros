"""The BaseWindow class provides the baseclass for the main application
window. It inherits directly from QMainWindow and organizes menus and
actions."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from PySide6.QtWidgets import QMainWindow

from morevistside import parseParent
from morevistside.actionmenus import MenuBar, StatusBar
from morevistutils.fields import SpecialField


class BaseWindow(QMainWindow):
  """The BaseWindow class provides the baseclass for the main application
  window. It inherits directly from QMainWindow and organizes menus and
  actions."""

  mainMenuBar = SpecialField(MenuBar)
  mainStatusBar = SpecialField(StatusBar)

  @mainMenuBar.CREATE
  def _createMainMenuBar(self, *args, **kwargs) -> MenuBar:
    """Creator function for main menubar"""
    return MenuBar(self, )

  @mainStatusBar.CREATE
  def _createMainStatusBar(self, *args, **kwargs) -> StatusBar:
    """Creator function for statusbar"""
    return StatusBar(self, )

  def __init__(self, *args, **kwargs) -> None:
    parent = parseParent(*args)
    QMainWindow.__init__(self, parent)
    self.setMenuBar(self.mainMenuBar)
    self.setStatusBar(self.mainStatusBar)

  @abstractmethod
  def initUI(self) -> None:
    """Initializes the UI by setting up widgets and layouts. Subclasses
    must implement this method."""

  def show(self) -> None:
    """Reimplementation invoking initUI, before parent show"""
    self.initUI()
    return QMainWindow.show(self)
