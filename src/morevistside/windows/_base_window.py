"""The BaseWindow class provides the baseclass for the main application
window. It inherits directly from QMainWindow and organizes menus and
actions."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
from abc import abstractmethod

from PySide6.QtWidgets import QMainWindow

from morevistside import parseParent
from morevistside.actionmenus import MenuBar


class BaseWindow(QMainWindow):
  """The BaseWindow class provides the baseclass for the main application
  window. It inherits directly from QMainWindow and organizes menus and
  actions."""

  @staticmethod
  def _getWindowTitle() -> str:
    """Getter-function for window title"""
    return os.environ.get('WINDOW_TITLE', 'Main Window')

  def __init__(self, *args, **kwargs) -> None:
    parent = parseParent(*args)
    self.__menu_bar__ = None
    QMainWindow.__init__(self, parent)
    self.setWindowTitle(self._getWindowTitle())
    self.setupMenuBar()

  def setupMenuBar(self, ) -> None:
    """Sets up the menu bar"""
    self.__menu_bar__ = MenuBar(self)
    self.setMenuBar(self.__menu_bar__)

  @abstractmethod
  def initUI(self) -> None:
    """Initializes the UI by setting up widgets and layouts. Subclasses
    must implement this method."""

  def show(self) -> None:
    """Reimplementation invoking initUI, before parent show"""
    self.initUI()
    return QMainWindow.show(self)
