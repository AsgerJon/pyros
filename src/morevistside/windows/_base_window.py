"""The BaseWindow class provides the baseclass for the main application
window. It inherits directly from QMainWindow and organizes menus and
actions."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
from abc import abstractmethod

from PySide6.QtGui import QAction
from PySide6.QtWidgets import QMainWindow
from icecream import ic
from vistutils.fields import Field

from morevistside import parseParent
from morevistside.actionmenus import MenuBar
from morevistutils.waitaminute import typeMsg

ic.configureOutput(includeContext=True)


class BaseWindow(QMainWindow):
  """The BaseWindow class provides the baseclass for the main application
  window. It inherits directly from QMainWindow and organizes menus and
  actions."""

  menuBar = Field()

  @menuBar.GET
  def getMenuBar(self, *args, **kwargs) -> MenuBar:
    """Getter-function for menu bar """
    if self.__menu_bar__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.setupMenuBar()
      return self.getMenuBar()
    if isinstance(self.__menu_bar__, MenuBar):
      e = typeMsg('__menu_bar__', self.__menu_bar__, MenuBar)
      raise TypeError(e)

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
    self.newAction = QAction()
    self.openAction = QAction()
    self.saveAction = QAction()
    self.saveAsAction = QAction()
    self.cutAction = QAction()
    self.copyAction = QAction()
    self.pasteAction = QAction()
    self.undoAction = QAction()
    self.redoAction = QAction()
    self.aboutQtAction = QAction()

  def connectActions(self) -> None:
    """Method responsible for connecting actions"""

  def setupMenuBar(self, ) -> None:
    """Sets up the menu bar"""
    self.__menu_bar__ = MenuBar(self)
    self.setMenuBar(self.__menu_bar__)

  def getMenuBar(self, **kwargs) -> MenuBar:
    """Getter-function for menu bar"""
    if self.__menu_bar__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.setupMenuBar()
      return self.getMenuBar(_recursion=True)
    if isinstance(self.__menu_bar__, MenuBar):
      return self.__menu_bar__
    e = typeMsg('__menu_bar__', self.__menu_bar__, MenuBar)
    raise TypeError(e)

  @abstractmethod
  def initUI(self) -> None:
    """Initializes the UI by setting up widgets and layouts. Subclasses
    must implement this method."""

  def show(self) -> None:
    """Reimplementation invoking initUI, before parent show"""
    self.initUI()
    return QMainWindow.show(self)
