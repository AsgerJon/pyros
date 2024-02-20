"""BaseWindow provides the main application window with menus and
actions.."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from PySide6.QtCore import Qt
from PySide6.QtGui import QKeyEvent
from PySide6.QtWidgets import QMainWindow
from icecream import ic

from PySide6.QtWidgets import QMenuBar as MenuBar

ic.configureOutput(includeContext=True)


class BaseWindow(QMainWindow):
  """BaseWindow provides the main application window with menus and
  actions."""

  def __init__(self, *args, **kwargs) -> None:
    QMainWindow.__init__(self, None)
    self.__menu_bar__ = None

  def setupMenus(self) -> None:
    """Sets up the menus for the window."""
    self.__menu_bar__ = MenuBar(self)
    self.setMenuBar(self.__menu_bar__)
    self.__menu_bar__.setupMenus()

  @abstractmethod
  def initUI(self) -> None:
    """Initializes the user interface. Subclasses are required to provide
    implementation of this method. """

  def show(self) -> None:
    """Shows the window."""
    self.setupMenus()
    self.initUI()
    QMainWindow.show(self)

  def keyReleaseEvent(self, event: QKeyEvent) -> None:
    """Reimplements the keyReleaseEvent method to handle key presses."""
    QMainWindow.keyReleaseEvent(self, event)
    if event.key() == Qt.Key_F1:
      ic(self.__menu_bar__)
      ic(self.menuBar())
      ic(self.__menu_bar__.geometry())
    if event.key() == Qt.Key_F2:
      ic(self.__menu_bar__.children())
      if self.__menu_bar__.update():
        for menu in self.__menu_bar__.children():
          ic(menu)
