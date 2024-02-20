"""ActionField subclasses ClassField to specifically create menus. It
expects as value class a subclass of QAction. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
from typing import Type, TYPE_CHECKING

from PySide6.QtGui import QAction, QIcon, QPixmap
from PySide6.QtWidgets import QMainWindow, QMenu
from icecream import ic
from vistutils.dirs import getProjectRoot
from vistutils.waitaminute import typeMsg

from morevistutils.fields import ClassField

if TYPE_CHECKING:
  from morevistside.actionmenus import AbstractMenu

ic.configureOutput(includeContext=True)


class ActionField(ClassField):
  """ActionField subclasses ClassField to specifically create menus. It
  expects as value class a subclass of QAction."""

  @staticmethod
  def _getIconDir() -> str:
    """Getter-function for the icon."""
    root = getProjectRoot()
    iconPath = os.path.join(
      root, 'src', 'morevistside', 'actionmenus', 'icons')
    if os.path.isdir(iconPath):
      return iconPath

  def _getIcon(self, ) -> QIcon:
    """Getter-function for the icon"""
    iconDir = self._getIconDir()
    iconName = self._getActionName()
    pix = QPixmap(os.path.join(iconDir, iconName))
    return QIcon(pix)

  def _createInstance(self,
                      instance: AbstractMenu,
                      owner: Type[AbstractMenu]) -> None:
    """Instantiates the inner class. """
    if instance is None:
      raise NotImplementedError
    if not isinstance(instance, QMenu):
      e = typeMsg('instance', instance, QMenu)
      raise TypeError(e)
    cls = self._getValueClass()
    if not issubclass(cls, QAction):
      e = """The value class must be a subclass of QAction."""
      raise TypeError(e)
    pvtName = self._getPrivateName()
    ownerListName = self.getOwnerListName()
    icon = self._getIcon()
    title = self._getActionName()
    mainWindow = instance.mainWindow
    action = instance.addAction(icon, title)
    action.setParent(mainWindow)
    existingActions = getattr(instance, ownerListName, [])
    setattr(instance, ownerListName, [*existingActions, action])
    setattr(instance, pvtName, action)

  def __init__(self, actionName: str, *args, **kwargs) -> None:
    self.__action_name__ = actionName
    ClassField.__init__(self, QAction, *args, **kwargs)

  def _getActionName(self) -> str:
    """Getter-function for the action name."""
    return self.__action_name__

  def _getFieldOwner(self) -> Type[QMenu]:
    """Reimplementation ensuring that only the MenuBar class can own
    instances of MenuField."""
    owner = self.__field_owner__
    if owner is QMenu or issubclass(owner, QMenu):
      return owner
    e = """Only instances of QMenu or subclasses may own instances of 
    ActionField!"""

  def getMainWindow(self) -> QMainWindow:
    """Getter-function for the main window."""
    owner = self._getFieldOwner()
    ic(owner)
    return owner.mainWindow
