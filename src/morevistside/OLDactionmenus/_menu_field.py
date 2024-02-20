"""MenuField subclasses ClassField to specifically create menus. It
expects as value class a subclass of QMenu."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Type, TYPE_CHECKING
from PySide6.QtWidgets import QMainWindow, QMenuBar
from icecream import ic
from vistutils.waitaminute import typeMsg

from morevistside.actionmenus import AbstractMenu
from morevistutils.fields import ClassField

if TYPE_CHECKING:
  from morevistside.actionmenus import MenuBar


class MenuField(ClassField):
  """MenuField subclasses ClassField to specifically create menus. It
  expects as value class a subclass of QMenu."""

  def _createInstance(self,
                      instance: MenuBar,
                      owner: Type[MenuBar]) -> None:
    """Instantiates the inner class. """
    if instance is None:
      raise NotImplementedError
    if not isinstance(instance, QMenuBar):
      e = typeMsg('instance', instance, QMenuBar)
      raise TypeError(e)
    cls = self._getValueClass()
    if not issubclass(cls, AbstractMenu):
      e = """The value class must be a subclass of AbstractMenu."""
      raise TypeError(e)
    pvtName = self._getPrivateName()
    ownerListName = self.getOwnerListName()
    mainWindow = instance.mainWindow
    newMenu = cls(mainWindow)
    newMenu.setupActions()
    setattr(instance, self._getPrivateName(), newMenu)
    existingMenus = getattr(owner, ownerListName, [])
    setattr(owner, ownerListName, [*existingMenus, newMenu])

  def __init__(self, menu: Type[AbstractMenu], *args, **kwargs) -> None:
    ClassField.__init__(self, menu, *args, **kwargs)

  def _getFieldOwner(self) -> type:
    """Reimplementation ensuring that only the MenuBar class can own
    instances of MenuField."""
    return ClassField._getFieldOwner(self)

  def getMainWindow(self) -> QMainWindow:
    """Getter-function for the main window."""
    owner = self._getFieldOwner()
    mainWindow = owner.mainWindow
    if isinstance(mainWindow, QMainWindow):
      return mainWindow
    e = typeMsg('mainWindow', mainWindow, QMainWindow)
    raise TypeError(e)
