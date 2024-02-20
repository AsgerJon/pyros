"""AbstractMenu provides an abstract base class for the menus based on the
QMenu class. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QMainWindow, QMenu
from icecream import ic
from vistutils.fields import Field
from vistutils.waitaminute import typeMsg

from morevistside.actionmenus import ActionField


class AbstractMenu(QMenu):
  """AbstractMenu provides an abstract base class for the menus based on the
  QMenu class. """

  __action_fields__ = []
  mainWindow = Field()

  def __init__(self, mainWindow: QMainWindow, *args, **kwargs) -> None:
    self.__main_window__ = mainWindow
    QMenu.__init__(self, mainWindow)

  def setupActions(self) -> None:
    """Sets up the actions for the menu."""
    fieldListName = ActionField.getOwnerListName()
    if hasattr(self, fieldListName):
      actionFields = getattr(self, fieldListName)
      if isinstance(actionFields, list):
        for field in actionFields:
          self.addAction(field.__get__(self, self.__class__))
      else:
        e = typeMsg('menuFields', actionFields, list)
        raise TypeError(e)
    else:
      e = """The menu has no attribute: '%s'""" % fieldListName
      raise AttributeError(e)

  @mainWindow.GET
  def getMainWindow(self) -> QMainWindow:
    """Getter-function for the main window."""
    ic(self)
    return self.__main_window__
