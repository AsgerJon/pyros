"""The AbstractMenu class subclasses QMenu providing the menu class."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from PySide6.QtCore import Qt
from PySide6.QtGui import QAction, QKeySequence
from PySide6.QtWidgets import QMenu
from icecream import ic

from morevistside import parseParent
from morevistside.actionmenus import getIcon
from morevistutils.waitaminute import typeMsg

ic.configureOutput(includeContext=True)


class AbstractMenu(QMenu):
  """The AbstractMenu class subclasses QMenu providing the menu class."""

  def __init__(self, *args, **kwargs) -> None:
    self.__icon__ = kwargs.get('icon', True)
    self.__field_owner__ = None
    self.__field_name__ = None
    parent = parseParent(*args)
    if parent is None:
      e = """Unable to parse parent widget!"""
      raise TypeError(e)
    title = None
    for arg in args:
      if isinstance(arg, str):
        title = arg
    if title is None:
      e = """Menu constructor received no title!"""
      raise ValueError(e)
    QMenu.__init__(self, title, parent)

    self.setupActions()

  def __set_name__(self, owner: type, name: str) -> None:
    """Invoked automatically upon instantiation in the class body of owner
    class."""
    self.__field_name__ = name
    self.__field_owner__ = owner

  def setupActions(self) -> None:
    """Sets up the menu"""
    for (name, text, shortCut) in self.collectActionInfo():
      if not self.__icon__:
        icon = getIcon('debug')
      elif isinstance(self.__icon__, str):
        icon = getIcon(self.__icon__)
      else:
        icon = getIcon(name)
      action = QAction(icon, text, self.parent())
      setattr(self.parent(), '%sAction' % name, action)
      if shortCut.count() and shortCut.toString():
        action.setShortcut(shortCut)
        action.setShortcutContext(Qt.ShortcutContext.WidgetShortcut)
      self.addAction(action)

  @abstractmethod
  def getNames(self) -> list[str]:
    """Getter-function for list of names"""

  @abstractmethod
  def getShortcuts(self) -> list[str]:
    """Getter-function for list of shortcuts. Must have same length as
    list of names. Insert an empty string or 'None' where a name has no
    shortcut."""

  @abstractmethod
  def getText(self) -> list[str]:
    """Getter-function for the list of action descriptions"""

  def collectActionInfo(self) -> list[tuple[str, str, str]]:
    """Getter-function for actions in this menu"""
    out = []
    names = self.getNames()
    shortCuts = self.getShortcuts()
    texts = self.getText()
    for (name, keys, text) in zip(names, shortCuts, texts):
      shortCut = QKeySequence(keys)
      out.append((name, text, shortCut))
    return out

  def getActions(self) -> list[QAction]:
    """Getter-function for list of actions"""
    out = []
    for action in self.actions():
      out.append(action)
    return out

  def getFieldName(self) -> str:
    """Getter-function for field name"""
    if self.__field_name__ is None:
      return QMenu.title(self)
    if isinstance(self.__field_name__, str):
      return self.__field_name__
    e = typeMsg('__field_name__', self.__field_name__, str)
    raise TypeError(e)

  def __str__(self, ) -> str:
    """String representation. Prints a list of all actions listed."""
    return self.getFieldName()

  def __getattr__(self, key: str) -> QAction:
    """Returns any action at given key or raises expected AttributeError"""
    for action in self.actions():
      if key.lower() in action.text():
        print('yolo')
        return action
    raise AttributeError(key)
