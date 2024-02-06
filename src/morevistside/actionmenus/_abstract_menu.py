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
from morevistside.actionmenus import actionFactory, getIcon

ic.configureOutput(includeContext=True)


class AbstractMenu(QMenu):
  """The AbstractMenu class subclasses QMenu providing the menu class."""

  def __init__(self, *args, **kwargs) -> None:
    parent = parseParent(*args)
    title = None
    for arg in args:
      if isinstance(arg, str):
        title = arg
    if title is None:
      e = """Menu constructor received no title!"""
      raise ValueError(e)
    QMenu.__init__(self, title, parent)
    self.setupActions()

  def setupActions(self) -> None:
    """Sets up the menu"""
    for (name, text, shortCut) in self.getActions():
      ic(name)
      icon = getIcon(name, )
      action = QAction(icon, text, self)
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

  def getActions(self) -> list[tuple[str, str, str]]:
    """Getter-function for actions in this menu"""
    out = []
    names = self.getNames()
    shortCuts = self.getShortcuts()
    texts = self.getText()
    for (name, keys, text) in zip(names, shortCuts, texts):
      shortCut = QKeySequence(keys)
      out.append((name, text, shortCut))
    return out
