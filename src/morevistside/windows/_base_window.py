"""The BaseWindow class provides the baseclass for the main application
window. It inherits directly from QMainWindow and organizes menus and
actions."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
from abc import abstractmethod

from PySide6.QtCore import Qt
from PySide6.QtGui import QAction, QKeySequence
from PySide6.QtWidgets import QMainWindow, QApplication, QMenu
from icecream import ic

from morevistside import parseParent
from morevistside.actionmenus import getIcon
from morevistutils.waitaminute import typeMsg

ic.configureOutput(includeContext=True)


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
    QMainWindow.__init__(self, parent)
    self.setWindowTitle(self._getWindowTitle())
    self.filesMenu = None
    self.editMenu = None
    self.helpMenu = None
    self.debugMenu = None
    self.newAction = None
    self.openAction = None
    self.saveAction = None
    self.saveAsAction = None
    self.exitAction = None
    self.cutAction = None
    self.copyAction = None
    self.pasteAction = None
    self.undoAction = None
    self.redoAction = None
    self.aboutQtAction = None
    self.debugAction01 = None
    self.debugAction02 = None
    self.debugAction03 = None
    self.debugAction04 = None
    self.debugAction05 = None
    self.debugAction06 = None
    self.debugAction07 = None
    self.debugAction08 = None
    self.debugAction09 = None
    self.debugAction10 = None
    self.debugAction11 = None
    self.debugAction12 = None
    self.setupMenuBar()

  def createDebugAction01(self) -> None:
    """Debug action 01 creator"""
    icon = getIcon('debug')
    text = '**DEBUG 01**'
    tooltip = 'Debug 01'
    shortCut = QKeySequence('CTRL+F1')
    parent = self.parent()
    self.debugAction01 = QAction(parent)
    self.debugAction01.setText(text)
    self.debugAction01.setIcon(icon)
    self.debugAction01.setShortcut(shortCut)
    self.debugAction01.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.debugAction01.setToolTip(tooltip)
    self.debugAction01.triggered.connect(self.debugFunc01)
    self.getDebugMenu().addAction(self.debugAction01)

  def getDebugAction01(self, **kwargs) -> QAction:
    """Getter-function for debug action 01"""
    if self.debugAction01 is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createDebugAction01()
      return self.getDebugAction01(_recursion=True)
    if isinstance(self.debugAction01, QAction):
      return self.debugAction01
    e = typeMsg('debugAction01', self.debugAction01, QAction)
    raise TypeError(e)

  def createDebugAction02(self) -> None:
    """Debug action 02 creator"""
    icon = getIcon('debug')
    text = '**DEBUG 02**'
    tooltip = 'Debug 02'
    shortCut = QKeySequence('CTRL+F2')
    parent = self.parent()
    self.debugAction02 = QAction(parent)
    self.debugAction02.setText(text)
    self.debugAction02.setIcon(icon)
    self.debugAction02.setShortcut(shortCut)
    self.debugAction02.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.debugAction02.setToolTip(tooltip)
    self.debugAction02.triggered.connect(self.debugFunc02)
    self.getDebugMenu().addAction(self.debugAction02)

  def getDebugAction02(self, **kwargs) -> QAction:
    """Getter-function for debug action 02"""
    if self.debugAction02 is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createDebugAction02()
      return self.getDebugAction02(_recursion=True)
    if isinstance(self.debugAction02, QAction):
      return self.debugAction02
    e = typeMsg('debugAction02', self.debugAction02, QAction)
    raise TypeError(e)

  def createDebugAction03(self) -> None:
    """Debug action 03 creator"""
    icon = getIcon('debug')
    text = '**DEBUG 03**'
    tooltip = 'Debug 03'
    shortCut = QKeySequence('CTRL+F3')
    parent = self.parent()
    self.debugAction03 = QAction(parent)
    self.debugAction03.setText(text)
    self.debugAction03.setIcon(icon)
    self.debugAction03.setShortcut(shortCut)
    self.debugAction03.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.debugAction03.setToolTip(tooltip)
    self.debugAction03.triggered.connect(self.debugFunc03)
    self.getDebugMenu().addAction(self.debugAction03)

  def getDebugAction03(self, **kwargs) -> QAction:
    """Getter-function for debug action 03"""
    if self.debugAction03 is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createDebugAction03()
      return self.getDebugAction03(_recursion=True)
    if isinstance(self.debugAction03, QAction):
      return self.debugAction03
    e = typeMsg('debugAction03', self.debugAction03, QAction)
    raise TypeError(e)

  def createDebugAction04(self) -> None:
    """Debug action 04 creator"""
    icon = getIcon('debug')
    text = '**DEBUG 04**'
    tooltip = 'Debug 04'
    shortCut = QKeySequence('CTRL+F4')
    parent = self.parent()
    self.debugAction04 = QAction(parent)
    self.debugAction04.setText(text)
    self.debugAction04.setIcon(icon)
    self.debugAction04.setShortcut(shortCut)
    self.debugAction04.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.debugAction04.setToolTip(tooltip)
    self.debugAction04.triggered.connect(self.debugFunc04)
    self.getDebugMenu().addAction(self.debugAction04)

  def getDebugAction04(self, **kwargs) -> QAction:
    """Getter-function for debug action 04"""
    if self.debugAction04 is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createDebugAction04()
      return self.getDebugAction04(_recursion=True)
    if isinstance(self.debugAction04, QAction):
      return self.debugAction04
    e = typeMsg('debugAction04', self.debugAction04, QAction)
    raise TypeError(e)

  def createDebugAction05(self) -> None:
    """Debug action 05 creator"""
    icon = getIcon('debug')
    text = '**DEBUG 05**'
    tooltip = 'Debug 05'
    shortCut = QKeySequence('CTRL+F5')
    parent = self.parent()
    self.debugAction05 = QAction(parent)
    self.debugAction05.setText(text)
    self.debugAction05.setIcon(icon)
    self.debugAction05.setShortcut(shortCut)
    self.debugAction05.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.debugAction05.setToolTip(tooltip)
    self.debugAction05.triggered.connect(self.debugFunc05)
    self.getDebugMenu().addAction(self.debugAction05)

  def getDebugAction05(self, **kwargs) -> QAction:
    """Getter-function for debug action 05"""
    if self.debugAction05 is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createDebugAction05()
      return self.getDebugAction05(_recursion=True)
    if isinstance(self.debugAction05, QAction):
      return self.debugAction05
    e = typeMsg('debugAction05', self.debugAction05, QAction)
    raise TypeError(e)

  def createDebugAction06(self) -> None:
    """Debug action 06 creator"""
    icon = getIcon('debug')
    text = '**DEBUG 06**'
    tooltip = 'Debug 06'
    shortCut = QKeySequence('CTRL+F6')
    parent = self.parent()
    self.debugAction06 = QAction(parent)
    self.debugAction06.setText(text)
    self.debugAction06.setIcon(icon)
    self.debugAction06.setShortcut(shortCut)
    self.debugAction06.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.debugAction06.setToolTip(tooltip)
    self.debugAction06.triggered.connect(self.debugFunc06)
    self.getDebugMenu().addAction(self.debugAction06)

  def getDebugAction06(self, **kwargs) -> QAction:
    """Getter-function for debug action 06"""
    if self.debugAction06 is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createDebugAction06()
      return self.getDebugAction06(_recursion=True)
    if isinstance(self.debugAction06, QAction):
      return self.debugAction06
    e = typeMsg('debugAction06', self.debugAction06, QAction)
    raise TypeError(e)

  def createDebugAction07(self) -> None:
    """Debug action 07 creator"""
    icon = getIcon('debug')
    text = '**DEBUG 07**'
    tooltip = 'Debug 07'
    shortCut = QKeySequence('CTRL+F7')
    parent = self.parent()
    self.debugAction07 = QAction(parent)
    self.debugAction07.setText(text)
    self.debugAction07.setIcon(icon)
    self.debugAction07.setShortcut(shortCut)
    self.debugAction07.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.debugAction07.setToolTip(tooltip)
    self.debugAction07.triggered.connect(self.debugFunc07)
    self.getDebugMenu().addAction(self.debugAction07)

  def getDebugAction07(self, **kwargs) -> QAction:
    """Getter-function for debug action 07"""
    if self.debugAction07 is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createDebugAction07()
      return self.getDebugAction07(_recursion=True)
    if isinstance(self.debugAction07, QAction):
      return self.debugAction07
    e = typeMsg('debugAction07', self.debugAction07, QAction)
    raise TypeError(e)

  def createDebugAction08(self) -> None:
    """Debug action 08 creator"""
    icon = getIcon('debug')
    text = '**DEBUG 08**'
    tooltip = 'Debug 08'
    shortCut = QKeySequence('CTRL+F8')
    parent = self.parent()
    self.debugAction08 = QAction(parent)
    self.debugAction08.setText(text)
    self.debugAction08.setIcon(icon)
    self.debugAction08.setShortcut(shortCut)
    self.debugAction08.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.debugAction08.setToolTip(tooltip)
    self.debugAction08.triggered.connect(self.debugFunc08)
    self.getDebugMenu().addAction(self.debugAction08)

  def getDebugAction08(self, **kwargs) -> QAction:
    """Getter-function for debug action 08"""
    if self.debugAction08 is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createDebugAction08()
      return self.getDebugAction08(_recursion=True)
    if isinstance(self.debugAction08, QAction):
      return self.debugAction08
    e = typeMsg('debugAction08', self.debugAction08, QAction)
    raise TypeError(e)

  def createDebugAction09(self) -> None:
    """Debug action 09 creator"""
    icon = getIcon('debug')
    text = '**DEBUG 09**'
    tooltip = 'Debug 09'
    shortCut = QKeySequence('CTRL+F9')
    parent = self.parent()
    self.debugAction09 = QAction(parent)
    self.debugAction09.setText(text)
    self.debugAction09.setIcon(icon)
    self.debugAction09.setShortcut(shortCut)
    self.debugAction09.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.debugAction09.setToolTip(tooltip)
    self.debugAction09.triggered.connect(self.debugFunc09)
    self.getDebugMenu().addAction(self.debugAction09)

  def getDebugAction09(self, **kwargs) -> QAction:
    """Getter-function for debug action 09"""
    if self.debugAction09 is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createDebugAction09()
      return self.getDebugAction09(_recursion=True)
    if isinstance(self.debugAction09, QAction):
      return self.debugAction09
    e = typeMsg('debugAction09', self.debugAction09, QAction)
    raise TypeError(e)

  def createDebugAction10(self) -> None:
    """Debug action 10 creator"""
    icon = getIcon('debug')
    text = '**DEBUG 10**'
    tooltip = 'Debug 10'
    shortCut = QKeySequence('CTRL+F10')
    parent = self.parent()
    self.debugAction10 = QAction(parent)
    self.debugAction10.setText(text)
    self.debugAction10.setIcon(icon)
    self.debugAction10.setShortcut(shortCut)
    self.debugAction10.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.debugAction10.setToolTip(tooltip)
    self.debugAction10.triggered.connect(self.debugFunc10)
    self.getDebugMenu().addAction(self.debugAction10)

  def getDebugAction10(self, **kwargs) -> QAction:
    """Getter-function for debug action 10"""
    if self.debugAction10 is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createDebugAction10()
      return self.getDebugAction10(_recursion=True)
    if isinstance(self.debugAction10, QAction):
      return self.debugAction10
    e = typeMsg('debugAction10', self.debugAction10, QAction)
    raise TypeError(e)

  def createDebugAction11(self) -> None:
    """Debug action 11 creator"""
    icon = getIcon('debug')
    text = '**DEBUG 11**'
    tooltip = 'Debug 11'
    shortCut = QKeySequence('CTRL+F11')
    parent = self.parent()
    self.debugAction11 = QAction(parent)
    self.debugAction11.setText(text)
    self.debugAction11.setIcon(icon)
    self.debugAction11.setShortcut(shortCut)
    self.debugAction11.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.debugAction11.setToolTip(tooltip)
    self.debugAction11.triggered.connect(self.debugFunc11)
    self.getDebugMenu().addAction(self.debugAction11)

  def getDebugAction11(self, **kwargs) -> QAction:
    """Getter-function for debug action 11"""
    if self.debugAction11 is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createDebugAction11()
      return self.getDebugAction11(_recursion=True)
    if isinstance(self.debugAction11, QAction):
      return self.debugAction11
    e = typeMsg('debugAction11', self.debugAction11, QAction)
    raise TypeError(e)

  def createDebugAction12(self) -> None:
    """Debug action 12 creator"""
    icon = getIcon('debug')
    text = '**DEBUG 12**'
    tooltip = 'Debug 12'
    shortCut = QKeySequence('CTRL+F12')
    parent = self.parent()
    self.debugAction12 = QAction(parent)
    self.debugAction12.setText(text)
    self.debugAction12.setIcon(icon)
    self.debugAction12.setShortcut(shortCut)
    self.debugAction12.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.debugAction12.setToolTip(tooltip)
    self.debugAction12.triggered.connect(self.debugFunc12)
    self.getDebugMenu().addAction(self.debugAction12)

  def getDebugAction12(self, **kwargs) -> QAction:
    """Getter-function for debug action 12"""
    if self.debugAction12 is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createDebugAction12()
      return self.getDebugAction12(_recursion=True)
    if isinstance(self.debugAction12, QAction):
      return self.debugAction12
    e = typeMsg('debugAction12', self.debugAction12, QAction)
    raise TypeError(e)

  def debugFunc01(self, ) -> None:
    """Debug function 01"""

  def debugFunc02(self, ) -> None:
    """Debug function 02"""

  def debugFunc03(self, ) -> None:
    """Debug function 03"""

  def debugFunc04(self, ) -> None:
    """Debug function 04"""

  def debugFunc05(self, ) -> None:
    """Debug function 05"""

  def debugFunc06(self, ) -> None:
    """Debug function 06"""

  def debugFunc07(self, ) -> None:
    """Debug function 07"""

  def debugFunc08(self, ) -> None:
    """Debug function 08"""

  def debugFunc09(self, ) -> None:
    """Debug function 09"""

  def debugFunc10(self, ) -> None:
    """Debug function 10"""

  def debugFunc11(self, ) -> None:
    """Debug function 11"""

  def debugFunc12(self, ) -> None:
    """Debug function 12"""

  def newFunc(self, ) -> None:
    """Docstring for new action"""

  def openFunc(self, ) -> None:
    """Docstring for open action"""

  def saveFunc(self, ) -> None:
    """Docstring for save action"""

  def saveAsFunc(self, ) -> None:
    """Docstring for saveAs action"""

  def exitFunc(self) -> None:
    """Docstring for exit action"""
    self.close()

  def cutFunc(self, ) -> None:
    """Docstring for cut action"""

  def copyFunc(self, ) -> None:
    """Docstring for copy action"""

  def pasteFunc(self, ) -> None:
    """Docstring for paste action"""

  def undoFunc(self, ) -> None:
    """Docstring for undo action"""

  def redoFunc(self, ) -> None:
    """Docstring for redo action"""

  @staticmethod
  def aboutQtFunc() -> None:
    """Docstring for About Qt Action"""
    QApplication.aboutQt()

  def setupMenuBar(self, ) -> None:
    """Sets up the menu bar"""
    self.menuBar().addMenu(self.getFilesMenu())
    self.menuBar().addMenu(self.getEditMenu())
    self.menuBar().addMenu(self.getHelpMenu())
    self.menuBar().addMenu(self.getDebugMenu())

  def createFilesMenu(self) -> None:
    """Creator function for files.json menu"""
    filesIcon = getIcon('files_menu')
    self.filesMenu = self.menuBar().addMenu('Files')
    # self.filesMenu.setIcon(filesIcon)
    # self.filesMenu.setTitle('Files')
    self.getNewAction()
    self.getOpenAction()
    self.getSaveAction()
    self.getSaveAsAction()
    self.getExitAction()

  def createEditMenu(self) -> None:
    """Creator function for edit menu"""
    editIcon = getIcon('edit_menu')
    self.editMenu = self.menuBar().addMenu('Edit')
    # self.editMenu.setIcon(editIcon)
    self.getCutAction()
    self.getCopyAction()
    self.getPasteAction()
    self.getUndoAction()
    self.getRedoAction()

  def createHelpMenu(self) -> None:
    """Creator function for help menu"""
    helpIcon = getIcon('help_menu')
    self.helpMenu = self.menuBar().addMenu('Help')
    # self.helpMenu.setIcon(helpIcon)
    self.getAboutQtAction()

  def createDebugMenu(self) -> None:
    """Creator function for debug menu"""
    debugIcon = getIcon('debug')
    self.debugMenu = self.menuBar().addMenu('Debug')
    # self.debugMenu.setIcon(debugIcon)
    self.getDebugAction01()
    self.getDebugAction02()
    self.getDebugAction03()
    self.getDebugAction04()
    self.getDebugAction05()
    self.getDebugAction06()
    self.getDebugAction07()
    self.getDebugAction08()
    self.getDebugAction09()
    self.getDebugAction10()
    self.getDebugAction11()
    self.getDebugAction12()

  def getFilesMenu(self, **kwargs) -> QMenu:
    """Getter-function for files.json menu"""
    if self.filesMenu is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createFilesMenu()
      return self.getFilesMenu(_recursion=True)
    if isinstance(self.filesMenu, QMenu):
      return self.filesMenu
    e = typeMsg('filesMenu', self.filesMenu, QMenu)
    raise TypeError(e)

  def getEditMenu(self, **kwargs) -> QMenu:
    """Getter-function for edit menu"""
    if self.editMenu is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createEditMenu()
      return self.getEditMenu(_recursion=True)
    if isinstance(self.editMenu, QMenu):
      return self.editMenu
    e = typeMsg('editMenu', self.editMenu, QMenu)
    raise TypeError(e)

  def getHelpMenu(self, **kwargs) -> QMenu:
    """Getter-function for help menu"""
    if self.helpMenu is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createHelpMenu()
      return self.getHelpMenu(_recursion=True)
    if isinstance(self.helpMenu, QMenu):
      return self.helpMenu
    e = typeMsg('helpMenu', self.helpMenu, QMenu)
    raise TypeError(e)

  def getDebugMenu(self, **kwargs) -> QMenu:
    """Getter-function for debug menu"""
    if self.debugMenu is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self.createDebugMenu()
      return self.getDebugMenu(_recursion=True)
    if isinstance(self.debugMenu, QMenu):
      return self.debugMenu
    e = typeMsg('debugMenu', self.debugMenu, QMenu)
    raise TypeError(e)

  def createNewAction(self) -> None:
    """Creator function for new action"""
    icon = getIcon('new')
    text = 'New'
    tooltip = 'New Project'
    shortCut = QKeySequence('CTRL+N')
    parent = self.parent()
    self.newAction = QAction(parent)
    self.newAction.setText(text)
    self.newAction.setIcon(icon)
    self.newAction.setShortcut(shortCut)
    self.newAction.setShortcutContext(Qt.ShortcutContext.ApplicationShortcut)
    self.newAction.setToolTip(tooltip)
    self.newAction.triggered.connect(self.newFunc)
    self.getFilesMenu().addAction(self.newAction)

  def createOpenAction(self) -> None:
    """Creates an action to open a file."""
    icon = getIcon('open')
    text = 'Open'
    tooltip = 'Open Existing Project'
    shortCut = QKeySequence('CTRL+O')
    parent = self.parent()
    self.openAction = QAction(parent)
    self.openAction.setText(text)
    self.openAction.setIcon(icon)
    self.openAction.setShortcut(shortCut)
    self.openAction.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.openAction.setToolTip(tooltip)
    self.openAction.triggered.connect(self.openFunc)
    self.getFilesMenu().addAction(self.openAction)

  def createSaveAction(self) -> None:
    """Creates an action to save the current project."""
    icon = getIcon('save')
    text = 'Save'
    tooltip = 'Save Project'
    shortCut = QKeySequence('CTRL+S')
    parent = self.parent()
    self.saveAction = QAction(parent)
    self.saveAction.setText(text)
    self.saveAction.setIcon(icon)
    self.saveAction.setShortcut(shortCut)
    self.saveAction.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.saveAction.setToolTip(tooltip)
    self.saveAction.triggered.connect(self.saveFunc)
    self.getFilesMenu().addAction(self.saveAction)

  def createSaveAsAction(self) -> None:
    """Creates an action for saving the project under a new name."""
    icon = getIcon('save_as')
    text = 'Save As...'
    tooltip = 'Save Project As...'
    shortCut = QKeySequence('CTRL+Shift+S')
    parent = self.parent()
    self.saveAsAction = QAction(parent)
    self.saveAsAction.setText(text)
    self.saveAsAction.setIcon(icon)
    self.saveAsAction.setShortcut(shortCut)
    self.saveAsAction.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.saveAsAction.setToolTip(tooltip)
    self.saveAsAction.triggered.connect(self.saveAsFunc)
    self.getFilesMenu().addAction(self.saveAsAction)

  def createExitAction(self) -> None:
    """Creates an action to exit the application."""
    icon = getIcon('exit')
    text = 'Exit'
    tooltip = 'Exit Application'
    shortCut = QKeySequence('ALT+F4')
    parent = self.parent()
    self.exitAction = QAction(parent)
    self.exitAction.setText(text)
    self.exitAction.setIcon(icon)
    self.exitAction.setShortcut(shortCut)
    self.exitAction.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.exitAction.setToolTip(tooltip)
    self.exitAction.triggered.connect(self.exitFunc)
    self.getFilesMenu().addAction(self.exitAction)

  def getNewAction(self, **kwargs) -> QAction:
    """Getter-function for new action."""
    if self.newAction is None:
      if kwargs.get('_recursion', False):
        raise RecursionError('Recursive call detected in getNewAction')
      self.createNewAction()
      return self.getNewAction(_recursion=True)
    if isinstance(self.newAction, QAction):
      return self.newAction
    e = typeMsg('newAction', self.newAction, QAction)
    raise TypeError(e)

  def getOpenAction(self, **kwargs) -> QAction:
    """Getter-function for open action."""
    if self.openAction is None:
      if kwargs.get('_recursion', False):
        raise RecursionError('Recursive call detected in getOpenAction')
      self.createOpenAction()
      return self.getOpenAction(_recursion=True)
    if isinstance(self.openAction, QAction):
      return self.openAction
    e = typeMsg('openAction', self.openAction, QAction)
    raise TypeError(e)

  def getSaveAction(self, **kwargs) -> QAction:
    """Getter-function for save action."""
    if self.saveAction is None:
      if kwargs.get('_recursion', False):
        raise RecursionError('Recursive call detected in getSaveAction')
      self.createSaveAction()
      return self.getSaveAction(_recursion=True)
    if isinstance(self.saveAction, QAction):
      return self.saveAction
    e = typeMsg('saveAction', self.saveAction, QAction)
    raise TypeError(e)

  def getSaveAsAction(self, **kwargs) -> QAction:
    """Getter-function for save as action."""
    if self.saveAsAction is None:
      if kwargs.get('_recursion', False):
        raise RecursionError('Recursive call detected in getSaveAsAction')
      self.createSaveAsAction()
      return self.getSaveAsAction(_recursion=True)
    if isinstance(self.saveAsAction, QAction):
      return self.saveAsAction
    e = typeMsg('saveAsAction', self.saveAsAction, QAction)
    raise TypeError(e)

  def getExitAction(self, **kwargs) -> QAction:
    """Getter-function for exit action."""
    if self.exitAction is None:
      if kwargs.get('_recursion', False):
        raise RecursionError('Recursive call detected in getExitAction')
      self.createExitAction()
      return self.getExitAction(_recursion=True)
    if isinstance(self.exitAction, QAction):
      return self.exitAction
    e = typeMsg('exitAction', self.exitAction, QAction)
    raise TypeError(e)

  def createCutAction(self) -> None:
    """Creates an action to cut text."""
    icon = getIcon('cut')
    text = 'Cut'
    tooltip = 'Cut Selected Text'
    shortCut = QKeySequence('CTRL+X')
    parent = self.parent()
    self.cutAction = QAction(parent)
    self.cutAction.setText(text)
    self.cutAction.setIcon(icon)
    self.cutAction.setShortcut(shortCut)
    self.cutAction.setShortcutContext(Qt.ShortcutContext.ApplicationShortcut)
    self.cutAction.setToolTip(tooltip)
    self.cutAction.triggered.connect(self.cutFunc)
    self.getEditMenu().addAction(self.cutAction)

  def getCutAction(self, **kwargs) -> QAction:
    """Getter-function for cut action."""
    if self.cutAction is None:
      if kwargs.get('_recursion', False):
        raise RecursionError('Recursive call detected in getCutAction')
      self.createCutAction()
      return self.getCutAction(_recursion=True)
    if isinstance(self.cutAction, QAction):
      return self.cutAction
    e = typeMsg('cutAction', self.cutAction, QAction)
    raise TypeError(e)

  def createCopyAction(self) -> None:
    """Creates an action to copy text."""
    icon = getIcon('copy')
    text = 'Copy'
    tooltip = 'Copy Selected Text'
    shortCut = QKeySequence('CTRL+C')
    parent = self.parent()
    self.copyAction = QAction(parent)
    self.copyAction.setText(text)
    self.copyAction.setIcon(icon)
    self.copyAction.setShortcut(shortCut)
    self.copyAction.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.copyAction.setToolTip(tooltip)
    self.copyAction.triggered.connect(self.copyFunc)
    self.getEditMenu().addAction(self.copyAction)

  def getCopyAction(self, **kwargs) -> QAction:
    """Getter-function for copy action."""
    if self.copyAction is None:
      if kwargs.get('_recursion', False):
        raise RecursionError('Recursive call detected in getCopyAction')
      self.createCopyAction()
      return self.getCopyAction(_recursion=True)
    if isinstance(self.copyAction, QAction):
      return self.copyAction
    e = typeMsg('copyAction', self.copyAction, QAction)
    raise TypeError(e)

  def createPasteAction(self) -> None:
    """Creates an action to paste text."""
    icon = getIcon('paste')
    text = 'Paste'
    tooltip = 'Paste Text'
    shortCut = QKeySequence('CTRL+V')
    parent = self.parent()
    self.pasteAction = QAction(parent)
    self.pasteAction.setText(text)
    self.pasteAction.setIcon(icon)
    self.pasteAction.setShortcut(shortCut)
    self.pasteAction.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.pasteAction.setToolTip(tooltip)
    self.pasteAction.triggered.connect(self.pasteFunc)
    self.getEditMenu().addAction(self.pasteAction)

  def getPasteAction(self, **kwargs) -> QAction:
    """Getter-function for paste action."""
    if self.pasteAction is None:
      if kwargs.get('_recursion', False):
        raise RecursionError('Recursive call detected in getPasteAction')
      self.createPasteAction()
      return self.getPasteAction(_recursion=True)
    if isinstance(self.pasteAction, QAction):
      return self.pasteAction
    e = typeMsg('pasteAction', self.pasteAction, QAction)
    raise TypeError(e)

  def createUndoAction(self) -> None:
    """Creates an action for undo."""
    icon = getIcon('undo')
    text = 'Undo'
    tooltip = 'Undo Last Action'
    shortCut = QKeySequence('CTRL+Z')
    parent = self.parent()
    self.undoAction = QAction(parent)
    self.undoAction.setText(text)
    self.undoAction.setIcon(icon)
    self.undoAction.setShortcut(shortCut)
    self.undoAction.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.undoAction.setToolTip(tooltip)
    self.undoAction.triggered.connect(self.undoFunc)
    self.getEditMenu().addAction(self.undoAction)

  def getUndoAction(self, **kwargs) -> QAction:
    """Getter-function for undo action."""
    if self.undoAction is None:
      if kwargs.get('_recursion', False):
        raise RecursionError('Recursive call detected in getUndoAction')
      self.createUndoAction()
      return self.getUndoAction(_recursion=True)
    if isinstance(self.undoAction, QAction):
      return self.undoAction
    e = typeMsg('undoAction', self.undoAction, QAction)
    raise TypeError(e)

  def createRedoAction(self) -> None:
    """Creates an action for redo."""
    icon = getIcon('redo')
    text = 'Redo'
    tooltip = 'Redo Last Action'
    shortCut = QKeySequence('CTRL+Y')
    parent = self.parent()
    self.redoAction = QAction(parent)
    self.redoAction.setText(text)
    self.redoAction.setIcon(icon)
    self.redoAction.setShortcut(shortCut)
    self.redoAction.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.redoAction.setToolTip(tooltip)
    self.redoAction.triggered.connect(self.redoFunc)
    self.getEditMenu().addAction(self.redoAction)

  def getRedoAction(self, **kwargs) -> QAction:
    """Getter-function for redo action."""
    if self.redoAction is None:
      if kwargs.get('_recursion', False):
        raise RecursionError('Recursive call detected in getRedoAction')
      self.createRedoAction()
      return self.getRedoAction(_recursion=True)
    if isinstance(self.redoAction, QAction):
      return self.redoAction
    e = typeMsg('redoAction', self.redoAction, QAction)
    raise TypeError(e)

  def createAboutQtAction(self, **kwargs) -> None:
    """Creator function for 'about Qt' action"""
    icon = getIcon('about_qt')
    text = 'About Qt'
    tooltip = 'About Qt information dialog'
    shortCut = QKeySequence('F12')
    parent = self.parent()
    self.aboutQtAction = QAction(parent)
    self.aboutQtAction.setText(text)
    self.aboutQtAction.setIcon(icon)
    self.aboutQtAction.setShortcut(shortCut)
    self.aboutQtAction.setShortcutContext(
      Qt.ShortcutContext.ApplicationShortcut)
    self.aboutQtAction.setToolTip(tooltip)
    self.aboutQtAction.triggered.connect(self.aboutQtFunc)
    self.getHelpMenu().addAction(self.aboutQtAction)

  def getAboutQtAction(self, **kwargs) -> QAction:
    """Getter-function for 'about Qt' action"""
    if self.aboutQtAction is None:
      if kwargs.get('_recursion', False):
        raise RecursionError('Recursion call detected for About Qt action!')
      self.createAboutQtAction()
      return self.getAboutQtAction(_recursion=True)
    if isinstance(self.aboutQtAction, QAction):
      return self.aboutQtAction
    e = typeMsg('aboutQtAction', self.aboutQtAction, QAction)
    raise TypeError(e)

  @abstractmethod
  def initUI(self) -> None:
    """Initializes the UI by setting up widgets and layouts. Subclasses
    must implement this method."""

  def show(self) -> None:
    """Reimplementation invoking initUI, before parent show"""
    ic('BaseWindow.show')
    self.initUI()
    return QMainWindow.show(self)
