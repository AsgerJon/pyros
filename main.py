"""Main tester script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

from PySide6.QtGui import QPainterPath
from PySide6.QtWidgets import QApplication, QWidget
from icecream import ic
from numpy import nan, full, array, isnan
from pyperclip import copy, paste
from rospy import Subscriber
from vistutils import maybe, getProjectRoot, monoSpace
import shiboken6
from shiboken6.Shiboken import Object
from vistutils.fields import apply

from morevistside import shibokinator
from morevistside.actionmenus import convertImage, getFids
from morevistside.paintmelike import PlotWindow
from morevistside.widgets import Space, Point
from morevistside.windows import MainWindow
from morevistutils import applyEnv

ic.configureOutput(includeContext=True)

applyEnv()


def tester00() -> None:
  """Hello world"""
  stuff = [os, sys, 'hello world', maybe, shiboken6]
  for item in stuff:
    print(item)


def tester01() -> None:
  """lmao"""

  app = QApplication()
  mainWindow = MainWindow()
  mainWindow.show()
  app.exec()
  # sys.exit(app.exec())


def tester02() -> None:
  """Shiboken"""

  mcls = type(QWidget)
  ic(mcls.__qualname__)
  ic(mcls.__class__.__qualname__)
  ic(mcls.__class__.__class__.__qualname__)

  mcls = type(QWidget)
  ic(sys.modules[mcls.__module__])
  ic(mcls.__module__)
  ic(mcls.__class__.__module__)
  ic(mcls.__class__.__class__.__module__)


def tester03() -> None:
  """Shiboken metaclass"""
  for (key, val) in shiboken6.Shiboken.__dict__.items():
    print(key, )

  ic(shiboken6.Shiboken.Object)
  ic(type(shiboken6.Shiboken.Object))
  ic(type(type(shiboken6.Shiboken.Object)))

  ic(isinstance(QPainterPath, shiboken6.Shiboken.Object.__class__))
  ic(shiboken6.Shiboken.Object.__class__)


def tester04() -> None:
  """lmao"""


def tester05() -> None:
  """yolo"""
  a = shibokinator(QWidget)
  print(a)
  b, c = shibokinator(QWidget)
  print(b, c)


def tester06() -> None:
  """yolo"""


def tester07() -> None:
  """YOLO"""


def tester08() -> None:
  """Image conversion test"""
  root = getProjectRoot()
  there = os.path.join(root, 'src', 'morevistside', 'actionmenus', 'icons')
  srcName = 'risitas.png'
  filePath = os.path.join(there, srcName)
  outPath = os.path.join(there, 'risitas.png')
  convertImage(filePath, outPath, 'PNG')


def tester09() -> None:
  """subscriber test"""

  data = getFids()

  for (key, val) in data.items():
    if os.path.exists(val):
      print(key, val)


def tester10() -> None:
  """ Fill sequentially"""

  sections = []
  bases = ["""self.debugAction%02d = QAction()"""]
  bases.append(
    """self.debugAction%02d.triggered.connect(self.debugFunc%02d)""")
  docString = lambda arg: '\"\"\"%s\"\"\"' % str(arg)
  bases.append(
    """<tab>def debugAction%0d2(self, ) -> None:<br>
    <tab><tab><doc><br><br>
    """
  )
  createAction = """<tab><tab>self.debugAction%02d = QAction()"""
  connectAction = """<tab><tab>self.debugAction%02d.triggered.connect(
  self.debugFunc%02d)"""
  funcAction = """<tab>def debugFunc%02d(self, ) -> None:<br>
    <tab><tab><doc><br>"""

  createCode = '\n'.join([monoSpace(createAction % i) for i in range(1, 13)])
  connectCode = '\n'.join([monoSpace(connectAction % (i, i)) for i in
                           range(1, 13)])
  funcLines = [funcAction % i for i in range(1, 13)]
  trip = '\"\"\"'
  baseDoc = """%sDebug function %%02d%s""" % (trip, trip)
  funcLines = [arg.replace('<doc>', baseDoc % (i + 1,)) for (i, arg) in
               enumerate(funcLines)]
  funcLines = [monoSpace(line) for line in funcLines]
  funcCode = '\n'.join(funcLines)

  code = '\n'.join([createCode, connectCode, funcCode])
  copy(code)
  print(code)


def tester11() -> None:
  """Code generation"""
  base = """
      self.newAction = QAction()<br>
    self.openAction = QAction()<br>
    self.saveAction = QAction()<br>
    self.saveAsAction = QAction()<br>
    self.cutAction = QAction()<br>
    self.copyAction = QAction()<br>
    self.pasteAction = QAction()<br>
    self.undoAction = QAction()<br>
    self.redoAction = QAction()<br>
  """
  base = monoSpace(base)
  lines = base.split('\n')
  lines = [line.strip() for line in lines]
  lines = [line for line in lines if line]
  lines = [line.replace('self.', '') for line in lines]
  lines = [line.replace('Action = QAction()', '') for line in lines]
  lines = [line.strip() for line in lines]
  trip = '\"\"\"'

  def connectLine(name: str) -> str:
    """Creates the code connect named action to named function"""
    base2 = """self.%sAction.triggered.connect(self.%sFunc)"""
    return base2 % (name, name)

  def triggerFunc(name: str) -> str:
    """Creates the code defining the triggered function"""
    name = monoSpace(name)
    base3 = """<tab>def %sFunc(self, ) -> None:<br>
    <tab><tab><trip>Docstring for %s action<trip><br>
    """
    return monoSpace(base3 % (name, name))

  lines = [triggerFunc(line) for line in lines]
  lines = [line.replace('<trip>', trip) for line in lines]

  code = '\n'.join(lines)
  copy(code)
  print(code)


def tester12() -> None:
  """More codegen"""
  base = paste()

  lines = [base.replace('01', '%02d' % i) for i in range(1, 13)]
  lines = [line.replace('F1', '%d' % i) for (i, line) in enumerate(lines)]
  code = '\n\n'.join(lines)
  copy(code)


def tester13() -> None:
  """More codegen"""
  base = paste()

  copy(base.replace('QAction()', 'None'))


def tester14() -> None:
  """Numpy test"""
  bla = full((16,), nan).tolist()
  print(bla)
  print([type(item) for item in bla])
  print([item == item for item in bla])

  a = array((0, nan, 2, 3))
  b = array((nan, 1, 2, 3))

  print(a[~isnan(a) * ~isnan(b)])


def tester15() -> None:
  """Test of mapping """
  tenSpace = Space(0, 10, 0, 10)
  bla = Space(8, 16, 8, 16)
  p = Point(7, 5)
  print(p)
  print(tenSpace.x0)
  print(tenSpace.x1)
  print(tenSpace.t0)
  print(tenSpace.t1)
  print(bla)


if __name__ == '__main__':
  tester01()
