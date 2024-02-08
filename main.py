"""Main tester script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

from PySide6.QtGui import QPainterPath
from PySide6.QtWidgets import QApplication, QWidget
from icecream import ic
from rospy import Subscriber
from vistutils import maybe, getProjectRoot
import shiboken6
from shiboken6.Shiboken import Object

from morevistside import shibokinator
from morevistside.actionmenus import convertImage, getFids
from morevistside.paintmelike import PlotWindow
from morevistside.windows import MainWindow

ic.configureOutput(includeContext=True)


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


if __name__ == '__main__':
  tester01()
