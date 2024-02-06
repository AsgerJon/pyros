"""Main tester script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import subprocess
import sys

from PySide6.QtCore import Signal
from PySide6.QtGui import QPainterPath, QImage
from PySide6.QtWidgets import QApplication, QWidget, QLabel
from icecream import ic
from vistutils import maybe, getProjectRoot
import shiboken6
from shiboken6.Shiboken import Object

from morevistside import shibokinator, parseHex
from morevistside.actionmenus import convertImage
from morevistside.windows import MainWindow
from tester_class_02 import SomeClass

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
  sys.exit(app.exec())


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
  lol = '#D0D0D0'
  print(parseHex(lol))
  print(sys.version_info)
  subprocess.run('mamba env export > environment.yml', shell=True, )


def tester07() -> None:
  """YOLO"""
  someInstance = SomeClass()
  namespace = getattr(SomeClass, '__namespace_object__')
  for (key, val) in namespace.getAnnotations().items():
    print(key, val)


def tester08() -> None:
  """Image conversion test"""
  root = getProjectRoot()
  there = os.path.join(root, 'src', 'morevistside', 'actionmenus', 'icons')
  srcName = 'help-icon-png-0.jpg'
  filePath = os.path.join(there, srcName)
  outPath = os.path.join(there, 'help_menu.png')
  convertImage(filePath, outPath, 'PNG')


if __name__ == '__main__':
  tester01()
