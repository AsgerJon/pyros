"""Main tester script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

from PySide6.QtGui import QPainterPath
from PySide6.QtWidgets import QApplication, QWidget
from icecream import ic
from vistutils import maybe
import shiboken6
from shiboken6.Shiboken import Object

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


if __name__ == '__main__':
  tester01()
