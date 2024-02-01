"""Main tester script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

from PySide6.QtWidgets import QApplication, QMainWindow, QWidget
from icecream import ic
from vistutils import maybe, getProjectRoot

from morevistside.windows import MainWindow

ic.configureOutput(includeContext=True)


def tester00() -> None:
  """Hello world"""
  stuff = [os, sys, 'hello world', maybe]
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


if __name__ == '__main__':
  tester01()
