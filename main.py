"""Main tester script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

from PySide6.QtWidgets import QApplication
from icecream import ic
from vistutils import maybe
import shiboken6

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
  # app.exec()
  sys.exit(app.exec())


if __name__ == '__main__':
  tester01()
