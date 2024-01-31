"""Main tester script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

from PySide6.QtWidgets import QApplication, QMainWindow
# from PySide6.QtWidgets import QApplication
from icecream import ic
from vistutils import maybe

from morevistside.windows import MainWindow


def tester00() -> None:
  """Hello world"""
  stuff = [os, sys, 'hello world', maybe]
  for item in stuff:
    print(item)


def tester01() -> None:
  """lmao"""
  ic(sys.argv)
  # raise RuntimeError
  app = QApplication()
  mainWindow = MainWindow()
  mainWindow.show()
  sys.exit(app.exec_())


if __name__ == '__main__':
  tester01()
