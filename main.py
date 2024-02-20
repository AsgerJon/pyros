"""Main tester script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

from PySide6.QtWidgets import QApplication
from icecream import ic
from vistutils.parse import maybe
import shiboken6

from tester_class_04 import SomeClass
from tester_class_05 import TestWindow

ic.configureOutput(includeContext=True)


def tester00() -> None:
  """Hello world"""
  stuff = [os, sys, 'hello world', maybe, shiboken6]
  for item in stuff:
    print(item)


def tester01() -> None:
  """lmao"""
  # app = QApplication()
  # ic()
  # mainWindow = MainWindow()
  # ic()
  # mainWindow.show()
  # ic()
  # sys.exit(app.exec())


def tester02() -> None:
  """YOLO"""
  bla = SomeClass('blabla', lmao=True, _root='f... da police')
  ic(bla.__new_args__)
  ic(bla.__init_args__)
  ic(bla.__new_kwargs__)
  ic(bla.__init_kwargs__)


def tester03() -> None:
  """Test window"""
  app = QApplication()
  mainWindow = TestWindow()
  mainWindow.show()
  app.exec()
  for (key, val) in mainWindow.__dict__.items():
    print(key, type(val))


if __name__ == '__main__':
  tester03()
