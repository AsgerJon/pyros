"""Main tester script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

from PySide6.QtWidgets import QApplication
from icecream import ic
from vistutils import maybe

from morevistside.actionmenus import getIcon
from morevistside.windows import MainWindow
from pyros import RosReg
from tester_class_01 import Point

ic.configureOutput(includeContext=True)


def tester00() -> None:
  """Hello world"""
  stuff = [os, sys, 'hello world', maybe]
  for item in stuff:
    print(item)


def tester01() -> None:
  """TypedField test"""
  p = Point(1, 2, 3)
  q = Point(4, 5, 6)
  print(p)
  print(q)

  for item in p:
    print(item)


def tester02() -> None:
  """RosNode"""


def tester03() -> None:
  """RosNode"""


def tester04() -> None:
  """Enum test"""
  for item in RosReg:
    print(item)


def tester05() -> None:
  """Test of topic listing"""
  app = QApplication(sys.argv)
  mainWindow = MainWindow()
  mainWindow.show()
  sys.exit(app.exec_())


def tester06() -> None:
  """Test af get icon"""
  names = ['new', 'open', 'risitas']
  for name in names:
    fid = getIcon(name)
    print(fid)


if __name__ == '__main__':
  tester05()
