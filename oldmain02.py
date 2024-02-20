"""Main tester script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import _io
import os
import subprocess
import sys

from PySide6.QtCore import QThread
from PySide6.QtWidgets import QApplication
from icecream import ic
from rospy import get_published_topics, init_node
from vistutils.parse import maybe
import shiboken6

from morevistside.windows import MainWindow
from tester_class_02 import ShareField

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
  """Fields sharing getter"""
  lol = ShareField()
  print(lol.a)
  print(lol.b)
  print(lol)


def tester03() -> None:
  """Of what type is QThread"""
  print(QThread)
  print(type(QThread))


def tester04() -> None:
  """Testing how responses when ROS is not initialized."""

  ic(os.environ['ROS_MASTER_URI'])
  topics = get_published_topics()
  ic(topics)
  for topicName, msgType in get_published_topics():
    ic(topicName, msgType)


def tester05() -> None:
  """Testing the subprocess module"""

  init_node('yolo', anonymous=False)


def tester06() -> None:
  """Testing get icons """
  lol = subprocess.Popen(['echo'],
                         stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE, )
  for (key, val) in lol.__dict__.items():
    break
    print(key, type(val))
  print(lol.stdout)
  print(type(lol.stdout))

  for (key, val) in lol.stdout.__dict__.items():
    print(key, type(val))

  for item in dir(lol.stdout):
    print(item)

  print(lol.stdout.readline())


def tester07() -> None:
  """lmao"""

  for item in dir(_io.BufferedReader):
    print(item)


def tester08() -> None:
  """Testing Custom App"""


if __name__ == '__main__':
  tester01()
