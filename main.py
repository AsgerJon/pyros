"""Main tester script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

from vistutils import maybe

from pyros import RosNode
from tester_class_01 import Point


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
  rosNode = RosNode()


if __name__ == '__main__':
  tester02()
