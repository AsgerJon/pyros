"""Main tester script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys
from pyros import MetaRos
from icecream import ic
from vistutils import maybe

from pyros import RosNode, RosReg
from tester_class_01 import Point
from tester_class_02 import Parent, Child

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
  rosNode = RosNode()


def tester03() -> None:
  """Inheritance"""
  parent = Parent()
  child = Child()
  child = object.__new__(Child)
  parent.someMethod()
  parent.otherMethod()
  child.someMethod()
  child.otherMethod()
  ic(Parent.someMethod is Child.someMethod)
  ic(Parent.otherMethod is Child.otherMethod)
  ic(Parent.staticMethod is Child.staticMethod)

  ic(parent.someMethod is child.someMethod)
  ic(parent.someMethod.__func__ is child.someMethod.__func__)
  ic(parent.otherMethod is child.otherMethod)
  ic(parent.otherMethod.__func__ is child.otherMethod.__func__)
  ic(parent.staticMethod is child.staticMethod)


def tester04() -> None:
  """Enum test"""
  for item in RosReg:
    print(item)


def tester05() -> None:
  """Test of topic listing"""
  node = RosNode('LOL')


if __name__ == '__main__':
  tester05()
