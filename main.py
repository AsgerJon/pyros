"""Main tester script"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys
from random import choice, shuffle

from vistutils import maybe

from pyros import RosMaster, Talker


# from dispatcher import Res


def tester00() -> None:
  """Hello world"""
  stuff = [os, sys, 'hello world']
  for item in stuff:
    print(item)


def tester01() -> None:
  """Maybe?"""
  some = [0, 0.0, 0.0 + 0.0 * 1j, list(), dict(), set(), '', (), b'']
  bla = ([*[None for _ in range(5)], choice(some)])
  shuffle(bla)
  print(maybe(bla))


def tester02() -> None:
  """RosMaster again"""
  rosMaster = RosMaster()
  print(rosMaster.URI)
  print(rosMaster.machine)
  for node in rosMaster:
    print(node)


def tester03() -> None:
  """Cat tester"""


def tester04() -> None:
  """Res tester"""
  talker = Talker()
  talker.run()


if __name__ == '__main__':
  tester04()
