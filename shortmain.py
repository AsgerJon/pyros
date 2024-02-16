"""Short tester"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from morevistutils import getIconsPath, Debug


def tester06() -> None:
  """Testing get icons """
  with Debug():
    print(getIconsPath())


if __name__ == '__main__':
  tester06()
