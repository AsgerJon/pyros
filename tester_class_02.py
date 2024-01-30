"""TESTER"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations


class Parent:

  @staticmethod
  def staticMethod() -> None:
    print('static method')

  def someMethod(self) -> None:
    print('parent method')

  def otherMethod(self) -> None:
    print('parent other method')


class Child(Parent):

  def otherMethod(self) -> None:
    print('child other method')
