"""Res subclasses CompletedProcess from the subprocess module exposing its
content in a more flexible manner"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from subprocess import CompletedProcess, PIPE, run
from typing import Self, Any

from morevistutils.fields import CustomField, apply


class MetaRes(type):
  """Cheeky"""

  def __instancecheck__(cls, instance) -> bool:
    if isinstance(instance, CompletedProcess):
      return True
    return type.__instancecheck__(cls, instance)


class Res(metaclass=MetaRes):
  """Res subclasses CompletedProcess from the subprocess module exposing its
  content in a more flexible manner. """

  out = CustomField()
  err = CustomField()
  cmd = CustomField()
  ret = CustomField()

  @apply('out', 'get')
  def getOut(self) -> str:
    """Getter-function for stdout"""
    res = self.__completed_process__
    return res.stdout.decode('utf-8')

  @apply('err', 'get')
  def getErr(self) -> str:
    """Getter-function for stderr"""
    res = self.__completed_process__
    return res.stderr.decode('utf-8')

  @apply('cmd', 'get')
  def getCmd(self) -> str:
    """Getter-function for stdout"""
    res = self.__completed_process__
    out = res.args
    if isinstance(out, str):
      return out
    if isinstance(out, bytes):
      return out.decode('utf-8')
    if isinstance(out, (list, tuple)):
      out = [*out, ]
    out2 = []
    for arg in out:
      if isinstance(arg, str):
        out2.append(arg)
      if isinstance(arg, bytes):
        out2.append(arg.decode('utf-8'))
    return ' '.join(out2)

  @apply('ret', 'get')
  def getRet(self) -> int:
    """Getter-function for stdout"""
    res = self.__completed_process__
    return int(res.returncode)

  def __init__(self, *args, **kwargs) -> None:
    cmd = ' '.join([arg for arg in args if isinstance(arg, str)])
    self.__completed_process__ = run(
      cmd, shell=True, stdout=PIPE, stderr=PIPE)

  def __int__(self, ) -> int:
    """Return code"""
    return self.__completed_process__.returncode

  def __str__(self, ) -> str:
    """String representation"""
    if not int(self):
      return self.out
