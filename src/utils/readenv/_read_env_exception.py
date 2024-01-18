"""The ReadEnvException provides a custom exception class for the readenv
module. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations


class ReadEnvException(Exception):
  """ReadEnvException provides a custom exception class for the readenv
  module. """

  def __init__(self, baseExc: Exception, msg: str = None) -> None:
    if msg is None:
      e = """readenv module failed because: %s"""
      msg = e % baseExc
    Exception.__init__(self, msg)
