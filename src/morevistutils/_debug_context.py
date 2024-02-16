"""DebugContext provides a context manager for debugging purposes."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys
import logging
from icecream import ic
from typing import Never

from morevistutils.fields import Field

from morevistutils import getProjectRoot
from morevistutils.waitaminute import typeMsg

ic.configureOutput(includeContext=True)


class Debug(logging.StreamHandler):
  """Debug class"""

  __log_file_name__ = 'latest.log'
  __root_handlers__ = None

  logFilePath = Field()

  @logFilePath.GET
  def _getLogFilePath(self, ) -> str:
    """Getter-function for log file path"""
    root = getProjectRoot()
    name = 'debug.log'
    if hasattr(self, '__log_file_name__'):
      name = getattr(self, '__log_file_name__')
      if not isinstance(name, str):
        e = typeMsg('__log_file_name__', name, str)
        raise TypeError(e)
    return os.path.join(root, name)

  @logFilePath.SET
  def _setLogFilePath(self, *_) -> Never:
    """Illegal setter function"""
    raise TypeError('The log file path is read only!')

  @logFilePath.DEL
  def _delLogFilePath(self, ) -> Never:
    """Illegal deleter function"""
    raise TypeError('The log file path is read only!')

  def __enter__(self) -> None:
    """Enter"""
    os.environ['DEBUG_ENV'] = 'True'
    logging.root.setLevel(logging.INFO)
    self.__old_handlers__ = logging.root.handlers
    logging.root.handlers = [self]
    # logging.basicConfig(filename=self.logFilePath, level=logging.DEBUG)
    # self.stdoutHandler = logging.StreamHandler(sys.stdout)
    # logging.getLogger().addHandler(self.stdoutHandler)
    # logging.debug('Entering debug environment')

  def __exit__(self, excType, excVal, excTb) -> None:
    """Exit"""
    logging.debug('Exiting debug environment')
    logging.debug('excType: %s', excType)
    logging.debug('excVal: %s', excVal)
    logging.debug('excTb: %s', excTb)
    os.environ['DEBUG_ENV'] = 'False'

  def emit(self, *args, **kwargs) -> None:
    """Log handle"""
    print(77 * '*')
    print('Received log message')
    print('%d positional arguments and %d keyword arguments' % (
      len(args), len(kwargs)))
    print('Positional arguments:')
    for arg in args:
      print('  %s' % arg)
    print('Keyword arguments:')
    for key, val in kwargs.items():
      print('  key: %s: type: %s' % (key, type(val)))
      print('    %s' % str(val))
