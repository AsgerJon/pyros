"""lmao dispatch"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import logging
import os
import subprocess
import time
from string import ascii_letters, digits, punctuation
from typing import Never

from PySide6.QtCore import QThread, Signal, QTimer, Qt, Slot
from icecream import ic
from vistutils import monoSpace, maybe

from morevistside import parseParent
from morevistutils.fields import Field
from morevistutils.waitaminute import typeMsg

ic.configureOutput(includeContext=True)


class Dispatch(QThread):
  """Class to manage and interact with a subprocess."""

  __time_out_fallback__ = 10000

  __terminal_commands__ = None
  __terminal_process__ = None
  __instance_creation_time__ = None
  __timeout_limit__ = None
  __timeout_guard__ = None

  command = Field()
  process = Field()
  age = Field()
  timeoutLimit = Field()
  timeoutGuard = Field()
  standardOut = Signal(str)
  standardErr = Signal(str)
  exit = Signal(int)

  @command.GET
  def _getCommand(self) -> str:
    """Getter for command"""
    if self.__terminal_commands__ is None:
      raise AttributeError
    if isinstance(self.__terminal_commands__, str):
      return self.__terminal_commands__
    e = typeMsg('__terminal_commands__', self.__terminal_commands__, str)
    raise TypeError(e)

  @command.SET
  def _setCommand(self, value: str) -> None:
    """Setter for command"""
    if self.__terminal_commands__ is None:
      self.__terminal_commands__ = self._collectCommands(value)
    else:
      e = """The command is already set and is write-once<!"""
      raise TypeError(e)

  @command.DEL
  def _delCommand(self) -> Never:
    """Deleter for command"""
    e = """The command is read-only!"""
    raise TypeError(e)

  def _createProcess(self) -> None:
    """Create the process"""
    if self.__terminal_process__ is not None:
      if isinstance(self.__terminal_process__, subprocess.Popen):
        if self.__terminal_process__.poll() is None:
          logging.warning(monoSpace("""The _createProcess method was 
          called with an existing process still running. This will now 
          be closed and a new one generated."""))
          self.__terminal_process__.kill()
          del self.__terminal_process__
    self.__terminal_process__ = subprocess.Popen(
      self.command,
      stdout=subprocess.PIPE,
      stderr=subprocess.PIPE,
      shell=True,
      text=True)

  @process.GET
  def _getProcess(self, **kwargs) -> subprocess.Popen:
    """Getter for process"""
    if self.__terminal_process__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createProcess()
      return self._getProcess(_recursion=True)
    if isinstance(self.__terminal_process__, subprocess.Popen):
      return self.__terminal_process__
    e = typeMsg('__terminal_process__', self.__terminal_process__,
                subprocess.Popen)
    raise TypeError(e)

  @process.SET
  def _setProcess(self, *_) -> Never:
    """Illegal setter function"""
    raise TypeError('The process is read only!')

  @process.DEL
  def _delProcess(self, ) -> Never:
    """Illegal deleter function"""
    raise TypeError('The process is read only!')

  @age.GET
  def _getAge(self, ) -> float:
    """Getter for age"""
    return time.time() - self.__instance_creation_time__

  @age.SET
  def _setAge(self, *_) -> Never:
    """Illegal setter function"""
    raise TypeError('The age is read only!')

  @age.DEL
  def _delAge(self, ) -> Never:
    """Illegal deleter function"""
    raise TypeError('The age is read only!')

  @timeoutLimit.GET
  def _getTimeoutLimit(self, **kwargs) -> int:
    """Getter for timeout limit"""
    if self.__timeout_limit__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      explicitLimit = self.__timeout_limit__
      envLimit = os.environ.get('TIMEOUT_LIMIT', None)
      fallback = self.__time_out_fallback__
      self.__timeout_limit__ = maybe(explicitLimit, envLimit, fallback)
      return self._getTimeoutLimit(_recursion=True)
    if isinstance(self.__timeout_limit__, int):
      return self.__timeout_limit__
    e = typeMsg('__timeout_limit__', self.__timeout_limit__, int)
    raise TypeError(e)

  @timeoutLimit.SET
  def _setTimeoutLimit(self, value: int) -> None:
    """Setter for timeout limit"""
    if isinstance(value, int):
      self.__timeout_limit__ = value
    elif (value - round(value)) ** 2 < 1e-08:
      self.__timeout_limit__ = int(round(value))
    else:
      e = typeMsg('value', value, int)
      raise TypeError(e)

  @timeoutLimit.DEL
  def _delTimeoutLimit(self, ) -> Never:
    """Illegal deleter function"""
    raise TypeError('The timeout limit is protected from deletion!')

  def _createTimeoutGuard(self) -> None:
    """Create the timeout guard"""
    self.__timeout_guard__ = QTimer()
    if self.timeoutLimit > 10000:
      self.__timeout_guard__.setTimerType(Qt.TimerType.VeryCoarseTimer)
    elif self.timeoutLimit > 1000:
      self.__timeout_guard__.setTimerType(Qt.TimerType.CoarseTimer)
    else:
      self.__timeout_guard__.setTimerType(Qt.TimerType.PreciseTimer)
    self.__timeout_guard__.setInterval(self.__timeout_limit__)
    self.__timeout_guard__.setSingleShot(True)

  @timeoutGuard.GET
  def _getTimeoutGuard(self, **kwargs) -> QTimer:
    """Getter for timeout guard"""
    if self.__timeout_guard__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      self._createTimeoutGuard()
      return self._getTimeoutGuard(_recursion=True)
    if isinstance(self.__timeout_guard__, QTimer):
      return self.__timeout_guard__
    e = typeMsg('__timeout_guard__', self.__timeout_guard__, QTimer)
    raise TypeError(e)

  @timeoutGuard.SET
  def _setTimeoutGuard(self, *_) -> Never:
    """Illegal setter function"""
    raise TypeError('The timeout guard is read only!')

  @timeoutGuard.DEL
  def _delTimeoutGuard(self, ) -> Never:
    """Illegal deleter function"""
    raise TypeError('The timeout guard is read only!')

  def __init__(self, *args, **kwargs: str) -> None:
    """
    Initialize Dispatch with a command and optional environment variables.

    :param command: The command to be executed.
    :param kwargs: Optional keyword arguments to be set as environment
    variables.
    """
    parent = parseParent(*args)
    QThread.__init__(self, parent)
    self.command = self._collectCommands(*args)
    self._updateEnv(**kwargs)

  @staticmethod
  def _collectCommands(*args: str) -> str:
    """
    Parse the command string into a list of strings.

    :param args: The command string.
    :return: The parsed command.
    """
    out = []
    for arg in args:
      if isinstance(arg, str):
        out.append(arg)
      elif isinstance(arg, bytes):
        out.append(arg.decode())
      elif isinstance(arg, (list, tuple)):
        out.extend(arg)
      elif isinstance(arg, (int, float)):
        out.append(str(arg))
    return ' '.join(out)

  @staticmethod
  def _updateEnv(**kwargs: str) -> None:
    """Set environment variables"""
    for key, value in kwargs.items():
      os.environ[key] = str(value)

  @staticmethod
  def _printableFilter(value: bytes) -> list[str]:
    """Filter for printable characters"""
    allowedChars = [ascii_letters, digits, punctuation, ' ']
    lines = value.decode().split(os.linesep)
    out = []
    for item in lines:
      line = []
      for char in item:
        if char in allowedChars:
          line.append(char)
      out.append(''.join(line))
    return out

  def run(self) -> None:
    """Implementation of the this method is left as an exercise for the
    copilot running chatGPT-4."""
    while self.process.poll() is None:
      out = self.process.stdout.readline() or ' '
      self.standardOut.emit(out)
    return self.exit.emit(self.process.poll())

  def pid(self) -> int:
    """Implementation of the this method is left as an exercise for the
    copilot running chatGPT-4."""
    return self.process.pid

  def kill(self) -> None:
    """Implementation of the this method is left as an exercise for the
    copilot running chatGPT-4."""
    self.timeoutGuard.stop()
    if self.process.poll() is None:
      self.process.kill()
    raise ValueError('The process is not running!')

  def isRunning(self) -> bool:
    """Implementation of the this method is left as an exercise for the
    copilot running chatGPT-4."""
    return True if self.process.poll() is None else False

  def __bool__(self) -> bool:
    """True processes are those that are running"""
    return self.isRunning()

  def timeoutHandle(self) -> None:
    """Implementation of the this method is left as an exercise for the
    copilot running chatGPT-4."""
    self.kill()
    raise TimeoutError('The process timed out!')
