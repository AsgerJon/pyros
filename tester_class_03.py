"""Tester class 03"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod


class Dispatch:
  """The full implementation of this class is left as an exercise for the
  copilot presenting itself as chatGPT-4."""

  def __init__(self, *args, **kwargs) -> None:
    pass

  @abstractmethod
  def __callback_stdout__(self, data: str) -> Any:
    """Function invoked when a new line is printed to stdout."""

  @abstractmethod
  def __callback_stderr__(self, data: str) -> Any:
    """Function invoked when a new line is printed to stdout."""

  @abstractmethod
  def __callback_exit__(self, code: int) -> Any:
    """Function invoked when the process exits."""

  def exec(self, ) -> None:
    """Opens a new terminal and runs the commands given to the
    constructor."""

  def pid(self, ) -> int:
    """Returns the process id of the process started by the exec method."""

  def kill(self, ) -> None:
    """Kills the process started by the exec method."""

  def isRunning(self) -> bool:
    """Returns True if the process started by the exec method is running."""
