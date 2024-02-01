"""MainWindow subclasses LayoutWindow and provides the business logic."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import time

from PySide6.QtCore import QTimer, Qt

from morevistside.windows import LayoutWindow


class MainWindow(LayoutWindow):
  """MainWindow subclasses LayoutWindow and provides the business logic."""

  def __init__(self, *args, **kwargs) -> None:
    LayoutWindow.__init__(self, *args, **kwargs)
    self._timer = QTimer()
    self._timer.setInterval(1000)
    self._timer.setSingleShot(False)
    self._timer.setTimerType(Qt.TimerType.PreciseTimer)
    self._timer.timeout.connect(self._timeoutFunc)
    self._tic = time.time()
    self._timer.start()

  def _timeoutFunc(self) -> None:
    newText = '%04d' % int(time.time() - self._tic)
    self.clock = newText
