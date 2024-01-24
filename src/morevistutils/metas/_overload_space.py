"""OverloadSpace provides a baseclass for use by custom metaclasses
implementing overloaded functions."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from vistutils import monoSpace

from morevistutils.metas import FlexSpace


class OverloadSpace(FlexSpace):
  """OverloadSpace provides a baseclass for use by custom metaclasses
  implementing overloaded functions."""

  def getNamespace(self, *args, **kwargs) -> dict:
    """Compiles the actual namespace to be used"""

  def __explicit_get_item__(self, key: str, ) -> Any:
    """All items in the dict are lists. The most recently added is
    returned by the basic item retrieval."""
    entry = dict.__getitem__(self, key)
    if isinstance(entry, list):
      return dict.__getitem__(self, key)[-1]
    e = """Expected all items in OverloadSpace instance to be of type 
    '%s', but received '%s' of type: '%s'!"""
    raise TypeError(monoSpace(e % (list, entry, type(entry))))

  def __setitem__(self, key: str, val: Any) -> None:
    if key not in self:
      return dict.__setitem__(self, key, [val])
    existing = dict.__getitem__(self, key)
    if not isinstance(existing, list):
      raise TypeError
    dict.__setitem__(self, key, [*existing, val])

  def __explicit_set_item__(self, key: str, Val: Any, old: Any) -> None:
    """Does nothing in this implementation"""

  def __explicit_del_item__(self, key: str, oldVal: Any) -> None:
    """Implementation"""
    dict.__delitem__(self, key)
