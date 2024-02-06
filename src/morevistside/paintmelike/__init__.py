"""The 'paintMeLike' module provides a descriptor class for widgets and
windows in the Qt framework. These descriptors implement a value retrieved
by the traditional __get__, but which is also reflected in an optional
painting operation. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ._abstract_stencil import AbstractStencil
from ._color_descriptor import ColorDescriptor
from ._solid_background import SolidBackground
from ._text_descriptor import TextDescriptor
from ._font_descriptor import FontDescriptor
from ._text_label import TextLabel
