"""LayoutWindow subclasses BaseWindow and provides the layout management
for the main application window. This class is responsible for creating
widgets and layouts in the main application window. It is not responsible
for connecting any signals and slots to and from the visual elements. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QMargins
from PySide6.QtWidgets import QGridLayout, QWidget
from vistutils.fields import Field

from morevistside.widgets import CornerWidget, HorizontalBanner, \
  VerticalBanner
from morevistside.windows import BaseWindow
from morevistutils.fields import TypedField, SpecialField


class LayoutWindow(BaseWindow):
  """LayoutWindow subclasses BaseWindow and provides the layout management
  for the main application window. This class is responsible for creating
  widgets and layouts in the main application window. It is not responsible
  for connecting any signals and slots to and from the visual elements. """

  bannerColor = Field()
  bannerBrush = Field()

  margins = Field()

  topLeft = SpecialField(CornerWidget)
  topRight = SpecialField(CornerWidget)
  bottomLeft = SpecialField(CornerWidget)
  bottomRight = SpecialField(CornerWidget)

  top = SpecialField(HorizontalBanner)
  bottom = SpecialField(HorizontalBanner)
  left = SpecialField(VerticalBanner)
  right = SpecialField(VerticalBanner)

  layout = SpecialField(QGridLayout)
  baseWidget = SpecialField(QWidget)

  @layout.CREATE
  def _createLayout(self, *args, **kwargs) -> QGridLayout:
    """Creator function for grid layout"""
    return QGridLayout()

  @baseWidget.CREATE
  def _createBaseWidget(self, *args, **kwargs) -> QWidget:
    """Creator function for base widget"""
    return QWidget()

  @margins.GET
  def _createMargins(self) -> QMargins:
    """Getter-function for the margins"""
    return QMargins(48, 32, 48, 32)

  @top.CREATE
  def _createTopBanner(self, *args, **kwargs) -> HorizontalBanner:
    """Creator function for top banner"""
    return HorizontalBanner(self, self.margins.top, self.bannerColor)

  @right.CREATE
  def _createRightBanner(self, *args, **kwargs) -> VerticalBanner:
    """Creator function for right banner"""
    return VerticalBanner(self, self.margins.right, self.bannerColor)

  @bottom.CREATE
  def _createBottomBanner(self, *args, **kwargs) -> HorizontalBanner:
    """Creator function for bottom banner"""
    return HorizontalBanner(self, self.margins.bottom, self.bannerColor)

  @left.CREATE
  def _createBottomBanner(self, *args, **kwargs) -> VerticalBanner:
    """Creator function for left banner"""
    return VerticalBanner(self, self.margins.left, self.bannerColor)

  @topLeft.CREATE
  def _createTopLeft(self, *args, **kwargs) -> CornerWidget:
    """Creator function for top left corner widget"""
    left, top = self.margins.left, self.margins.top
    corner = CornerWidget(left, top, self.bannerColor)
    return corner

  @topRight.CREATE
  def _createTopRight(self, *args, **kwargs) -> CornerWidget:
    """Creator function for top right corner widget"""
    right, top = self.margins.right, self.margins.top
    corner = CornerWidget(right, top, self.bannerColor)
    return corner

  @bottomLeft.CREATE
  def _createBottomLeft(self, *args, **kwargs) -> CornerWidget:
    """Creator function for bottom left corner widget"""
    left, bottom = self.margins.left, self.margins.bottom
    corner = CornerWidget(left, bottom, self.bannerColor)
    return corner

  @bottomRight.CREATE
  def _createBottomRight(self, *args, **kwargs) -> CornerWidget:
    """Creator function for bottom right corner widget"""
    right, bottom = self.margins.right, self.margins.bottom
    corner = CornerWidget(right, bottom, self.bannerColor)
    return corner

  def __init__(self, *args, **kwargs) -> None:
    BaseWindow.__init__(self, *args, **kwargs)

  def initUI(self, ) -> None:
    """Sets up the widgets"""
    self.layout.addWidget(self.topLeft, 0, 0, 1, 1)
    self.layout.addWidget(self.top, 0, 1, 1, 1)
    self.layout.addWidget(self.topRight, 0, 2, 1, 1)

    self.layout.addWidget(self.left, 1, 0, 1, 1)
    self.layout.addWidget(self.right, 1, 2, 1, 1)

    self.layout.addWidget(self.bottomLeft, 2, 0, 1, 1)
    self.layout.addWidget(self.bottom, 2, 1, 1, 1)
    self.layout.addWidget(self.bottomRight, 2, 2, 1, 1)

    self.baseWidget.setLayout(self.layout)
    self.setCentralWidget(self.baseWidget)
