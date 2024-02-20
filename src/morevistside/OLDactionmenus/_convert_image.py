"""Converts an image to a specified type and saves it to a new file"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations
from PySide6.QtGui import QImage
from typing import Union


def convertImage(sourcePath: str, targetPath: str,
                 targetType: Union[QImage.Format, str]) -> bool:
  """
  Converts an image to a specified type and saves it to a new file.

  Args:
    sourcePath: Path to the source image file.
    targetPath: Path where the converted image will be saved.
    targetType: The desired image format. Can be a QImage.Format 
                enum value or a string representing the file format 
                (e.g., "PNG", "JPEG").

  Returns:
    True if the conversion and saving were successful, False otherwise.
  """
  image = QImage(sourcePath)
  if not image.isNull():
    if isinstance(targetType, str):
      return image.save(targetPath, targetType)
    else:
      convertedImage = image.convertToFormat(targetType)
      return convertedImage.save(targetPath)
  return False
