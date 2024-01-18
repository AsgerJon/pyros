"""The readenv module provides functionality required for reading .env
files. The module assumes that the root folder may be identified by
containing files: LICENSE, README.md and .env.example."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ._read_env_exception import ReadEnvException
