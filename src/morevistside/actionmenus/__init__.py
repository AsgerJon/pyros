"""The actionmenus module provides class for menu bars, menus and actions."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from ._icon_factory import getIcon, getPix, getFids
from ._abstract_menu import AbstractMenu
from ._action_factory import actionFactory
from ._files_menu import FilesMenu
from ._edit_menu import EditMenu
from ._help_menu import HelpMenu
from ._menu_bar import MenuBar
from ._status_bar import StatusBar