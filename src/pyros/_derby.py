"""Derby compares the speed of different data structures. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os.path
import sys
from pyperclip import copy

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
copy(sys.path[-1])
print(sys.path)
import timeit
import numpy as np

from pyros import TurtleRoll, DataRoll

stuff = [np, DataRoll, TurtleRoll]

# Parameters for the test
num_items = 100000  # Number of items to append
buffer_capacity = 10000  # Capacity of the circular buffer

# Setup code for Python list-based implementation
setup_python = f'''
from __main__ import TurtleRoll
cb = TurtleRoll({buffer_capacity})
'''

# Setup code for Numba + NumPy implementation
setup_numba = f'''
from __main__ import DataRoll
cb = DataRoll({buffer_capacity}, dtype=np.float64)
'''

# Test code (same for both to ensure a fair comparison)
test_code = f'''
for i in range({num_items}):
    cb.append(i)
'''

# Timing the Python list-based implementation
time_python = timeit.timeit(stmt=test_code, setup=setup_python, number=1)
print(f"Python List-based Circular Buffer: {time_python:.4f} seconds")

# Timing the Numba + NumPy implementation
time_numba = timeit.timeit(stmt=test_code, setup=setup_numba, number=1)
print(f"Numba + NumPy Circular Buffer: {time_numba:.4f} seconds")
