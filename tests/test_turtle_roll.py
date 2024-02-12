"""Test of TurtleRoll"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import unittest


class TestTurtle(unittest.TestCase):
  def test_positions(self):
    turtle = Turtle(capacity=5)
    moves = [(1, 1), (2, 2), (3, 3), (4, 4), (5, 5), (6, 6)]

    for pos in moves:
      turtle.move(pos)

    # Only the last 5 positions should be stored due to the capacity limit
    expected_positions = [(2, 2), (3, 3), (4, 4), (5, 5), (6, 6)]
    self.assertEqual(turtle.get_positions(), expected_positions)


if __name__ == '__main__':
  unittest.main()
