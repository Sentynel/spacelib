import unittest

from spacelib import util

class TestVectors(unittest.TestCase):
    def setUp(self):
        self.v1 = (1,0,0)
        self.v2 = (0,1,0)
        self.v3 = (3,4,0)

    def test_dot_product(self):
        self.assertEqual(util.dot_product(self.v1, self.v2), 0)
        self.assertEqual(util.dot_product(self.v1, self.v1), 1)

    def test_magnitude(self):
        self.assertEqual(util.vector_magnitude(self.v1), 1)
        self.assertEqual(util.vector_magnitude(self.v3), 5)

    def test_angle(self):
        self.assertEqual(util.vector_angle(self.v1, self.v1), 0)
        self.assertEqual(util.vector_angle(self.v1, self.v2), 90)
