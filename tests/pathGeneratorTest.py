import unittest
import sys
sys.path.insert(0, '../src')
import pathGenerator

class SimplisticTest(unittest.TestCase):

    def test(self):
        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()