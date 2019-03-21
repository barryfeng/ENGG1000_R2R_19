import unittest

class TestBasicFunc(unittest.TestCase):
    def test_upper(self):
        self.assertEqual('foo'.upper(), 'FOO', 'Works!')

if __name__ == '__main__':
    unittest.main()
