#!/usr/bin/env python

import unittest
import test_base, test_dynamic, test_kinematic, test_util

def suite():
	suites = (
		# test_base.suite(),
		# test_dynamic.suite(),
		# test_kinematic.suite(),
		test_util.suite()
	)
	# combine them into a single suite
	return unittest.TestSuite(suites)

if __name__ == '__main__':
	unittest.TextTestRunner(verbosity=3).run(suite())
