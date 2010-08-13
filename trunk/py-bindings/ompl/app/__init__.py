from ompl.geometric import *
try:
	from _app import *
except:
	print """  The ompl.app module could not be imported. This module is only included
  with the OMPL-app front-end, not the library-only distribution. If you
  downloaded the OMPL-app front-end, perhaps the ompl.app module did not
  compile?"""
