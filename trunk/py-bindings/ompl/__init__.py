import ctypes
from ctypes.util import find_library
ompl = find_library('ompl')
ompl_app = find_library('ompl_app')
if ompl:
	ctypes.CDLL(ompl, ctypes.RTLD_GLOBAL)
if ompl_app:
	ctypes.CDLL(ompl_app, ctypes.RTLD_GLOBAL)
