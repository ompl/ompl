import ctypes
from ctypes.util import find_library

for __lib in ['ompl', 'ompl_app']:
	__libname = find_library(__lib)
	if __libname:
		ctypes.CDLL(__libname, ctypes.RTLD_GLOBAL)
	else:
		from platform import system
		from os.path import abspath, dirname
		__sys = system()
		if __sys=='Windows':
			__ext='.dll'
		elif __sys=='Darwin':
			__ext='.dylib'
		else: # Linux, other UNIX systems
			__ext='.so'
		ctypes.CDLL(
			dirname(abspath(__file__))+'/lib'+__lib+__ext,
			ctypes.RTLD_GLOBAL)
