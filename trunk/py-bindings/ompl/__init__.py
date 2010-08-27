import ctypes
from ctypes.util import find_library

for lib in ['ompl', 'ompl_app']:
	libname = find_library(lib)
	if libname:
		ctypes.CDLL(libname, ctypes.RTLD_GLOBAL)
	else:
		from os.path import abspath, dirname
		# find_library can't seem to find the libraries on Linux,
		# even if the libs are in the current directory or
		# if LD_LIBRARY_PATH is set to the directory where the libs
		# reside. The call below still succeeds, though.
		ctypes.CDLL(
			dirname(abspath(__file__))+'/lib'+lib+'.so',
			ctypes.RTLD_GLOBAL)
