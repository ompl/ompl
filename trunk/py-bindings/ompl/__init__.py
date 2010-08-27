import ctypes
from ctypes.util import find_library

for lib in ['ompl', 'ompl_app']:
	libname = find_library(lib)
	if libname:
		ctypes.CDLL(libname, ctypes.RTLD_GLOBAL)
	else:
		# find_library can't seem to find the libraries on Linux,
		# even if the libs are in the current directory or
		# if LD_LIBRARY_PATH is set to the directory where the libs
		# reside. The call below still succeeds, though.
		ctypes.CDLL('lib'+lib+'.so', ctypes.RTLD_GLOBAL)
