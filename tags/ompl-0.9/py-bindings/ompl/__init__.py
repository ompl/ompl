def dll_loader(lib, fallback_dir):
	import ctypes
	from ctypes.util import find_library
	libname = find_library(lib)
	if libname:
		ctypes.CDLL(libname, ctypes.RTLD_GLOBAL)
	else:
		from platform import system
		from os.path import isfile
		
		sys = system()
		if sys=='Windows':
			ext='.dll'
		elif sys=='Darwin':
			ext='.dylib'
		else: # Linux, other UNIX systems
			ext='.so'
		fname = fallback_dir+'/lib'+lib+ext
		if isfile(fname):
			ctypes.CDLL(fname, ctypes.RTLD_GLOBAL)
		else:
			raise ImportError

