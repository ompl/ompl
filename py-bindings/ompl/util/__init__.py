from os.path import abspath, dirname
from sys import platform
if platform != 'nt' and platform != 'win32':
    from ompl import dll_loader
    dll_loader('ompl', dirname(abspath(__file__)))
from _util import *
