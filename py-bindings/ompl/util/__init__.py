from os.path import abspath, dirname
from ompl import dll_loader
dll_loader('ompl', dirname(abspath(__file__)))
from ompl.util._util import *
