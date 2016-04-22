from os.path import abspath, dirname
from ompl import dll_loader
dll_loader('ompl', dirname(abspath(__file__)))
from ompl.util._util import *
import inspect

def OMPL_DEBUG(text):
    c = inspect.currentframe().f_back
    getOutputHandler().log(text, LogLevel.LOG_DEBUG, c.f_code.co_filename, c.f_lineno)
def OMPL_INFORM(text):
    c = inspect.currentframe().f_back
    getOutputHandler().log(text, LogLevel.LOG_INFO, c.f_code.co_filename, c.f_lineno)
def OMPL_WARN(text):
    c = inspect.currentframe().f_back
    getOutputHandler().log(text, LogLevel.LOG_WARN, c.f_code.co_filename, c.f_lineno)
def OMPL_ERROR(text):
    c = inspect.currentframe().f_back
    getOutputHandler().log(text, LogLevel.LOG_ERROR, c.f_code.co_filename, c.f_lineno)
