#!/usr/bin/env python3

# This script is called by the Blender plugin and run outside of Blender.
# It can call procedures within the plugin using the call() function.


#NOTE: Be sure to run complicated objects through stringify(), instead of 
# str() when composing the argument for call() to get rid of newlines.

#NOTE: use print(..., file=sys.stderr) for debug output to file 'ext.out'

import sys

def stringify(thing):
    """
    Prepare a Python object for transmission over pipes.
    """
    
    # remove newlines from string representation
    return ' '.join(repr(thing).split('\n'))


def call(cmd):
    """
    Request a function call from the plugin; cmd must not
    contain any newlines.
    """

    # submit cmd to stdout, return eval()'ed stdin response
    return eval(input(cmd + '\n'))
    

# Demonstration

# extract world state data from plugin
state = call('extractState()')

# submit world state data
call('submitState(' + stringify(state) + ')')

# tell plugin not to listen for any more commands
call('quit()')


