#!/usr/bin/env python3

# This script is called by a game engine script and run outside of Blender.
# It can call procedures within Blender using the call() function.


#NOTE: Be sure to run complicated objects through stringify(), instead of 
# str() when composing the argument for call() to get rid of newlines.

#NOTE: use print(..., file=sys.stderr) for debug output to file 'plan.out'

import sys

def stringify(thing):
    """
    Prepare a Python object for transmission over pipes.
    """
    
    # remove newlines from string representation
    return ' '.join(repr(thing).split('\n'))


def call(cmd):
    """
    Request a function call from Blender; cmd must not
    contain any newlines.
    """

    # submit cmd to stdout, return eval()'ed stdin response
    return eval(input(cmd + '\n'))
    

# Demonstration

# for 100 ticks
for i in range(100):

    if i==0:   # at tick 0
        # extract world state data from engine
        state = call('extractState()')
    elif i==50:    # at tick 50
        # set world state to what it was at tick 0
        call('submitState(' + stringify(state) + ')')

    # tell script not to listen for any more commands and continue
    call('nextTick()')

# stop the game engine
call('endPlanning()')

