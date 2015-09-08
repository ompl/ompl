#!/bin/sh
tar cf - -s/html/core/ html | tar xf - -C ${HOME}/src/ompl.github.io
