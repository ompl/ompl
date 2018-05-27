#!/bin/sh
rm -rf html
doxygen Doxyfile
cp -r ompl.css images ieee-ram-2012-ompl.pdf ../install-ompl-ubuntu.sh html
