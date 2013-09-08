#!/usr/bin/env python
# This script makes the boost::numeric::odeint source future proof by
# changing the namespace and include directory to something OMPL specific.
#
# It modifies the current directory (and subdirectories) by changing all
# instances of the namespace odeint to omplext_odeint, and all include
# paths of <boost/numeric/odeint/...> to <omplext_odeint/boost/numeric/odeint/...>

import os
import sys

# Changes the source code by renaming the odeint namespace to omplext_odeint
# and "moves" the directory tree to omplext_odeint/boost/numeric/odeint
def FixupSource(headerFilename):
    header = open(headerFilename, 'r')

    lines = header.readlines()
    newlines = []

    for l in lines:
        # Changing namespace odeint to omplext_odeint
        if l.find('namespace odeint') != -1:
            line = l
            line = line.replace ('namespace odeint', 'namespace omplext_odeint')
            newlines.append(line)
        # Changing instances of boost::numeric::odeint to boost::numeric::omplext_odeint
        elif l.find('boost::numeric::odeint') != -1:
            line = l
            line = line.replace ('boost::numeric::odeint', 'boost::numeric::omplext_odeint')
            newlines.append(line)
        # Changing instance of typename odeint:: to typename omplext_odeint
        elif l.find('typename odeint::') != -1:
            line = l
            line = line.replace ('typename odeint::', 'typename omplext_odeint::')
            newlines.append(line)
        # Patch includes
        elif l.find('#include') != -1 and l.find('boost/numeric/odeint') != -1:
            line = l
            line = line.replace ('boost/numeric/odeint', 'omplext_odeint/boost/numeric/odeint')
            newlines.append(line)
        # Patch macros
        elif l.find('#ifndef BOOST_NUMERIC_ODEINT_') != -1 or l.find('#define BOOST_NUMERIC_ODEINT_') != -1:
            line = l
            line = line.replace ('BOOST_NUMERIC_ODEINT_', 'OMPLEXT_BOOST_NUMERIC_ODEINT_')
            newlines.append(line)
        else:
            newlines.append(l)
    header.close()

    # Writing the revised source code
    header = open(headerFilename, 'w')
    for l in newlines:
        header.write(l)
    header.close()

def main(searchDirectory):

    # Find all headers and fix the source.
    for root, dirs, files in os.walk(searchDirectory):
        for f in files:
            if f.endswith('.hpp'):
                ret = FixupSource(os.path.join(root, f))


if __name__ == '__main__':
    searchDirectory = "."
    if len(sys.argv) == 2:
        searchDirectory = str(sys.argv[1])
    main (searchDirectory)


