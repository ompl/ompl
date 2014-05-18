# - Find the GCC-XML front-end executable.
#
# This module will define the following variables:
#  GCCXML - the GCC-XML front-end executable.

#=============================================================================
# Copyright 2001-2009 Kitware, Inc.
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of CMake, substitute the full
#  License text for the above reference.)

# added extra path, to check for gccxml installed in build directory
find_program(GCCXML NAMES gccxml
    PATHS "${PROJECT_BINARY_DIR}/pyplusplus/bin"
    [HKEY_CURRENT_USER\\Software\\Kitware\\GCC_XML;loc]
    "$ENV{ProgramFiles}/GCC_XML"
    "C:/Program Files/GCC_XML")

mark_as_advanced(GCCXML)
