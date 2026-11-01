#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include "ompl/util/Exception.h"
#include "init.h"

namespace nb = nanobind;

void ompl::binding::util::init_Exception(nb::module_ &m)
{
    // Not "Exception": `from ompl.util import *` would shadow the builtin. ompl/util.py keeps the old alias.
    nb::exception<ompl::Exception>(m, "OMPLException", PyExc_RuntimeError);
}
