#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include "ompl/util/Exception.h"
#include "init.h"

namespace nb = nanobind;

void ompl::binding::util::init_Exception(nb::module_ &m)
{
    nb::exception<ompl::Exception>(m, "Exception", PyExc_RuntimeError);
}
