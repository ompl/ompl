#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/Path.h"
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::base::init_Path(nb::module_& m)
{
    nb::class_<ompl::base::Path>(m, "Path");
}
