#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/Path.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_Path(nb::module_& m)
{
    // TODO [ob::Path][IMPLEMENT]
    nb::class_<ob::Path>(m, "Path");
}
