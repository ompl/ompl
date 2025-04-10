#include <nanobind/nanobind.h>
#include "ompl/base/GenericParam.h"
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::base::init_GenericParam(nb::module_& m)
{
    nb::class_<ompl::base::ParamSet>(m, "ParamSet");
}
