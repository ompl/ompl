#include <nanobind/nanobind.h>
#include "ompl/base/GenericParam.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_GenericParam(nb::module_& m)
{
    // TODO [ob::GenericParam][IMPLEMENT]
    nb::class_<ob::GenericParam>(m, "GenericParam")
        ;

    // TODO [ob::SpecificParam][TRAMPOLINE]
    // nb::class_<ob::SpecificParam, ob::GenericParam>(m, "SpecificParam");

    // TODO [ob::ParamSet][IMPLEMENT]
    nb::class_<ob::ParamSet>(m, "ParamSet")
        ;
}
