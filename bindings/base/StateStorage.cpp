#include <nanobind/nanobind.h>
#include "ompl/base/StateStorage.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_StateStorage(nb::module_& m)
{
    // TODO [ob::StateStorage][IMPLEMENT]
    nb::class_<ob::StateStorage>(m, "StateStorage")
        ;
}
