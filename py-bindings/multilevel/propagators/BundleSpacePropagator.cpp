#include <nanobind/nanobind.h>

#include <ompl/multilevel/datastructures/propagators/BundleSpacePropagator.h>
#include "../init.h"

namespace nb = nanobind;
namespace om = ompl::multilevel;

void ompl::binding::multilevel::initPropagators_BundleSpacePropagator(nb::module_ &m)
{
    nb::class_<om::BundleSpacePropagator>(m, "BundleSpacePropagator");
}
