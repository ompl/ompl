#include <nanobind/nanobind.h>

#include <ompl/multilevel/datastructures/graphsampler/RandomVertex.h>
#include "../init.h"

namespace nb = nanobind;
namespace om = ompl::multilevel;

void ompl::binding::multilevel::initGraphsampler_RandomVertex(nb::module_ &m)
{
    nb::class_<om::BundleSpaceGraphSamplerRandomVertex, om::BundleSpaceGraphSampler>(m, "RandomVertex")
        .def(nb::init<om::BundleSpaceGraph *>(), nb::arg("graph"));
}
