#include <nanobind/nanobind.h>

#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace om = ompl::multilevel;

void ompl::binding::multilevel::initGraphsampler_GraphSampler(nb::module_ &m)
{
    nb::class_<om::BundleSpaceGraphSampler>(m, "GraphSampler")
        .def(
            "sample", [](om::BundleSpaceGraphSampler &self, ob::State *xRandom) { self.sample(xRandom); },
            nb::arg("xRandom"))
        .def("setPathBiasStartSegment", &om::BundleSpaceGraphSampler::setPathBiasStartSegment, nb::arg("bias"))
        .def("getPathBiasStartSegment", &om::BundleSpaceGraphSampler::getPathBiasStartSegment)
        .def("disableSegmentBias", &om::BundleSpaceGraphSampler::disableSegmentBias)
        .def("disablePathBias", &om::BundleSpaceGraphSampler::disablePathBias)
        .def("clear", &om::BundleSpaceGraphSampler::clear);
}
