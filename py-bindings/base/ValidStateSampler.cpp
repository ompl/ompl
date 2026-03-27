#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/function.h>
#include <nanobind/trampoline.h>
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_ValidStateSampler(nb::module_ &m)
{
    // Trampoline class for Python subclassing
    struct PyValidStateSampler : ob::ValidStateSampler
    {
        NB_TRAMPOLINE(ob::ValidStateSampler, 2);

        bool sample(ob::State *state) override
        {
            NB_OVERRIDE_PURE(sample, state);
        }

        bool sampleNear(ob::State *state, const ob::State *near, double distance) override
        {
            NB_OVERRIDE_PURE(sampleNear, state, near, distance);
        }
    };

    nb::class_<ob::ValidStateSampler, PyValidStateSampler /* <-- trampoline */>(m, "ValidStateSampler")
        .def(nb::init<const ob::SpaceInformation *>(), nb::arg("si"))
        .def("getName", &ob::ValidStateSampler::getName)
        .def("setName", &ob::ValidStateSampler::setName, nb::arg("name"))
        .def("sample", &ob::ValidStateSampler::sample, nb::arg("state"))
        .def("sampleNear", &ob::ValidStateSampler::sampleNear, nb::arg("state"), nb::arg("near"), nb::arg("distance"))
        .def("setNrAttempts", &ob::ValidStateSampler::setNrAttempts, nb::arg("attempts"))
        .def("getNrAttempts", &ob::ValidStateSampler::getNrAttempts);

    // ValidStateSamplerAllocator is std::function<ValidStateSamplerPtr(const SpaceInformation *)>
    // nanobind handles std::function automatically with stl/function.h
}
