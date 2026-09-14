#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>

#include "ompl/base/samplers/deterministic/DeterministicSequence.h"
#include "ompl/base/samplers/deterministic/HaltonSequence.h"
#include "ompl/base/samplers/deterministic/PrecomputedSequence.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplersDeterministic_DeterministicSequence(nb::module_ &m)
{
    struct PyDeterministicSequence : ob::DeterministicSequence
    {
        NB_TRAMPOLINE(ob::DeterministicSequence, 1);

        std::vector<double> sample() override
        {
            NB_OVERRIDE_PURE(sample);
        }
    };

    nb::class_<ob::DeterministicSequence, PyDeterministicSequence>(m, "DeterministicSequence")
        .def(nb::init<unsigned int>(), nb::arg("dimensions"))
        .def_prop_ro("dimensions", [](const ob::DeterministicSequence &ds) { return ds.dimensions_; })
        .def("sample", &ob::DeterministicSequence::sample);

    nb::class_<ob::HaltonSequence1D>(m, "HaltonSequence1D")
        .def(nb::init<>())
        .def(nb::init<unsigned int>(), nb::arg("base"))
        .def("setBase", &ob::HaltonSequence1D::setBase, nb::arg("base"))
        .def("sample", &ob::HaltonSequence1D::sample);

    nb::class_<ob::HaltonSequence, ob::DeterministicSequence>(m, "HaltonSequence")
        .def(nb::init<unsigned int>(), nb::arg("dimensions"))
        .def(nb::init<unsigned int, std::vector<unsigned int>>(), nb::arg("dimensions"), nb::arg("bases"))
        .def("sample", &ob::HaltonSequence::sample);

    nb::class_<ob::PrecomputedSequence, ob::DeterministicSequence>(m, "PrecomputedSequence")
        .def(nb::init<std::string, unsigned int, bool, std::size_t>(), nb::arg("path"), nb::arg("dimensions"),
             nb::arg("shuffle") = false, nb::arg("max_samples") = 0)
        .def("sample", &ob::PrecomputedSequence::sample);
}
