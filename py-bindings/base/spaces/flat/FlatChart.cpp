#include <nanobind/nanobind.h>
#include <nanobind/trampoline.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/spaces/flat/FlatChart.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

namespace
{
    struct PyFlatChart : ob::FlatChart
    {
        NB_TRAMPOLINE(ob::FlatChart, 2);

        void difference(const ob::State *from, const ob::State *to, Eigen::Ref<Eigen::VectorXd> out) const override
        {
            NB_OVERRIDE_PURE(difference, from, to, out);
        }

        void advance(const ob::State *from, const Eigen::Ref<const Eigen::VectorXd> &delta,
                     ob::State *out) const override
        {
            NB_OVERRIDE_PURE(advance, from, delta, out);
        }
    };

    // Reaches the protected periods of any chart, so a Python subclass can set them the way a C++ subclass writes
    // the member.
    struct FlatChartPeriods : ob::FlatChart
    {
        static std::vector<double> &of(ob::FlatChart &chart)
        {
            return chart.*(&FlatChartPeriods::periods_);
        }
    };
}  // namespace

void ompl::binding::base::initSpacesFlat_FlatChart(nb::module_ &m)
{
    nb::class_<ob::FlatChart, PyFlatChart>(m, "FlatChart")
        .def(nb::init<unsigned int>(), nb::arg("dimension"))
        .def("getDimension", &ob::FlatChart::getDimension)
        .def("getPeriods", &ob::FlatChart::getPeriods)
        .def(
            "setPeriods",
            [](ob::FlatChart &chart, const std::vector<double> &periods)
            {
                if (periods.size() != chart.getDimension())
                    throw nb::value_error("setPeriods: periods must hold one entry per coordinate");
                FlatChartPeriods::of(chart) = periods;
            },
            nb::arg("periods"))
        .def("difference", &ob::FlatChart::difference, nb::arg("from_"), nb::arg("to"), nb::arg("out"))
        .def("advance", &ob::FlatChart::advance, nb::arg("from_"), nb::arg("delta"), nb::arg("out"));

    nb::class_<ob::RealVectorFlatChart, ob::FlatChart>(m, "RealVectorFlatChart")
        .def(nb::init<unsigned int>(), nb::arg("dimension"));

    nb::class_<ob::SO2FlatChart, ob::FlatChart>(m, "SO2FlatChart").def(nb::init<>());

    nb::class_<ob::CompoundFlatChart, ob::FlatChart>(m, "CompoundFlatChart")
        .def(nb::init<std::vector<ob::FlatChartPtr>>(), nb::arg("components"))
        .def("getComponents", &ob::CompoundFlatChart::getComponents);

    m.def("allocFlatChart", &ob::allocFlatChart, nb::arg("space"));
}
