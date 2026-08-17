#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/RRTXstatic.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_RRTXstatic(nb::module_ &m)
{
    nb::class_<og::RRTXstatic, ob::Planner>(m, "RRTXstatic")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve",
             [](og::RRTXstatic &self, nb::object what)
             {
                 if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                 {
                     return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                 }
                 else if (nb::isinstance<double>(what))
                 {
                     return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                 }
                 else
                 {
                     throw nb::type_error(
                         "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
                 }
             })
        .def(
            "getPlannerData", [](const og::RRTXstatic &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::RRTXstatic::clear)
        .def("setup", &og::RRTXstatic::setup)
        .def("setGoalBias", &og::RRTXstatic::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &og::RRTXstatic::getGoalBias)
        .def("setInformedSampling", &og::RRTXstatic::setInformedSampling, nb::arg("informedSampling"))
        .def("getInformedSampling", &og::RRTXstatic::getInformedSampling)
        .def("setSampleRejection", &og::RRTXstatic::setSampleRejection, nb::arg("reject"))
        .def("getSampleRejection", &og::RRTXstatic::getSampleRejection)
        .def("setNumSamplingAttempts", &og::RRTXstatic::setNumSamplingAttempts, nb::arg("numAttempts"))
        .def("getNumSamplingAttempts", &og::RRTXstatic::getNumSamplingAttempts)
        .def("setEpsilon", &og::RRTXstatic::setEpsilon, nb::arg("epsilon"))
        .def("getEpsilon", &og::RRTXstatic::getEpsilon)
        .def("setRange", &og::RRTXstatic::setRange, nb::arg("distance"))
        .def("getRange", &og::RRTXstatic::getRange)
        .def("setRewireFactor", &og::RRTXstatic::setRewireFactor, nb::arg("rewireFactor"))
        .def("getRewireFactor", &og::RRTXstatic::getRewireFactor)
        .def("setKNearest", &og::RRTXstatic::setKNearest, nb::arg("useKNearest"))
        .def("getKNearest", &og::RRTXstatic::getKNearest)
        .def("setUpdateChildren", &og::RRTXstatic::setUpdateChildren, nb::arg("val"))
        .def("getUpdateChildren", &og::RRTXstatic::getUpdateChildren)
        .def("setVariant", &og::RRTXstatic::setVariant, nb::arg("variant"))
        .def("getVariant", &og::RRTXstatic::getVariant)
        .def("setAlpha", &og::RRTXstatic::setAlpha, nb::arg("a"))
        .def("getAlpha", &og::RRTXstatic::getAlpha)
        .def("numIterations", &og::RRTXstatic::numIterations)
        .def("bestCost", &og::RRTXstatic::bestCost);
}
