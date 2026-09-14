#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/samplers/ConditionalStateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ValidStateSampler.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

namespace
{
    struct PyConditionalStateSampler : ob::ConditionalStateSampler
    {
        ob::ConditionalStateSampler::Motion *startMotionPtr{nullptr};
        std::vector<ob::ConditionalStateSampler::Motion *> goalMotionsStorage;
        std::vector<ob::ConditionalStateSampler::Motion *> newBatchGoalMotionsStorage;
        bool sampleOldBatchFlag{false};

        explicit PyConditionalStateSampler(const ob::SpaceInformation *si)
          : ob::ConditionalStateSampler(si, startMotionPtr, goalMotionsStorage, newBatchGoalMotionsStorage,
                                        sampleOldBatchFlag)
        {
        }
    };
}  // namespace

void ompl::binding::base::initSamplers_ConditionalStateSampler(nb::module_ &m)
{
    nb::class_<ob::ConditionalStateSampler::Motion>(m, "ConditionalStateSamplerMotion")
        .def(nb::init<>())
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def_rw("root", &ob::ConditionalStateSampler::Motion::root)
        .def_rw("state", &ob::ConditionalStateSampler::Motion::state)
        .def_rw("parent", &ob::ConditionalStateSampler::Motion::parent)
        .def_rw("connectionPoint", &ob::ConditionalStateSampler::Motion::connectionPoint)
        .def_rw("numConnections", &ob::ConditionalStateSampler::Motion::numConnections);

    nb::class_<ob::ConditionalStateSampler, PyConditionalStateSampler, ob::ValidStateSampler>(m,
                                                                                              "ConditionalStateSampler")
        .def(nb::init<const ob::SpaceInformation *>(), nb::arg("si"))
        .def("startMotionPtr", [](ob::ConditionalStateSampler &self)
             { return static_cast<PyConditionalStateSampler &>(self).startMotionPtr; })
        .def(
            "setStartMotionPtr", [](ob::ConditionalStateSampler &self, ob::ConditionalStateSampler::Motion *motion)
            { static_cast<PyConditionalStateSampler &>(self).startMotionPtr = motion; }, nb::arg("motion"))
        .def(
            "goalMotions", [](ob::ConditionalStateSampler &self) -> std::vector<ob::ConditionalStateSampler::Motion *> &
            { return static_cast<PyConditionalStateSampler &>(self).goalMotionsStorage; },
            nb::rv_policy::reference_internal)
        .def(
            "newBatchGoalMotions",
            [](ob::ConditionalStateSampler &self) -> std::vector<ob::ConditionalStateSampler::Motion *> &
            { return static_cast<PyConditionalStateSampler &>(self).newBatchGoalMotionsStorage; },
            nb::rv_policy::reference_internal)
        .def("sampleOldBatchFlag", [](ob::ConditionalStateSampler &self)
             { return static_cast<PyConditionalStateSampler &>(self).sampleOldBatchFlag; })
        .def(
            "setSampleOldBatchFlag", [](ob::ConditionalStateSampler &self, bool value)
            { static_cast<PyConditionalStateSampler &>(self).sampleOldBatchFlag = value; }, nb::arg("value"))
        .def("sample", &ob::ConditionalStateSampler::sample, nb::arg("state"))
        .def("sampleNear", &ob::ConditionalStateSampler::sampleNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"));
}
