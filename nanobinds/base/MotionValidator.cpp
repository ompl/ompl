#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/trampoline.h>
#include "ompl/base/MotionValidator.h"
#include "ompl/base/SpaceInformation.h"
#include "init.hh"
namespace nb = nanobind;// Trampoline class for Python subclassing
class PyMotionValidator : public ompl::base::MotionValidator {
public:
    NB_TRAMPOLINE(ompl::base::MotionValidator, 2);    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override {
        NB_OVERRIDE_PURE(checkMotion, s1, s2);
    }    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                    std::pair<ompl::base::State *, double> &lastValid) const override {
        // This is tricky to bind due to the output parameter
        // For now, default to calling the simpler version
        return checkMotion(s1, s2);
    }
};void ompl::binding::base::init_MotionValidator(nb::module_& m)
{
    nb::class_<ompl::base::MotionValidator, PyMotionValidator>(m, "MotionValidator")
        .def(nb::init<ompl::base::SpaceInformation*>())
        .def("checkMotion",
             nb::overload_cast<const ompl::base::State*, const ompl::base::State*>
             (&ompl::base::MotionValidator::checkMotion, nb::const_))
        .def("getValidMotionCount", &ompl::base::MotionValidator::getValidMotionCount)
        .def("getInvalidMotionCount", &ompl::base::MotionValidator::getInvalidMotionCount)
        .def("getCheckedMotionCount", &ompl::base::MotionValidator::getCheckedMotionCount)
        .def("getValidMotionFraction", &ompl::base::MotionValidator::getValidMotionFraction)
        .def("resetMotionCounter", &ompl::base::MotionValidator::resetMotionCounter);
}