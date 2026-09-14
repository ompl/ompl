#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/trampoline.h>
#include "ompl/base/MotionValidator.h"
#include "ompl/base/SpaceInformation.h"
#include "init.h"
namespace nb = nanobind;  // Trampoline class for Python subclassing
class PyMotionValidator : public ompl::base::MotionValidator
{
public:
    NB_TRAMPOLINE(ompl::base::MotionValidator, 2);
    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override
    {
        NB_OVERRIDE_PURE(checkMotion, s1, s2);
    }
    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                     std::pair<ompl::base::State *, double> &lastValid) const override
    {
        nb::detail::ticket nb_ticket(nb_trampoline, "checkMotionWithLastValid", false);
        // Without the override, lastValid keeps whatever the caller put there: LBKPIECE1 and RLRT
        // then discard the valid prefix, and PDST keeps an unvalidated state. Same as before.
        if (!nb_ticket.key.is_valid())
            return checkMotion(s1, s2);

        nb::object state = nb::none();
        if (lastValid.first != nullptr)
            state = nb::cast(lastValid.first, nb::rv_policy::reference);

        // The fraction comes back as a return value because nanobind's pair caster copies; the state
        // is written through the reference above.
        auto result = nb::cast<std::pair<bool, double>>(nb_trampoline.base().attr(nb_ticket.key)(s1, s2, state));
        lastValid.second = result.second;
        return result.first;
    }
};
void ompl::binding::base::init_MotionValidator(nb::module_ &m)
{
    nb::class_<ompl::base::MotionValidator, PyMotionValidator>(m, "MotionValidator")
        .def(nb::init<ompl::base::SpaceInformation *>())
        .def("checkMotion", nb::overload_cast<const ompl::base::State *, const ompl::base::State *>(
                                &ompl::base::MotionValidator::checkMotion, nb::const_))
        .def(
            "checkMotionWithLastValid",
            [](const ompl::base::MotionValidator &mv, const ompl::base::State *s1, const ompl::base::State *s2,
               ompl::base::State *lastValidState)
            {
                std::pair<ompl::base::State *, double> lastValid(lastValidState, 0.0);
                bool valid = mv.checkMotion(s1, s2, lastValid);
                return std::make_pair(valid, lastValid.second);
            },
            nb::arg("s1"), nb::arg("s2"), nb::arg("lastValidState").none())
        .def("getValidMotionCount", &ompl::base::MotionValidator::getValidMotionCount)
        .def("getInvalidMotionCount", &ompl::base::MotionValidator::getInvalidMotionCount)
        .def("getCheckedMotionCount", &ompl::base::MotionValidator::getCheckedMotionCount)
        .def("getValidMotionFraction", &ompl::base::MotionValidator::getValidMotionFraction)
        .def("resetMotionCounter", &ompl::base::MotionValidator::resetMotionCounter);
}
