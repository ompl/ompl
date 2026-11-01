#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/pair.h>
// #include <nanobind/eigen/dense.h>
#include <nanobind/trampoline.h>

#include "ompl/base/spaces/constraint/ConstrainedStateSpace.h"
#include "ompl/base/Constraint.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpacesConstraint_ConstrainedStateSpace(nb::module_ &m)
{
    // Trampoline for ConstrainedStateSpace so Python can override its virtuals:
    struct PyConstrainedStateSpace : ob::ConstrainedStateSpace
    {
        // We have 5 virtuals: clear, setup, sanityChecks, discreteGeodesic, geodesicInterpolate
        NB_TRAMPOLINE(ConstrainedStateSpace, 5);

        // OMPL frees every state in *geodesic, so it gets clones of the (ok, states) the override returns.
        bool discreteGeodesic(const ob::State *from, const ob::State *to, bool interpolate,
                              std::vector<ob::State *> *geodesic) const override
        {
            nb::detail::ticket nb_ticket(nb_trampoline, "discreteGeodesic", true);
            // Held while cloning: the returned list may own the only reference to its states.
            nb::object result = nb_trampoline.base().attr(nb_ticket.key)(from, to, interpolate);
            auto [ok, states] = nb::cast<std::pair<bool, std::vector<const ob::State *>>>(result);
            // interpolate() indexes the geodesic whenever ok is true.
            if (ok && geodesic != nullptr && states.empty())
                throw nb::value_error("discreteGeodesic() returned ok=True with no states");
            if (geodesic != nullptr)
                for (const ob::State *state : states)
                    geodesic->push_back(cloneState(state));
            return ok;
        }

        // Optional override
        ob::State *geodesicInterpolate(const std::vector<ob::State *> &geodesic, double t) const override
        {
            NB_OVERRIDE(geodesicInterpolate, geodesic, t);
        }

        // Optional overrides
        void setup() override
        {
            NB_OVERRIDE(setup);
        }
        void clear() override
        {
            NB_OVERRIDE(clear);
        }
        void sanityChecks() const override
        {
            NB_OVERRIDE(sanityChecks);
        }
    };

    m.attr("CONSTRAINED_STATE_SPACE_DELTA") = nb::cast(ompl::magic::CONSTRAINED_STATE_SPACE_DELTA);
    m.attr("CONSTRAINED_STATE_SPACE_LAMBDA") = nb::cast(ompl::magic::CONSTRAINED_STATE_SPACE_LAMBDA);

    // TODO [ob::ConstrainedMotionValidator][TEST]
    nb::class_<ob::ConstrainedMotionValidator, ob::MotionValidator>(m, "ConstrainedMotionValidator")
        .def(nb::init<ob::SpaceInformation *>(), nb::arg("si"))
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("siPtr"))
        .def("checkMotion",
             nb::overload_cast<const ob::State *, const ob::State *>(&ob::ConstrainedMotionValidator::checkMotion,
                                                                     nb::const_),
             nb::arg("s1"), nb::arg("s2"));

    // TODO [ob::ConstrainedStateSpace::StateType][TEST]
    nb::class_<ompl::base::ConstrainedStateSpace::StateType, ompl::base::WrapperStateSpace::StateType>(
        m, "ConstrainedStateSpaceStateType")
        .def(
            "copy",
            [](ob::ConstrainedStateSpace::StateType &st, const std::vector<double> &vec)
            {
                if (vec.size() != static_cast<std::size_t>(st.size()))
                    throw nb::value_error(
                        ("copy() expects " + std::to_string(st.size()) + " value(s), got " + std::to_string(vec.size()))
                            .c_str());
                Eigen::VectorXd v = Eigen::Map<const Eigen::VectorXd>(vec.data(), vec.size());
                st.copy(v);
            },
            nb::arg("other"))
        .def(
            "__getitem__",
            [](const ob::ConstrainedStateSpace::StateType &st, unsigned int i)
            {
                if (i >= static_cast<std::size_t>(st.size()))
                    throw nb::index_error("state index out of range");
                return st[i];
            },
            nb::arg("i"));

    // TODO [ob::ConstrainedStateSpace][TEST]
    nb::class_<ob::ConstrainedStateSpace, ob::WrapperStateSpace, PyConstrainedStateSpace /* <-- trampoline */>(
        m, "ConstrainedStateSpace")
        .def(nb::init<const ob::StateSpacePtr &, const ob::ConstraintPtr &>(), nb::arg("ambientSpace"),
             nb::arg("constraint"))
        .def("setSpaceInformation", &ob::ConstrainedStateSpace::setSpaceInformation, nb::arg("si"))
        .def("setup", &ob::ConstrainedStateSpace::setup)
        .def("clear", &ob::ConstrainedStateSpace::clear)
        .def("allocState", &ob::ConstrainedStateSpace::allocState)
        .def("constrainedSanityChecks", &ob::ConstrainedStateSpace::constrainedSanityChecks, nb::arg("flags"))
        .def("sanityChecks", &ob::ConstrainedStateSpace::sanityChecks)
        .def("validSegmentCount", &ob::ConstrainedStateSpace::validSegmentCount, nb::arg("s1"), nb::arg("s2"))
        .def("interpolate", &ob::ConstrainedStateSpace::interpolate, nb::arg("from"), nb::arg("to"), nb::arg("t"),
             nb::arg("state"))
        .def("geodesicInterpolate", &ob::ConstrainedStateSpace::geodesicInterpolate, nb::arg("geodesic"), nb::arg("t"))
        .def("setDelta", &ob::ConstrainedStateSpace::setDelta, nb::arg("delta"))
        .def("getDelta", &ob::ConstrainedStateSpace::getDelta)
        .def("setLambda", &ob::ConstrainedStateSpace::setLambda, nb::arg("lambda"))
        .def("getLambda", &ob::ConstrainedStateSpace::getLambda)
        .def("getAmbientDimension", &ob::ConstrainedStateSpace::getAmbientDimension)
        .def("getManifoldDimension", &ob::ConstrainedStateSpace::getManifoldDimension)
        .def("getConstraint", &ob::ConstrainedStateSpace::getConstraint, nb::rv_policy::reference_internal);
}
