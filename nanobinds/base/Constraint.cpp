#include <nanobind/nanobind.h>
#include <nanobind/trampoline.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/eigen/dense.h>
#include "ompl/base/Constraint.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_Constraint(nb::module_ &m)
{
    m.attr("CONSTRAINT_PROJECTION_TOLERANCE") = nb::cast(ompl::magic::CONSTRAINT_PROJECTION_TOLERANCE);
    m.attr("CONSTRAINT_PROJECTION_MAX_ITERATIONS") = nb::cast(ompl::magic::CONSTRAINT_PROJECTION_MAX_ITERATIONS);

    // trampoline to allow Python subclassing of the abstract base
    struct PyConstraint : ob::Constraint
    {
        NB_TRAMPOLINE(ob::Constraint, 6);
        void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
        {
            NB_OVERRIDE_PURE(function, x, out);
        }
        void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
        {
            NB_OVERRIDE(jacobian, x, out);
        }
        bool project(Eigen::Ref<Eigen::VectorXd> x) const override
        {
            NB_OVERRIDE(project, x);
        }
        double distance(const Eigen::Ref<const Eigen::VectorXd> &x) const override
        {
            NB_OVERRIDE(distance, x);
        }
        bool isSatisfied(const Eigen::Ref<const Eigen::VectorXd> &x) const override
        {
            NB_OVERRIDE(isSatisfied, x);
        }
    };

    // bind Constraint (only non-virtual methods)
    nb::class_<ob::Constraint, PyConstraint /* <-- trampoline */>(m, "Constraint")
        .def(nb::init<unsigned int, unsigned int, double>(), nb::arg("ambientDim"), nb::arg("coDim"),
             nb::arg("tolerance") = ompl::magic::CONSTRAINT_PROJECTION_TOLERANCE)
        .def("getAmbientDimension", &ob::Constraint::getAmbientDimension)
        .def("getManifoldDimension", &ob::Constraint::getManifoldDimension)
        .def("getCoDimension", &ob::Constraint::getCoDimension)
        .def("setManifoldDimension", &ob::Constraint::setManifoldDimension, nb::arg("k"))
        .def("getTolerance", &ob::Constraint::getTolerance)
        .def("getMaxIterations", &ob::Constraint::getMaxIterations)
        .def("setTolerance", &ob::Constraint::setTolerance, nb::arg("tolerance"))
        .def("setMaxIterations", &ob::Constraint::setMaxIterations, nb::arg("iterations"))
        // .def("jacobian", &ob::Constraint::jacobian, nb::arg("x"), nb::arg("out"))
        .def("project", nb::overload_cast<Eigen::Ref<Eigen::VectorXd>>(&ob::Constraint::project, nb::const_),
             nb::arg("x"))
        .def("project", nb::overload_cast<ob::State*>(&ob::Constraint::project, nb::const_),
             nb::arg("x"))
        .def("distance", nb::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&>(&ob::Constraint::distance, nb::const_),
             nb::arg("x"))
        .def("distance", nb::overload_cast<const ob::State*>(&ob::Constraint::distance, nb::const_),
             nb::arg("x"))
        .def("isSatisfied", nb::overload_cast<const Eigen::Ref<const Eigen::VectorXd>&>(&ob::Constraint::isSatisfied, nb::const_),
             nb::arg("x"))
        .def("isSatisfied", nb::overload_cast<const ob::State*>(&ob::Constraint::isSatisfied, nb::const_),
             nb::arg("x"))
        ;

    // bind ConstraintIntersection (inherits Constraint)
    nb::class_<ob::ConstraintIntersection, ob::Constraint>(m, "ConstraintIntersection")
        .def(nb::init<unsigned int, std::vector<ob::ConstraintPtr>>(), nb::arg("ambientDim"), nb::arg("constraints"));

    // bind ConstraintObjective (inherits OptimizationObjective)
    // nb::class_<ob::ConstraintObjective, ob::OptimizationObjective>(m, "ConstraintObjective")
    //     .def(nb::init<ob::ConstraintPtr, ob::SpaceInformationPtr>(),
    //          nb::arg("constraint"),
    //          nb::arg("si"));
}
