#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <nanobind/trampoline.h>
#include <nanobind/eigen/dense.h>
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/base/StateSpace.h"
#include "PyGC.h"
#include "init.h"
#include <sstream>

namespace nb = nanobind;
namespace gc = ompl::binding::gc;

namespace
{
    int projectionTraverse(PyObject *self, visitproc visit, void *arg)
    {
        Py_VISIT(Py_TYPE(self));
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;
        return ompl::binding::base::visitParams(nb::inst_ptr<ompl::base::ProjectionEvaluator>(self)->params(), visit,
                                                arg);
    }

    int projectionClear(PyObject *self)
    {
        if (nb::inst_ready(self) && gc::pythonOwned(self))
            ompl::binding::base::clearParams(nb::inst_ptr<ompl::base::ProjectionEvaluator>(self)->params());
        return 0;
    }

    // Eigen cannot resize a Ref, so a size mismatch would read or write out of bounds.
    void checkCoordinateSizes(const ompl::base::ProjectionEvaluator &pe, Eigen::Index projection, Eigen::Index coord)
    {
        const auto cells = static_cast<Eigen::Index>(pe.getCellSizes().size());
        if (cells != static_cast<Eigen::Index>(pe.getDimension()))
            throw nb::value_error("cell sizes are not set for every dimension; call setCellSizes() or setup()");
        if (projection != cells || coord != cells)
            throw nb::value_error("projection and coord must have one entry per projection dimension");
    }

    const PyType_Slot projectionSlots[] = {{Py_tp_traverse, (void *)projectionTraverse},
                                           {Py_tp_clear, (void *)projectionClear},
                                           {0, 0}};
}  // namespace

void ompl::binding::base::init_ProjectionEvaluator(nb::module_ &m)
{
    nb::class_<ompl::base::ProjectionMatrix>(m, "ProjectionMatrix")
        .def(nb::init<>())
        // return the class variable mat
        .def_rw("mat", &ompl::base::ProjectionMatrix::mat)
        .def_static("ComputeRandom",
                    nb::overload_cast<unsigned int, unsigned int, const std::vector<double> &>(
                        &ompl::base::ProjectionMatrix::ComputeRandom),
                    nb::arg("from"), nb::arg("to"), nb::arg("scale"))
        .def_static("ComputeRandom",
                    nb::overload_cast<unsigned int, unsigned int>(&ompl::base::ProjectionMatrix::ComputeRandom),
                    nb::arg("from"), nb::arg("to"))
        .def("computeRandom",
             nb::overload_cast<unsigned int, unsigned int, const std::vector<double> &>(
                 &ompl::base::ProjectionMatrix::computeRandom),
             nb::arg("from"), nb::arg("to"), nb::arg("scale"))
        .def("computeRandom",
             nb::overload_cast<unsigned int, unsigned int>(&ompl::base::ProjectionMatrix::computeRandom),
             nb::arg("from"), nb::arg("to"))
        .def(
            "project",
            [](const ompl::base::ProjectionMatrix &pm, const std::vector<double> &from, Eigen::Ref<Eigen::VectorXd> to)
            {
                // project() reads mat.cols() doubles through the raw pointer.
                if (from.size() != static_cast<std::size_t>(pm.mat.cols()))
                    throw nb::value_error("from must have one entry per matrix column");
                pm.project(from.data(), to);
            },
            nb::arg("from"), nb::arg("to"))
        .def("print", [](const ompl::base::ProjectionMatrix &pm) { pm.print(std::cout); })
        .def("__repr__",
             [](const ompl::base::ProjectionMatrix &pm)
             {
                 std::ostringstream oss;
                 pm.print(oss);
                 return oss.str();
             });

    struct PyProjectionEvaluator : ompl::base::ProjectionEvaluator
    {
        NB_TRAMPOLINE(ompl::base::ProjectionEvaluator, 5);
        unsigned int getDimension() const override
        {
            NB_OVERRIDE_PURE(getDimension);
        }
        void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
        {
            NB_OVERRIDE_PURE(project, state, projection);
        }
        void setCellSizes(const std::vector<double> &cellSizes) override
        {
            NB_OVERRIDE(setCellSizes, cellSizes);
        }
        void defaultCellSizes() override
        {
            NB_OVERRIDE(defaultCellSizes);
        }
        void setup() override
        {
            NB_OVERRIDE(setup);
        }
    };
    nb::class_<ompl::base::ProjectionEvaluator, PyProjectionEvaluator /* <-- trampoline */>(
        m, "ProjectionEvaluator", nb::type_slots(projectionSlots))
        .def(nb::init<const ompl::base::StateSpacePtr &>(), nb::arg("space"))
        .def("getDimension", &ompl::base::ProjectionEvaluator::getDimension)
        .def("project", &ompl::base::ProjectionEvaluator::project, nb::arg("state"), nb::arg("projection"))
        .def("setCellSizes",
             nb::overload_cast<const std::vector<double> &>(&ompl::base::ProjectionEvaluator::setCellSizes),
             nb::arg("cellSizes"))
        .def("setCellSizes", nb::overload_cast<unsigned int, double>(&ompl::base::ProjectionEvaluator::setCellSizes),
             nb::arg("dim"), nb::arg("cellSize"))
        .def("defaultCellSizes", &ompl::base::ProjectionEvaluator::defaultCellSizes)
        .def("setup", &ompl::base::ProjectionEvaluator::setup)
        .def("printSettings", [](const ompl::base::ProjectionEvaluator &pe) { pe.printSettings(std::cout); })
        .def(
            "printProjection", [](const ompl::base::ProjectionEvaluator &pe, Eigen::VectorXd projection)
            { pe.printProjection(projection, std::cout); }, nb::arg("projection"))
        .def("mulCellSizes", &ompl::base::ProjectionEvaluator::mulCellSizes, "Multiply the cell sizes by a factor",
             nb::arg("factor"))
        .def("userConfigured", &ompl::base::ProjectionEvaluator::userConfigured)
        .def("getCellSizes", nb::overload_cast<>(&ompl::base::ProjectionEvaluator::getCellSizes, nb::const_))
        .def("getCellSizes",
             nb::overload_cast<unsigned int>(&ompl::base::ProjectionEvaluator::getCellSizes, nb::const_),
             nb::arg("dim"))
        .def("checkCellSizes", &ompl::base::ProjectionEvaluator::checkCellSizes)
        .def("inferCellSizes", &ompl::base::ProjectionEvaluator::inferCellSizes)
        .def("checkBounds", &ompl::base::ProjectionEvaluator::checkBounds)
        .def("hasBounds", &ompl::base::ProjectionEvaluator::hasBounds)
        .def("setBounds", &ompl::base::ProjectionEvaluator::setBounds, nb::arg("bounds"))
        .def("getBounds", &ompl::base::ProjectionEvaluator::getBounds)
        .def("inferBounds", &ompl::base::ProjectionEvaluator::inferBounds)
        .def(
            "computeCoordinates",
            [](const ompl::base::ProjectionEvaluator &pe, const Eigen::Ref<Eigen::VectorXd> &projection,
               Eigen::Ref<Eigen::VectorXi> coord)
            {
                checkCoordinateSizes(pe, projection.size(), coord.size());
                pe.computeCoordinates(projection, coord);
            },
            nb::arg("projection"), nb::arg("coord"))
        .def(
            "computeCoordinates",
            [](const ompl::base::ProjectionEvaluator &pe, const ompl::base::State *state,
               Eigen::Ref<Eigen::VectorXi> coord)
            {
                checkCoordinateSizes(pe, pe.getDimension(), coord.size());
                pe.computeCoordinates(state, coord);
            },
            nb::arg("state"), nb::arg("coord"))
        .def("params", nb::overload_cast<>(&ompl::base::ProjectionEvaluator::params),
             nb::rv_policy::reference_internal);

    nb::class_<ompl::base::SubspaceProjectionEvaluator, ompl::base::ProjectionEvaluator>(m, "SubspaceProjectionEvaluato"
                                                                                            "r")
        .def(nb::init<const ompl::base::StateSpace *, unsigned int, ompl::base::ProjectionEvaluatorPtr>(),
             nb::arg("space"), nb::arg("index"), nb::arg("projToUse") = ompl::base::ProjectionEvaluatorPtr())
        .def("setup", &ompl::base::SubspaceProjectionEvaluator::setup)
        .def("getDimension", &ompl::base::SubspaceProjectionEvaluator::getDimension)
        .def("project", &ompl::base::SubspaceProjectionEvaluator::project, nb::arg("state"), nb::arg("projection"));
}
