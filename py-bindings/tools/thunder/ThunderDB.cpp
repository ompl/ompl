#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <memory>
#include <vector>

#include "ompl/base/PlannerData.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/tools/thunder/ThunderDB.h"
#include "ompl/util/Exception.h"
#include "PlannerSolve.h"
#include "PyGC.h"
#include "../init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;

namespace
{
    og::SPARSdb &requireSPARSdb(ot::ThunderDB &db)
    {
        const ot::SPARSdbPtr &spars = db.getSPARSdb();
        if (!spars)
            throw ompl::Exception("ThunderDB has no SPARSdb; call setSPARSdb() first");
        return *spars;
    }

    int thunderTraverse(PyObject *self, visitproc visit, void *arg)
    {
        Py_VISIT(Py_TYPE(self));
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;
        return gc::visitHeld(nb::inst_ptr<ot::ThunderDB>(self)->getSPARSdb(), visit, arg);
    }

    int thunderClear(PyObject *self)
    {
        if (nb::inst_ready(self) && gc::pythonOwned(self))
        {
            auto &db = *nb::inst_ptr<ot::ThunderDB>(self);
            if (gc::holdsPython(db.getSPARSdb()))
            {
                ot::SPARSdbPtr none;
                db.setSPARSdb(none);
            }
        }
        return 0;
    }

    PyType_Slot thunderSlots[] = {{Py_tp_traverse, (void *)thunderTraverse},
                                  {Py_tp_clear, (void *)thunderClear},
                                  {0, nullptr}};
}  // namespace

void ompl::binding::tools::initThunder_ThunderDB(nb::module_ &m)
{
    nb::class_<ot::ThunderDB>(m, "ThunderDB", nb::type_slots(thunderSlots))
        .def(nb::init<const ob::StateSpacePtr &>(), nb::arg("space"))
        .def("load", &ot::ThunderDB::load, nb::arg("fileName"))
        .def("save", &ot::ThunderDB::save, nb::arg("fileName"))
        .def("saveIfChanged", &ot::ThunderDB::saveIfChanged, nb::arg("fileName"))
        .def(
            "addPath",
            [](ot::ThunderDB &db, og::PathGeometric &path)
            {
                double insertionTime = 0.0;
                const bool ok = db.addPath(path, insertionTime);
                return nb::make_tuple(ok, insertionTime);
            },
            nb::arg("solutionPath"))
        .def("getAllPlannerDatas",
             [](const ot::ThunderDB &db)
             {
                 std::vector<ob::PlannerDataPtr> plannerDatas;
                 db.getAllPlannerDatas(plannerDatas);
                 return plannerDatas;
             })
        .def(
            "setSPARSdb", [](ot::ThunderDB &db, ot::SPARSdbPtr prm) { db.setSPARSdb(prm); }, nb::arg("prm"))
        .def("getSPARSdb", &ot::ThunderDB::getSPARSdb)
        .def(
            "findNearestStartGoal",
            [](ot::ThunderDB &db, int nearestK, const ob::State *start, const ob::State *goal, nb::handle ptc)
            {
                requireSPARSdb(db);
                const ob::PlannerTerminationCondition condition = ompl::binding::toTerminationCondition(ptc);
                og::SPARSdb::CandidateSolution candidate;
                bool found = false;
                {
                    nb::gil_scoped_release release;
                    found = db.findNearestStartGoal(nearestK, start, goal, candidate, condition);
                }
                return found ? std::dynamic_pointer_cast<og::PathGeometric>(candidate.path_) : nullptr;
            },
            nb::arg("nearestK"), nb::arg("start"), nb::arg("goal"), nb::arg("ptc"))
        .def("debugVertex", &ot::ThunderDB::debugVertex, nb::arg("vertex"))
        .def("debugState", &ot::ThunderDB::debugState, nb::arg("state"))
        .def("getNumPathsInserted", &ot::ThunderDB::getNumPathsInserted)
        .def("getSavingEnabled", &ot::ThunderDB::getSavingEnabled)
        .def("setSavingEnabled", &ot::ThunderDB::setSavingEnabled, nb::arg("savingEnabled"))
        .def("isEmpty", [](ot::ThunderDB &db) { return requireSPARSdb(db).getNumVertices() == 0u; });
}
