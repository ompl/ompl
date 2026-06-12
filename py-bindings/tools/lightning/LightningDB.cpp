#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "ompl/tools/lightning/LightningDB.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace ot = ompl::tools;

void ompl::binding::tools::initLightning_LightningDB(nb::module_ &m)
{
    nb::class_<ot::LightningDB>(m, "LightningDB")
        .def(nb::init<const ob::StateSpacePtr &>(), nb::arg("space"))
        .def("load", &ot::LightningDB::load, nb::arg("fileName"))
        .def("save", &ot::LightningDB::save, nb::arg("fileName"))
        .def("saveIfChanged", &ot::LightningDB::saveIfChanged, nb::arg("fileName"))
        .def(
            "addPath",
            [](ot::LightningDB &db, ompl::geometric::PathGeometric &path)
            {
                double insertionTime = 0.0;
                db.addPath(path, insertionTime);
                return insertionTime;
            },
            nb::arg("solutionPath"))
        .def("getExperiencesCount", &ot::LightningDB::getExperiencesCount)
        .def("getStatesCount", &ot::LightningDB::getStatesCount)
        .def("getNumUnsavedPaths", &ot::LightningDB::getNumUnsavedPaths)
        .def("isEmpty", &ot::LightningDB::isEmpty)
        .def(
            "findNearestStartGoal", &ot::LightningDB::findNearestStartGoal, nb::arg("nearestK"), nb::arg("start"),
            nb::arg("goal"));
}
