#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "ompl/tools/thunder/ThunderDB.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace ot = ompl::tools;

void ompl::binding::tools::initThunder_ThunderDB(nb::module_ &m)
{
    nb::class_<ot::ThunderDB>(m, "ThunderDB")
        .def(nb::init<const ob::StateSpacePtr &>(), nb::arg("space"))
        .def("load", &ot::ThunderDB::load, nb::arg("fileName"))
        .def("save", &ot::ThunderDB::save, nb::arg("fileName"))
        .def("saveIfChanged", &ot::ThunderDB::saveIfChanged, nb::arg("fileName"))
        .def(
            "addPath",
            [](ot::ThunderDB &db, ompl::geometric::PathGeometric &path)
            {
                double insertionTime = 0.0;
                const bool ok = db.addPath(path, insertionTime);
                return nb::make_tuple(ok, insertionTime);
            },
            nb::arg("solutionPath"))
        .def(
            "setSPARSdb", [](ot::ThunderDB &db, ot::SPARSdbPtr prm) { db.setSPARSdb(prm); }, nb::arg("prm"))
        .def("getSPARSdb", &ot::ThunderDB::getSPARSdb, nb::rv_policy::reference_internal);
}
