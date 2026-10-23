#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/geometric/GeneticSearch.h"
#include "ompl/base/goals/GoalRegion.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::init_GeneticSearch(nb::module_ &m)
{
    nb::class_<og::GeneticSearch>(m, "GeneticSearch")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def(
            "solve",
            [](og::GeneticSearch &self, double solveTime, const ob::GoalRegion &goal, ob::State *result,
               nb::object hint)
            {
                if (hint.is_none())
                    return self.solve(solveTime, goal, result);
                return self.solve(solveTime, goal, result, nb::cast<std::vector<ob::State *>>(hint));
            },
            nb::arg("solveTime"), nb::arg("goal"), nb::arg("result"), nb::arg("hint") = nb::none())
        .def("setMaxImproveSteps", &og::GeneticSearch::setMaxImproveSteps, nb::arg("maxSteps"))
        .def("getMaxImproveSteps", &og::GeneticSearch::getMaxImproveSteps)
        .def("setValidityCheck", &og::GeneticSearch::setValidityCheck, nb::arg("valid"))
        .def("getValidityCheck", &og::GeneticSearch::getValidityCheck)
        .def("setTryImprove", &og::GeneticSearch::setTryImprove, nb::arg("flag"))
        .def("getTryImprove", &og::GeneticSearch::getTryImprove)
        .def("setPoolSize", &og::GeneticSearch::setPoolSize, nb::arg("size"))
        .def("getPoolSize", &og::GeneticSearch::getPoolSize)
        .def("setPoolMutationSize", &og::GeneticSearch::setPoolMutationSize, nb::arg("size"))
        .def("getPoolMutationSize", &og::GeneticSearch::getPoolMutationSize)
        .def("setPoolRandomSize", &og::GeneticSearch::setPoolRandomSize, nb::arg("size"))
        .def("getPoolRandomSize", &og::GeneticSearch::getPoolRandomSize)
        .def("setRange", &og::GeneticSearch::setRange, nb::arg("distance"))
        .def("getRange", &og::GeneticSearch::getRange)
        .def("clear", &og::GeneticSearch::clear);
}
