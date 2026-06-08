#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/chrono.h>
#include <sstream>

#include "ompl/tools/benchmark/Benchmark.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/control/SimpleSetup.h"
#include "ompl/base/Planner.h"
#include "../init.h"

namespace nb = nanobind;
namespace ot = ompl::tools;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

void ompl::binding::tools::initBenchmark_Benchmark(nb::module_ &m)
{
    // Bind Status struct
    nb::class_<ot::Benchmark::Status>(m, "Status")
        .def(nb::init<>())
        .def_rw("running", &ot::Benchmark::Status::running)
        .def_rw("activeRun", &ot::Benchmark::Status::activeRun)
        .def_rw("progressPercentage", &ot::Benchmark::Status::progressPercentage)
        .def_rw("activePlanner", &ot::Benchmark::Status::activePlanner);

    // Bind RunProperties struct (inherits from std::map<std::string, std::string>)
    // explicit std::map base removed to avoid conflict with default type casters
    nb::class_<ot::Benchmark::RunProperties>(m, "RunProperties")
        .def(nb::init<>())
        .def("__getitem__",
             [](const ot::Benchmark::RunProperties &p, const std::string &key)
             {
                 auto it = p.find(key);
                 if (it == p.end())
                     throw nb::key_error();
                 return it->second;
             })
        .def("__setitem__",
             [](ot::Benchmark::RunProperties &p, const std::string &key, const std::string &val) { p[key] = val; })
        .def("__len__", &ot::Benchmark::RunProperties::size)
        .def("__contains__",
             [](const ot::Benchmark::RunProperties &p, const std::string &key) { return p.find(key) != p.end(); })
        .def(
            "to_dict", [](const ot::Benchmark::RunProperties &p) -> std::map<std::string, std::string> { return p; },
            "Convert to a Python dictionary copy.");

    // Bind PlannerExperiment struct
    nb::class_<ot::Benchmark::PlannerExperiment>(m, "PlannerExperiment")
        .def(nb::init<>())
        .def_rw("name", &ot::Benchmark::PlannerExperiment::name)
        .def_rw("runs", &ot::Benchmark::PlannerExperiment::runs)
        .def_rw("progressPropertyNames", &ot::Benchmark::PlannerExperiment::progressPropertyNames)
        .def_rw("runsProgressData", &ot::Benchmark::PlannerExperiment::runsProgressData)
        .def_rw("common", &ot::Benchmark::PlannerExperiment::common)
        .def("__eq__", &ot::Benchmark::PlannerExperiment::operator==, nb::is_operator());

    // Bind CompleteExperiment struct
    nb::class_<ot::Benchmark::CompleteExperiment>(m, "CompleteExperiment")
        .def(nb::init<>())
        .def_rw("name", &ot::Benchmark::CompleteExperiment::name)
        .def_rw("planners", &ot::Benchmark::CompleteExperiment::planners)
        .def_rw("maxTime", &ot::Benchmark::CompleteExperiment::maxTime)
        .def_rw("maxMem", &ot::Benchmark::CompleteExperiment::maxMem)
        .def_rw("runCount", &ot::Benchmark::CompleteExperiment::runCount)
        .def_rw("startTime", &ot::Benchmark::CompleteExperiment::startTime)
        .def_rw("totalDuration", &ot::Benchmark::CompleteExperiment::totalDuration)
        .def_rw("setupInfo", &ot::Benchmark::CompleteExperiment::setupInfo)
        .def_rw("seed", &ot::Benchmark::CompleteExperiment::seed)
        .def_rw("host", &ot::Benchmark::CompleteExperiment::host)
        .def_rw("cpuInfo", &ot::Benchmark::CompleteExperiment::cpuInfo)
        .def_rw("parameters", &ot::Benchmark::CompleteExperiment::parameters);

    // Bind Request struct
    nb::class_<ot::Benchmark::Request>(m, "Request")
        .def(nb::init<double, double, unsigned int, double, bool, bool, bool>(), nb::arg("maxTime") = 5.0,
             nb::arg("maxMem") = 4096.0, nb::arg("runCount") = 100, nb::arg("timeBetweenUpdates") = 0.05,
             nb::arg("displayProgress") = true, nb::arg("saveConsoleOutput") = true, nb::arg("simplify") = true)
        .def_rw("maxTime", &ot::Benchmark::Request::maxTime)
        .def_rw("maxMem", &ot::Benchmark::Request::maxMem)
        .def_rw("runCount", &ot::Benchmark::Request::runCount)
        .def_rw("timeBetweenUpdates", &ot::Benchmark::Request::timeBetweenUpdates)
        .def_rw("displayProgress", &ot::Benchmark::Request::displayProgress)
        .def_rw("saveConsoleOutput", &ot::Benchmark::Request::saveConsoleOutput)
        .def_rw("simplify", &ot::Benchmark::Request::simplify);

    // Bind Benchmark class
    nb::class_<ot::Benchmark>(m, "Benchmark")
        // Constructors
        .def(nb::init<og::SimpleSetup &, const std::string &>(), nb::arg("setup"), nb::arg("name") = std::string())
        .def(nb::init<oc::SimpleSetup &, const std::string &>(), nb::arg("setup"), nb::arg("name") = std::string())

        // Experiment parameter methods
        .def("addExperimentParameter", &ot::Benchmark::addExperimentParameter, nb::arg("name"), nb::arg("type"),
             nb::arg("value"))
        .def("getExperimentParameters", &ot::Benchmark::getExperimentParameters, nb::rv_policy::reference_internal)
        .def("numExperimentParameters", &ot::Benchmark::numExperimentParameters)

        // Experiment name methods
        .def("setExperimentName", &ot::Benchmark::setExperimentName, nb::arg("name"))
        .def("getExperimentName", &ot::Benchmark::getExperimentName, nb::rv_policy::reference_internal)

        // Planner management methods
        .def("addPlanner", &ot::Benchmark::addPlanner, nb::arg("planner"))
        .def("addPlannerAllocator", &ot::Benchmark::addPlannerAllocator, nb::arg("pa"))
        .def("clearPlanners", &ot::Benchmark::clearPlanners)

        // Event setters
        .def("setPlannerSwitchEvent", &ot::Benchmark::setPlannerSwitchEvent, nb::arg("event"))
        .def("setPreRunEvent", &ot::Benchmark::setPreRunEvent, nb::arg("event"))
        .def("setPostRunEvent", &ot::Benchmark::setPostRunEvent, nb::arg("event"))

        // Benchmark execution
        .def("benchmark", &ot::Benchmark::benchmark, nb::arg("req"))

        // Status and data access
        .def("getStatus", &ot::Benchmark::getStatus, nb::rv_policy::reference_internal)
        .def("getRecordedExperimentData", &ot::Benchmark::getRecordedExperimentData, nb::rv_policy::reference_internal)

        // Save results methods
        .def(
            "saveResultsToStream", [](const ot::Benchmark &self) { return self.saveResultsToStream(std::cout); },
            "Save the results of the benchmark to stdout.")
        .def("saveResultsToFile", nb::overload_cast<const char *>(&ot::Benchmark::saveResultsToFile, nb::const_),
             nb::arg("filename"), "Save the results of the benchmark to a file.")
        .def("saveResultsToFile", nb::overload_cast<>(&ot::Benchmark::saveResultsToFile, nb::const_),
             "Save the results of the benchmark to a file with auto-generated name.");
}
