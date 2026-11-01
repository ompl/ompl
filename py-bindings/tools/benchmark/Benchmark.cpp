#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/chrono.h>
#include <algorithm>
#include <sstream>
#include <stdexcept>

#include "ompl/tools/benchmark/Benchmark.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/control/SimpleSetup.h"
#include "ompl/base/Planner.h"
#include "ompl/util/Console.h"
#include "PyGC.h"
#include "../init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;
namespace ot = ompl::tools;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

namespace
{
    struct BenchmarkAccess : ot::Benchmark
    {
        using ot::Benchmark::planners_;
        using ot::Benchmark::plannerSwitch_;
        using ot::Benchmark::postRun_;
        using ot::Benchmark::preRun_;
    };

    constexpr auto plannersOf = &BenchmarkAccess::planners_;
    constexpr auto plannerSwitchOf = &BenchmarkAccess::plannerSwitch_;
    constexpr auto preRunOf = &BenchmarkAccess::preRun_;
    constexpr auto postRunOf = &BenchmarkAccess::postRun_;

    /// Passes the callback a copy of the run's properties and writes the copy back: nanobind would hand the
    /// callback a throwaway copy of the reference, and a view would dangle once the run is stored.
    struct PostRunEvent
    {
        gc::SharedRef fn;

        void operator()(const ob::PlannerPtr &planner, ot::Benchmark::RunProperties &run) const
        {
            nb::gil_scoped_acquire gil;
            nb::handle callable = fn.get();
            if (!callable.is_valid())
                throw std::runtime_error("the Python callback is no longer set");
            nb::object props = nb::cast(run, nb::rv_policy::copy);
            callable(planner, props);
            run = nb::cast<const ot::Benchmark::RunProperties &>(props);
        }
    };

    /// benchmark() swaps in its own handler and on return leaves OMPL's previous slot pointing at it after it
    /// has been freed, so both slots are put back.
    class OutputHandlerSlots
    {
    public:
        OutputHandlerSlots()
        {
            current_ = ompl::msg::getOutputHandler();
            ompl::msg::restorePreviousOutputHandler();
            previous_ = ompl::msg::getOutputHandler();
            ompl::msg::restorePreviousOutputHandler();
        }
        OutputHandlerSlots(const OutputHandlerSlots &) = delete;
        OutputHandlerSlots &operator=(const OutputHandlerSlots &) = delete;
        ~OutputHandlerSlots()
        {
            ompl::msg::useOutputHandler(previous_);
            ompl::msg::useOutputHandler(current_);
        }

    private:
        ompl::msg::OutputHandler *current_;
        ompl::msg::OutputHandler *previous_;
    };

    int benchmarkTraverse(PyObject *self, visitproc visit, void *arg)
    {
        Py_VISIT(Py_TYPE(self));
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;
        const ot::Benchmark &b = *nb::inst_ptr<ot::Benchmark>(self);
        for (const ob::PlannerPtr &planner : b.*plannersOf)
            if (int rc = gc::visitHeld(planner, visit, arg))
                return rc;
        if (int rc = gc::visitHeld(b.*plannerSwitchOf, visit, arg))
            return rc;
        if (int rc = gc::visitHeld(b.*preRunOf, visit, arg))
            return rc;
        if (const auto *event = (b.*postRunOf).target<PostRunEvent>())
            if (nb::handle held = event->fn.exclusive(); held.is_valid())
                Py_VISIT(held.ptr());
        return 0;
    }

    int benchmarkClear(PyObject *self)
    {
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;
        ot::Benchmark &b = *nb::inst_ptr<ot::Benchmark>(self);
        auto &planners = b.*plannersOf;
        planners.erase(std::remove_if(planners.begin(), planners.end(),
                                      [](const ob::PlannerPtr &p) { return gc::holdsPython(p); }),
                       planners.end());
        if (gc::holdsPython(b.*plannerSwitchOf))
            b.setPlannerSwitchEvent(ot::Benchmark::PreSetupEvent());
        if (gc::holdsPython(b.*preRunOf))
            b.setPreRunEvent(ot::Benchmark::PreSetupEvent());
        if ((b.*postRunOf).target<PostRunEvent>() != nullptr)
            b.setPostRunEvent(ot::Benchmark::PostSetupEvent());
        return 0;
    }

    PyType_Slot benchmarkSlots[] = {{Py_tp_traverse, (void *)benchmarkTraverse},
                                    {Py_tp_clear, (void *)benchmarkClear},
                                    {0, nullptr}};
}  // namespace

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
    nb::class_<ot::Benchmark>(m, "Benchmark", nb::type_slots(benchmarkSlots))
        // Benchmark keeps a raw pointer to the setup.
        .def(nb::init<og::SimpleSetup &, const std::string &>(), nb::arg("setup"), nb::arg("name") = std::string(),
             nb::keep_alive<1, 2>())
        .def(nb::init<oc::SimpleSetup &, const std::string &>(), nb::arg("setup"), nb::arg("name") = std::string(),
             nb::keep_alive<1, 2>())

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
        .def(
            "setPlannerSwitchEvent", [](ot::Benchmark &b, nb::callable event)
            { b.setPlannerSwitchEvent(gc::function<ot::Benchmark::PreSetupEvent>(std::move(event))); },
            nb::arg("event"))
        .def(
            "setPlannerSwitchEvent", [](ot::Benchmark &b, std::nullptr_t)
            { b.setPlannerSwitchEvent(ot::Benchmark::PreSetupEvent()); }, nb::arg("event").none())
        .def(
            "setPreRunEvent", [](ot::Benchmark &b, nb::callable event)
            { b.setPreRunEvent(gc::function<ot::Benchmark::PreSetupEvent>(std::move(event))); }, nb::arg("event"))
        .def(
            "setPreRunEvent", [](ot::Benchmark &b, std::nullptr_t)
            { b.setPreRunEvent(ot::Benchmark::PreSetupEvent()); }, nb::arg("event").none())
        .def(
            "setPostRunEvent", [](ot::Benchmark &b, nb::callable event)
            { b.setPostRunEvent(PostRunEvent{gc::SharedRef(std::move(event))}); }, nb::arg("event"))
        .def(
            "setPostRunEvent", [](ot::Benchmark &b, std::nullptr_t)
            { b.setPostRunEvent(ot::Benchmark::PostSetupEvent()); }, nb::arg("event").none())

        // Runs planners on threads of their own and joins them, and every Python callback it reaches takes the GIL.
        .def(
            "benchmark",
            [](ot::Benchmark &b, const ot::Benchmark::Request &req)
            {
                nb::gil_scoped_release release;
                OutputHandlerSlots slots;
                b.benchmark(req);
            },
            nb::arg("req"))

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
