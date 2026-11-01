#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/function.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>

#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Time.h"
#include "PyGC.h"
#include "init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;
namespace ob = ompl::base;

namespace
{
    /// Runs on the asking thread, not on OMPL's eval thread, whose join in ~PTC deadlocks when the GIL is held.
    class PeriodicPythonCondition
    {
    public:
        PeriodicPythonCondition(nb::object fn, double period)
          : state_(std::make_shared<State>(gc::SharedRef(std::move(fn)), period))
        {
        }

        bool operator()() const
        {
            State &s = *state_;
            std::unique_lock<std::mutex> lock(s.mutex, std::try_to_lock);
            // Another thread is evaluating, possibly waiting for the GIL this caller holds.
            if (!lock.owns_lock())
                return s.value.load();
            const auto now = std::chrono::steady_clock::now();
            if (!s.evaluated || now - s.last >= s.period)
            {
                s.value = gc::call<bool>(s.fn);
                s.last = now;
                s.evaluated = true;
            }
            return s.value.load();
        }

    private:
        struct State
        {
            State(gc::SharedRef f, double seconds)
              : fn(std::move(f))
              , period(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(std::max(seconds, 0.0))))
            {
            }

            gc::SharedRef fn;
            const std::chrono::steady_clock::duration period;
            std::mutex mutex;
            std::chrono::steady_clock::time_point last;
            bool evaluated{false};
            std::atomic<bool> value{false};
        };

        std::shared_ptr<State> state_;
    };
}  // namespace

void ompl::binding::base::init_PlannerTerminationCondition(nb::module_ &m)
{
    // Bind the PlannerTerminationCondition class.
    nb::class_<ob::PlannerTerminationCondition>(m, "PlannerTerminationCondition")
        .def(nb::init<const std::function<bool()> &>(), nb::arg("fn"))
        .def(
            "__init__", [](ob::PlannerTerminationCondition *self, nb::callable fn, double period)
            { new (self) ob::PlannerTerminationCondition(PeriodicPythonCondition(std::move(fn), period)); },
            nb::arg("fn"), nb::arg("period"))
        .def("__call__", &ob::PlannerTerminationCondition::operator())
        .def("terminate", &ob::PlannerTerminationCondition::terminate)
        .def("eval", &ob::PlannerTerminationCondition::eval);

    // Bind non-member termination condition functions.
    m.def("plannerNonTerminatingCondition", &ob::plannerNonTerminatingCondition);

    m.def("plannerAlwaysTerminatingCondition", &ob::plannerAlwaysTerminatingCondition);

    m.def("plannerOrTerminationCondition", &ob::plannerOrTerminationCondition, nb::arg("c1"), nb::arg("c2"));

    m.def("plannerAndTerminationCondition", &ob::plannerAndTerminationCondition, nb::arg("c1"), nb::arg("c2"));

    // Overloads for timedPlannerTerminationCondition.
    m.def("timedPlannerTerminationCondition", nb::overload_cast<double>(&ob::timedPlannerTerminationCondition),
          nb::arg("duration"));

    m.def("timedPlannerTerminationCondition", nb::overload_cast<double, double>(&ob::timedPlannerTerminationCondition),
          nb::arg("duration"), nb::arg("interval"));

    m.def("exactSolnPlannerTerminationCondition", &ob::exactSolnPlannerTerminationCondition, nb::arg("pdef"));
}
