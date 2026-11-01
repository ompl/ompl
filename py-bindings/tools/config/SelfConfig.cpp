#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <sstream>
#include <string>
#include <utility>

#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/tools/config/SelfConfig.h"
#include "PyGC.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace ot = ompl::tools;
namespace gc = ompl::binding::gc;

namespace
{
    // A SelfConfig points into a static map whose entry dies with the first shared_ptr it saw for the SI, which
    // for a Python-owned SI is a temporary. So the binding keeps the SI and builds a SelfConfig for each call.
    struct SelfConfigBinding
    {
        SelfConfigBinding(ob::SpaceInformationPtr si, std::string context)
          : si(std::move(si)), context(std::move(context))
        {
        }

        ot::SelfConfig get() const
        {
            return ot::SelfConfig(si, context);
        }

        ob::SpaceInformationPtr si;
        std::string context;
    };

    int selfConfigTraverse(PyObject *self, visitproc visit, void *arg)
    {
        Py_VISIT(Py_TYPE(self));
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;
        return gc::visitHeld(nb::inst_ptr<SelfConfigBinding>(self)->si, visit, arg);
    }

    int selfConfigClear(PyObject *self)
    {
        if (nb::inst_ready(self) && gc::pythonOwned(self))
        {
            auto &binding = *nb::inst_ptr<SelfConfigBinding>(self);
            if (gc::holdsPython(binding.si))
                binding.si.reset();
        }
        return 0;
    }

    PyType_Slot selfConfigSlots[] = {{Py_tp_traverse, (void *)selfConfigTraverse},
                                     {Py_tp_clear, (void *)selfConfigClear},
                                     {0, nullptr}};
}  // namespace

void ompl::binding::tools::initConfig_SelfConfig(nb::module_ &m)
{
    nb::class_<SelfConfigBinding>(m, "SelfConfig", nb::type_slots(selfConfigSlots))
        .def(nb::init<ob::SpaceInformationPtr, std::string>(), nb::arg("si"), nb::arg("context") = std::string())
        .def("getProbabilityOfValidState",
             [](const SelfConfigBinding &self) { return self.get().getProbabilityOfValidState(); })
        .def("getAverageValidMotionLength",
             [](const SelfConfigBinding &self) { return self.get().getAverageValidMotionLength(); })
        .def(
            "configureValidStateSamplingAttempts",
            [](const SelfConfigBinding &self, unsigned int attempts)
            {
                self.get().configureValidStateSamplingAttempts(attempts);
                return attempts;
            },
            nb::arg("attempts"))
        .def(
            "configurePlannerRange",
            [](const SelfConfigBinding &self, double range)
            {
                self.get().configurePlannerRange(range);
                return range;
            },
            nb::arg("range"))
        .def(
            "configureProjectionEvaluator",
            [](const SelfConfigBinding &self, ob::ProjectionEvaluatorPtr proj)
            {
                self.get().configureProjectionEvaluator(proj);
                return proj;
            },
            nb::arg("proj").none())
        .def("__repr__",
             [](const SelfConfigBinding &self)
             {
                 std::ostringstream oss;
                 self.get().print(oss);
                 return oss.str();
             })
        .def_static("getDefaultPlanner", &ot::SelfConfig::getDefaultPlanner, nb::arg("goal"));
}
