#include <nanobind/nanobind.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>
#include <sstream>

#include "ompl/base/GenericParam.h"
#include "PyGC.h"
#include "init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;
namespace ob = ompl::base;

namespace
{
    /// Holds the setter and getter in members so the collector can see them; see PyGC.h. Neither is
    /// recoverable from SpecificParam, which stores them in private std::functions.
    template <typename T>
    struct PySpecificParam : ob::SpecificParam<T>
    {
        nb::object setter;
        nb::object getter;

        PySpecificParam(const std::string &name, nb::object set, nb::object get)
          : ob::SpecificParam<T>(
                name, [this](T value) { setter(value); },
                get.is_none() ? typename ob::SpecificParam<T>::GetterFn() :
                                typename ob::SpecificParam<T>::GetterFn([this] { return nb::cast<T>(getter()); }))
          , setter(std::move(set))
          , getter(std::move(get))
        {
        }

        // The stored setter and getter capture `this`.
        PySpecificParam(const PySpecificParam &) = delete;
        PySpecificParam &operator=(const PySpecificParam &) = delete;
    };

    template <typename T>
    void bindParam(nb::module_ &m, const char *name)
    {
        using Param = PySpecificParam<T>;
        nb::class_<Param, ob::GenericParam>(m, name, nb::type_slots(gc::gcSlots<Param, &Param::setter, &Param::getter>))
            .def(
                "__init__", [](Param *self, const std::string &paramName, nb::callable setter, nb::object getter)
                { new (self) Param(paramName, std::move(setter), std::move(getter)); }, nb::arg("name"),
                nb::arg("setter"), nb::arg("getter") = nb::none());
    }
}  // namespace

void ompl::binding::base::init_GenericParam(nb::module_ &m)
{
    struct PyGenericParam : ob::GenericParam
    {
        NB_TRAMPOLINE(ob::GenericParam, 2);

        bool setValue(const std::string &value) override
        {
            NB_OVERRIDE_PURE(setValue, value);
        }

        std::string getValue() const override
        {
            NB_OVERRIDE_PURE(getValue);
        }
    };

    nb::class_<ob::GenericParam, PyGenericParam>(m, "GenericParam")
        .def(nb::init<std::string>(), nb::arg("name"))
        .def("getName", &ob::GenericParam::getName)
        .def("setName", &ob::GenericParam::setName, nb::arg("name"))
        .def("setValue", &ob::GenericParam::setValue, nb::arg("value"))
        .def("getValue", &ob::GenericParam::getValue)
        .def("setRangeSuggestion", &ob::GenericParam::setRangeSuggestion, nb::arg("rangeSuggestion"))
        .def("getRangeSuggestion", &ob::GenericParam::getRangeSuggestion);

    bindParam<bool>(m, "BoolParam");
    bindParam<int>(m, "IntParam");
    bindParam<double>(m, "DoubleParam");
    bindParam<std::string>(m, "StringParam");

    nb::class_<ob::ParamSet>(m, "ParamSet")
        .def(nb::init<>())
        .def("add", &ob::ParamSet::add, nb::arg("param"))
        .def("remove", &ob::ParamSet::remove, nb::arg("name"))
        .def("include", &ob::ParamSet::include, nb::arg("other"), nb::arg("prefix") = "")
        .def("setParam", &ob::ParamSet::setParam, nb::arg("key"), nb::arg("value"))
        .def(
            "getParam",
            [](const ob::ParamSet &ps, const std::string &key)
            {
                std::string value;
                if (!ps.getParam(key, value))
                    return std::optional<std::string>{};
                return std::optional<std::string>{value};
            },
            nb::arg("key"))
        .def("setParams", &ob::ParamSet::setParams, nb::arg("kv"), nb::arg("ignoreUnknown") = false)
        .def("getParams",
             [](const ob::ParamSet &ps)
             {
                 std::map<std::string, std::string> params;
                 ps.getParams(params);
                 return params;
             })
        .def("getParamNames",
             [](const ob::ParamSet &ps)
             {
                 std::vector<std::string> names;
                 ps.getParamNames(names);
                 return names;
             })
        .def("getParamValues",
             [](const ob::ParamSet &ps)
             {
                 std::vector<std::string> vals;
                 ps.getParamValues(vals);
                 return vals;
             })
        .def("hasParam", &ob::ParamSet::hasParam, nb::arg("key"))
        .def("size", &ob::ParamSet::size)
        .def("clear", &ob::ParamSet::clear)
        .def("__repr__",
             [](const ob::ParamSet &ps)
             {
                 std::ostringstream oss;
                 ps.print(oss);
                 return oss.str();
             });
}
