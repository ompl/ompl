#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>
#include <sstream>

#include "ompl/base/GenericParam.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

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

    nb::class_<ob::SpecificParam<bool>, ob::GenericParam>(m, "BoolParam")
        .def(nb::init<const std::string &, ob::SpecificParam<bool>::SetterFn, ob::SpecificParam<bool>::GetterFn>(),
             nb::arg("name"), nb::arg("setter"), nb::arg("getter") = ob::SpecificParam<bool>::GetterFn());

    nb::class_<ob::SpecificParam<int>, ob::GenericParam>(m, "IntParam")
        .def(nb::init<const std::string &, ob::SpecificParam<int>::SetterFn, ob::SpecificParam<int>::GetterFn>(),
             nb::arg("name"), nb::arg("setter"), nb::arg("getter") = ob::SpecificParam<int>::GetterFn());

    nb::class_<ob::SpecificParam<double>, ob::GenericParam>(m, "DoubleParam")
        .def(nb::init<const std::string &, ob::SpecificParam<double>::SetterFn, ob::SpecificParam<double>::GetterFn>(),
             nb::arg("name"), nb::arg("setter"), nb::arg("getter") = ob::SpecificParam<double>::GetterFn());

    nb::class_<ob::SpecificParam<std::string>, ob::GenericParam>(m, "StringParam")
        .def(nb::init<const std::string &, ob::SpecificParam<std::string>::SetterFn,
                      ob::SpecificParam<std::string>::GetterFn>(),
             nb::arg("name"), nb::arg("setter"), nb::arg("getter") = ob::SpecificParam<std::string>::GetterFn());

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
        .def("print",
             [](const ob::ParamSet &ps)
             {
                 std::ostringstream oss;
                 ps.print(oss);
                 return oss.str();
             });
}
