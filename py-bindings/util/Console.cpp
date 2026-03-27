#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include "ompl/util/Console.h"
#include "init.h"

namespace nb = nanobind;
namespace om = ompl;

void ompl::binding::util::init_Console(nb::module_ &m)
{
    // Bind the LogLevel enum
    nb::enum_<om::msg::LogLevel>(m, "LogLevel")
        .value("LOG_DEV2", om::msg::LOG_DEV2)
        .value("LOG_DEV1", om::msg::LOG_DEV1)
        .value("LOG_DEBUG", om::msg::LOG_DEBUG)
        .value("LOG_INFO", om::msg::LOG_INFO)
        .value("LOG_WARN", om::msg::LOG_WARN)
        .value("LOG_ERROR", om::msg::LOG_ERROR)
        .value("LOG_NONE", om::msg::LOG_NONE)
        .export_values();

    // Bind OutputHandler base class
    nb::class_<om::msg::OutputHandler>(m, "OutputHandler")
        .def("log", &om::msg::OutputHandler::log, nb::arg("text"), nb::arg("level"), nb::arg("filename"),
             nb::arg("line"));

    // Bind OutputHandlerSTD
    nb::class_<om::msg::OutputHandlerSTD, om::msg::OutputHandler>(m, "OutputHandlerSTD")
        .def(nb::init<>())
        .def("log", &om::msg::OutputHandlerSTD::log, nb::arg("text"), nb::arg("level"), nb::arg("filename"),
             nb::arg("line"));

    // Bind OutputHandlerFile
    nb::class_<om::msg::OutputHandlerFile, om::msg::OutputHandler>(m, "OutputHandlerFile")
        .def(nb::init<const char *>(), nb::arg("filename"))
        .def("log", &om::msg::OutputHandlerFile::log, nb::arg("text"), nb::arg("level"), nb::arg("filename"),
             nb::arg("line"));

    // Bind logging control functions
    m.def("setLogLevel", &om::msg::setLogLevel, nb::arg("level"));
    m.def("getLogLevel", &om::msg::getLogLevel);
    m.def("noOutputHandler", &om::msg::noOutputHandler);
    m.def("restorePreviousOutputHandler", &om::msg::restorePreviousOutputHandler);
    m.def("useOutputHandler", &om::msg::useOutputHandler, nb::arg("oh"));
    m.def("getOutputHandler", &om::msg::getOutputHandler, nb::rv_policy::reference);

    // Bind logging macros as functions
    m.def(
        "OMPL_ERROR", [](const std::string &msg)
        { om::msg::log(__FILE__, __LINE__, om::msg::LOG_ERROR, "%s", msg.c_str()); }, nb::arg("message"));
    m.def(
        "OMPL_WARN", [](const std::string &msg)
        { om::msg::log(__FILE__, __LINE__, om::msg::LOG_WARN, "%s", msg.c_str()); }, nb::arg("message"));
    m.def(
        "OMPL_INFORM", [](const std::string &msg)
        { om::msg::log(__FILE__, __LINE__, om::msg::LOG_INFO, "%s", msg.c_str()); }, nb::arg("message"));
    m.def(
        "OMPL_DEBUG", [](const std::string &msg)
        { om::msg::log(__FILE__, __LINE__, om::msg::LOG_DEBUG, "%s", msg.c_str()); }, nb::arg("message"));
    m.def(
        "OMPL_DEVMSG1", [](const std::string &msg)
        { om::msg::log(__FILE__, __LINE__, om::msg::LOG_DEV1, "%s", msg.c_str()); }, nb::arg("message"));
    m.def(
        "OMPL_DEVMSG2", [](const std::string &msg)
        { om::msg::log(__FILE__, __LINE__, om::msg::LOG_DEV2, "%s", msg.c_str()); }, nb::arg("message"));
}
