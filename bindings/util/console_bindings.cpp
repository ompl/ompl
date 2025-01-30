#include <nanobind/nanobind.h>
// TODO
// Need this file???
namespace nb = nanobind;

void init_console(nb::module_& m) {
    // Expose logging macros as functions
    m.def("log_error", [](const std::string& message) {
        OMPL_ERROR("%s", message.c_str());
    });

    m.def("log_warn", [](const std::string& message) {
        OMPL_WARN("%s", message.c_str());
    });

    m.def("log_info", [](const std::string& message) {
        OMPL_INFORM("%s", message.c_str());
    });

    m.def("log_debug", [](const std::string& message) {
        OMPL_DEBUG("%s", message.c_str());
    });

    // Expose LogLevel enum
    nb::enum_<ompl::msg::LogLevel>(m, "LogLevel")
        .value("LOG_DEV2", ompl::msg::LogLevel::LOG_DEV2)
        .value("LOG_DEV1", ompl::msg::LogLevel::LOG_DEV1)
        .value("LOG_DEBUG", ompl::msg::LogLevel::LOG_DEBUG)
        .value("LOG_INFO", ompl::msg::LogLevel::LOG_INFO)
        .value("LOG_WARN", ompl::msg::LogLevel::LOG_WARN)
        .value("LOG_ERROR", ompl::msg::LogLevel::LOG_ERROR)
        .value("LOG_NONE", ompl::msg::LogLevel::LOG_NONE);

    // Expose OutputHandler class
    nb::class_<ompl::msg::OutputHandler>(m, "OutputHandler")
        .def("log", &ompl::msg::OutputHandler::log);
}