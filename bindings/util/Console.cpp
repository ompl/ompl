#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include "ompl/util/Console.h"
#include "init.hh"

namespace nb = nanobind;
namespace om = ompl;
void ompl::binding::util::init_Console(nb::module_& m)
{
    // #define OMPL_ERROR(fmt, ...) ompl::msg::log(__FILE__, __LINE__, ompl::msg::LOG_ERROR, fmt, ##__VA_ARGS__)
    m.def("OMPL_ERROR",
          [](const std::string &msg) {
              om::msg::log(__FILE__, __LINE__, om::msg::LOG_ERROR, "%s", msg.c_str());
          },
          nb::arg("message"));
    // #define OMPL_WARN(fmt, ...) ompl::msg::log(__FILE__, __LINE__, ompl::msg::LOG_WARN, fmt, ##__VA_ARGS__)
    m.def("OMPL_WARN",
          [](const std::string &msg) {
              om::msg::log(__FILE__, __LINE__, om::msg::LOG_WARN, "%s", msg.c_str());
          },
          nb::arg("message"));
    // #define OMPL_INFORM(fmt, ...) ompl::msg::log(__FILE__, __LINE__, ompl::msg::LOG_INFO, fmt, ##__VA_ARGS__)
    m.def("OMPL_INFORM",
          [](const std::string &msg) {
              om::msg::log(__FILE__, __LINE__, om::msg::LOG_INFO, "%s", msg.c_str());
          },
          nb::arg("message"));
    // #define OMPL_DEBUG(fmt, ...) ompl::msg::log(__FILE__, __LINE__, ompl::msg::LOG_DEBUG, fmt, ##__VA_ARGS__)
    m.def("OMPL_DEBUG",
          [](const std::string &msg) {
              om::msg::log(__FILE__, __LINE__, om::msg::LOG_DEBUG, "%s", msg.c_str());
          },
          nb::arg("message"));
    // #define OMPL_DEVMSG1(fmt, ...) ompl::msg::log(__FILE__, __LINE__, ompl::msg::LOG_DEV1, fmt, ##__VA_ARGS__)
    m.def("OMPL_DEVMSG1",
          [](const std::string &msg) {
              om::msg::log(__FILE__, __LINE__, om::msg::LOG_DEV1, "%s", msg.c_str());
          },
          nb::arg("message"));
    // #define OMPL_DEVMSG2(fmt, ...) ompl::msg::log(__FILE__, __LINE__, ompl::msg::LOG_DEV2, fmt, ##__VA_ARGS__)
    m.def("OMPL_DEVMSG2",
          [](const std::string &msg) {
              om::msg::log(__FILE__, __LINE__, om::msg::LOG_DEV2, "%s", msg.c_str());
          },
          nb::arg("message"));
}
