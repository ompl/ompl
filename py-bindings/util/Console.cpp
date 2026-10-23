#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/trampoline.h>

#include <algorithm>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "ompl/util/Console.h"
#include "init.h"

namespace nb = nanobind;
namespace om = ompl;

namespace
{
    // OMPL calls the handler with its console mutex held, on whichever thread logs. Calling Python there waits
    // for the GIL while a GIL holder may be waiting for that mutex, so a Python handler only queues the message
    // and it is delivered later with the GIL held and the mutex free.
    class LogDelivery
    {
    public:
        static void push(const om::msg::OutputHandler *handler, nb::handle self, const std::string &text,
                         om::msg::LogLevel level, const char *filename, int line)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopped_)
                return;
            queue_.push_back({handler, self, text, level, filename != nullptr ? filename : "", line});
            ++pushed_;
            if (!running_)
            {
                running_ = true;
                std::thread(&LogDelivery::run).detach();
            }
            cv_.notify_all();
        }

        static void forget(const om::msg::OutputHandler *handler)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto removed = std::remove_if(queue_.begin(), queue_.end(),
                                                [handler](const Entry &e) { return e.handler == handler; });
            consumed_ += static_cast<std::uint64_t>(queue_.end() - removed);
            queue_.erase(removed, queue_.end());
        }

        /// Delivers every message queued so far. Requires the GIL.
        static void flush()
        {
            std::unique_lock<std::mutex> lock(mutex_);
            const std::uint64_t target = pushed_;
            while (consumed_ < target)
            {
                if (!draining_)
                {
                    drainLocked(lock);
                    continue;
                }
                if (drainer_ == std::this_thread::get_id())
                    return;
                lock.unlock();
                {
                    nb::gil_scoped_release release;
                    std::unique_lock<std::mutex> wait(mutex_);
                    cv_.wait(wait, [target] { return consumed_ >= target || !draining_; });
                }
                lock.lock();
            }
        }

        /// Stops delivery for good and delivers what is left. Requires the GIL.
        static void shutdown()
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                stopped_ = true;
                cv_.notify_all();
            }
            {
                nb::gil_scoped_release release;
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [] { return !running_ && !draining_; });
            }
            std::unique_lock<std::mutex> lock(mutex_);
            if (!draining_)
                drainLocked(lock);
        }

    private:
        struct Entry
        {
            const om::msg::OutputHandler *handler;
            nb::handle self;
            std::string text;
            om::msg::LogLevel level;
            std::string filename;
            int line;
        };

        static void deliver(const Entry &e)
        {
            nb::object self = nb::borrow(e.self);
            try
            {
                self.attr("log")(e.text, e.level, e.filename, e.line);
            }
            catch (nb::python_error &err)
            {
                err.discard_as_unraisable(self);
            }
            catch (const std::exception &err)
            {
                PyErr_SetString(PyExc_RuntimeError, err.what());
                PyErr_WriteUnraisable(self.ptr());
            }
        }

        // Requires the GIL and the lock, and no other drainer.
        static void drainLocked(std::unique_lock<std::mutex> &lock)
        {
            draining_ = true;
            drainer_ = std::this_thread::get_id();
            while (!queue_.empty())
            {
                Entry e = std::move(queue_.front());
                queue_.pop_front();
                lock.unlock();
                deliver(e);
                lock.lock();
                ++consumed_;
            }
            draining_ = false;
            drainer_ = std::thread::id();
            cv_.notify_all();
        }

        static void run()
        {
            std::unique_lock<std::mutex> lock(mutex_);
            while (!stopped_)
            {
                if (queue_.empty() || draining_)
                {
                    cv_.wait(lock);
                    continue;
                }
                lock.unlock();
                {
                    nb::gil_scoped_acquire gil;
                    std::unique_lock<std::mutex> inner(mutex_);
                    if (!stopped_ && !draining_)
                        drainLocked(inner);
                }
                lock.lock();
            }
            running_ = false;
            cv_.notify_all();
        }

        static inline std::mutex mutex_;
        static inline std::condition_variable cv_;
        static inline std::deque<Entry> queue_;
        static inline std::uint64_t pushed_{0};
        static inline std::uint64_t consumed_{0};
        static inline bool draining_{false};
        static inline std::thread::id drainer_;
        static inline bool running_{false};
        static inline bool stopped_{false};
    };

    /// OMPL keeps raw pointers to the current and previous handler, so the Python objects behind them are kept
    /// alive here, mirroring the slot changes made from Python.
    class InstalledHandlers
    {
    public:
        static void use(nb::object handler)
        {
            previous_ = std::move(current_);
            current_ = std::move(handler);
        }

        static void restorePrevious()
        {
            std::swap(previous_, current_);
        }

        static void setFallback(om::msg::OutputHandler *fallback)
        {
            fallback_ = fallback;
        }

        /// Points both of OMPL's slots back at the handler that was current at import, then drops the references.
        static void clear()
        {
            if (current_.is_valid() || previous_.is_valid())
            {
                om::msg::useOutputHandler(fallback_);
                om::msg::useOutputHandler(fallback_);
            }
            current_.reset();
            previous_.reset();
        }

    private:
        static inline nb::object current_;
        static inline nb::object previous_;
        static inline om::msg::OutputHandler *fallback_{nullptr};
    };

    class PyOutputHandler : public om::msg::OutputHandler
    {
    public:
        NB_TRAMPOLINE(om::msg::OutputHandler, 1);

        ~PyOutputHandler() override
        {
            LogDelivery::forget(this);
        }

        void log(const std::string &text, om::msg::LogLevel level, const char *filename, int line) override
        {
            LogDelivery::push(this, nb_trampoline.base(), text, level, filename, line);
        }
    };

    class PyOutputHandlerSTD : public om::msg::OutputHandlerSTD
    {
    public:
        NB_TRAMPOLINE(om::msg::OutputHandlerSTD, 1);

        ~PyOutputHandlerSTD() override
        {
            LogDelivery::forget(this);
        }

        void log(const std::string &text, om::msg::LogLevel level, const char *filename, int line) override
        {
            LogDelivery::push(this, nb_trampoline.base(), text, level, filename, line);
        }
    };

    void logFromPython(om::msg::LogLevel level, const std::string &msg)
    {
        om::msg::log(__FILE__, __LINE__, level, "%s", msg.c_str());
        LogDelivery::flush();
    }
}  // namespace

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

    // These log() bindings must not reach a trampoline, which would queue the message again instead of writing it.
    nb::class_<om::msg::OutputHandler, PyOutputHandler>(m, "OutputHandler")
        .def(nb::init<>())
        .def(
            "log",
            [](om::msg::OutputHandler &h, const std::string &text, om::msg::LogLevel level, const std::string &filename,
               int line)
            {
                if (dynamic_cast<PyOutputHandler *>(&h) != nullptr)
                    throw nb::type_error("OutputHandler.log() is abstract; override it in the subclass");
                h.log(text, level, filename.c_str(), line);
            },
            nb::arg("text"), nb::arg("level"), nb::arg("filename"), nb::arg("line"));

    nb::class_<om::msg::OutputHandlerSTD, PyOutputHandlerSTD, om::msg::OutputHandler>(m, "OutputHandlerSTD")
        .def(nb::init<>())
        .def(
            "log",
            [](om::msg::OutputHandlerSTD &h, const std::string &text, om::msg::LogLevel level,
               const std::string &filename, int line)
            { h.om::msg::OutputHandlerSTD::log(text, level, filename.c_str(), line); },
            nb::arg("text"), nb::arg("level"), nb::arg("filename"), nb::arg("line"));

    nb::class_<om::msg::OutputHandlerFile, om::msg::OutputHandler>(m, "OutputHandlerFile")
        .def(nb::init<const char *>(), nb::arg("filename"))
        .def("log", &om::msg::OutputHandlerFile::log, nb::arg("text"), nb::arg("level"), nb::arg("filename"),
             nb::arg("line"));

    m.def("setLogLevel", &om::msg::setLogLevel, nb::arg("level"));
    m.def("getLogLevel", &om::msg::getLogLevel);
    m.def("noOutputHandler",
          []
          {
              om::msg::noOutputHandler();
              InstalledHandlers::use(nb::object());
          });
    m.def("restorePreviousOutputHandler",
          []
          {
              om::msg::restorePreviousOutputHandler();
              InstalledHandlers::restorePrevious();
          });
    m.def(
        "useOutputHandler",
        [](om::msg::OutputHandler *oh)
        {
            om::msg::useOutputHandler(oh);
            InstalledHandlers::use(oh != nullptr ? nb::find(oh) : nb::object());
        },
        nb::arg("oh").none());
    m.def("getOutputHandler", &om::msg::getOutputHandler, nb::rv_policy::reference);

    m.def("OMPL_ERROR", [](const std::string &msg) { logFromPython(om::msg::LOG_ERROR, msg); }, nb::arg("message"));
    m.def("OMPL_WARN", [](const std::string &msg) { logFromPython(om::msg::LOG_WARN, msg); }, nb::arg("message"));
    m.def("OMPL_INFORM", [](const std::string &msg) { logFromPython(om::msg::LOG_INFO, msg); }, nb::arg("message"));
    m.def("OMPL_DEBUG", [](const std::string &msg) { logFromPython(om::msg::LOG_DEBUG, msg); }, nb::arg("message"));
    m.def("OMPL_DEVMSG1", [](const std::string &msg) { logFromPython(om::msg::LOG_DEV1, msg); }, nb::arg("message"));
    m.def("OMPL_DEVMSG2", [](const std::string &msg) { logFromPython(om::msg::LOG_DEV2, msg); }, nb::arg("message"));

    InstalledHandlers::setFallback(om::msg::getOutputHandler());
    nb::module_::import_("atexit").attr("register")(nb::cpp_function(
        []
        {
            LogDelivery::shutdown();
            InstalledHandlers::clear();
        }));
}
