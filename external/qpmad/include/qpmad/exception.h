/**
    @file
    @author  Alexander Sherikov

    @copyright 2019 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Throw & assert macro
*/

#ifndef H_QPMAD_EXCEPTION
#define H_QPMAD_EXCEPTION

#include <stdexcept>

#define QPMAD_UTILS_THROW_EXCEPTION(exception_type, message) throw exception_type((message))

#define QPMAD_UTILS_THROW(s)                                                                                           \
    QPMAD_UTILS_THROW_EXCEPTION(std::runtime_error, (std::string("In ") + __func__ + "() // " + (s)))


#define QPMAD_UTILS_PERSISTENT_ASSERT(condition, ...)                                                                  \
    if (!(condition))                                                                                                  \
    {                                                                                                                  \
        QPMAD_UTILS_THROW(__VA_ARGS__);                                                                                \
    };

#ifdef DNDEBUG
#    define QPMAD_UTILS_ASSERT(condition, ...)
#else
#    define QPMAD_UTILS_ASSERT(condition, ...) QPMAD_UTILS_PERSISTENT_ASSERT(condition, __VA_ARGS__)
#endif

#endif
