/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Rice University.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Alcides Viamontes Esquivel, Mark Moll */

/******************************************************************************
 * How to wrap std::function objects with boost.python and use them both from
 * C++ and Python.
 *
 * This is a significantly enhanced version of the original version by
 * Alcides Viamontes Esquivel.
 * See https://wiki.python.org/moin/boost.python/HowTo?action=AttachFile&do=get&target=py_boost_function.hpp
 *
 ******************************************************************************
 *
 * Usage example:
 *
 *     For the boost.python C++ module:
 *
 *     ...
 *     #include "py_std_function.hpp"
 *     ...
 *
 *     void module_greeter_f(std::string const& origin)
 *     {
 *           cout << "Hello world, by " << origin << endl;
 *     }
 *
 *     BOOST_PYTHON_MODULE( foo ) {
 *
 *          using namespace boost::python;
 *
 *          def_function<void(std::string const&) >(
 *              "greeter_function_t",
 *              "A greeting function"
 *          );
 *
 *          PY_REGISTER_FUNCTION(void(std::string const&),
 *              greeter_function_t, "Greeter function type")
 *     }
 *
 * From python code:
 *
 *     - Invoke:
 *
 *          >>> import foo
 *          >>> foo.module_greeter("world")
 *
 *     - Create instances from python:
 *
 *          >>> def my_greetings(hi):
 *          >>> ... print hi, ", world"
 *          >>> ...
 *          >>> grfunc = foo.greeter_function_t( my_greetings )
 *
 */

#ifndef PY_BINDINGS_PY_STD_FUNCTION_
#define PY_BINDINGS_PY_STD_FUNCTION_

#include <functional>
#include <type_traits>

#include <boost/python.hpp>

namespace detail
{
    template <typename T>
    struct WrapType
    {
        using ArgType = typename std::conditional<std::is_pointer<T>::value || std::is_reference<T>::value,
            T, T&>::type;
        using ReturnType = typename std::conditional<std::is_same<T, ArgType>::value,
            typename std::conditional<std::is_pointer<T>::value,
                boost::python::pointer_wrapper<T>,
                typename std::conditional<std::is_fundamental<std::decay_t<T>>::value,
                    std::remove_reference_t<T>,
                    boost::reference_wrapper<std::remove_reference_t<T>> >::type>::type,
            T>::type;
        static inline auto wrap(ArgType t)
        {
            return ReturnType(t);
        }
    };

    template <typename FT>
    struct PyobjectInvoker;

    template <typename R, typename...Args>
    struct PyobjectInvoker<R(Args...)>
    {
        PyobjectInvoker(boost::python::object o) : callable(o)
        {
        }
        R operator()(Args... args)
        {
            PyGILState_STATE gstate = PyGILState_Ensure();
            R result = boost::python::extract<R>(callable(WrapType<Args>::wrap(args)...));
            PyGILState_Release(gstate);
            return result;
        }
        boost::python::object callable;
    };
    template <typename...Args>
    struct PyobjectInvoker<void(Args...)>
    {
        PyobjectInvoker(boost::python::object o) : callable(o)
        {
        }
        void operator()(Args... args)
        {
            PyGILState_STATE gstate = PyGILState_Ensure();
            callable(WrapType<Args>::wrap(args)...);
            PyGILState_Release(gstate);
        }
        boost::python::object callable;
    };
}  // namespace detail

template <typename FT>
void def_function(const char *func_name, const char *func_doc)
{
    BOOST_STATIC_ASSERT(std::is_function<FT>::value);
    namespace bp = boost::python;
    using function_t = std::function<FT>;
    bp::class_<function_t>(func_name, func_doc, bp::no_init)
        .def("__call__", &function_t::operator());
}

#define PYREGISTER_FUNCTION(FT, func_name, func_doc)               \
    BOOST_STATIC_ASSERT(std::is_function<FT>::value);              \
    boost::python::def(BOOST_PP_STRINGIZE(func_name),              \
        +[](boost::python::object o) -> std::function<FT>          \
        {                                                          \
            return detail::PyobjectInvoker<FT>(o);                 \
        }, func_doc);                                              \
    def_function<FT>(BOOST_PP_STRINGIZE(func_name##_t), func_doc); \
    boost::python::implicitly_convertible<                         \
        boost::python::object,                                     \
        detail::PyobjectInvoker<FT>>();

#endif  // PY_BINDINGS_PY_STD_FUNCTION_
