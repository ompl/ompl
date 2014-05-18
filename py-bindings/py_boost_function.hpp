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
 * How to wrap boost.function objects with boost.python and use them both from
 * C++ and Python.
 *
 * This is a significantly enhanced version of the original version by
 * Alcides Viamontes Esquivel.
 * *
 ******************************************************************************
 *
 * Usage example:
 *
 *     For the boost.python C++ module:
 *
 *     ...
 *     #include <boost/python.hpp>
 *     ...
 *     #include "py_boost_function.hpp"
 *     ...
 *
 *     void module_greeter_f(std::string const& origin)
 *     {
 *           cout << "Hello world, by " << origin << endl;
 *     }
 *
 *     PYDECLARE_FUNCTION(void(std::string const&), greeter_function_t)
 *
 *     BOOST_PYTHON_MODULE( foo ) {
 *
 *          using namespace boost.python;
 *
 *          def_function< void(std::string const&) >(
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



#ifndef PY_BINDINGS_PY_BOOST_FUNCTION_
#define PY_BINDINGS_PY_BOOST_FUNCTION_

#include <boost/python.hpp>

#include <boost/function.hpp>
#include <boost/function_types/function_type.hpp>
#include <boost/function_types/result_type.hpp>
#include <boost/function_types/is_function.hpp>

#include <boost/type_traits/function_traits.hpp>
#include <boost/type_traits/remove_reference.hpp>
#include <boost/type_traits/is_fundamental.hpp>

#include <boost/preprocessor/repetition/enum_binary_params.hpp>
#include <boost/preprocessor/repetition/enum_params.hpp>
#include <boost/preprocessor/repetition/repeat.hpp>
#include <boost/preprocessor/repetition/repeat_from_to.hpp>
#include <boost/preprocessor/arithmetic/inc.hpp>
#include <boost/preprocessor/arithmetic/dec.hpp>

#include <iostream>

namespace detail {

    template <typename FT, typename RT, int arity>
    struct pyobject_invoker;

    template <typename RT>
    struct pyobject_invoker<RT(), RT, 0 >
    {
        pyobject_invoker(boost::python::object o) : callable(o) {}

        RT operator( )( )
        {
            PyGILState_STATE gstate = PyGILState_Ensure();
            RT result = boost::python::extract<RT>( callable( ) );
            PyGILState_Release( gstate );
            return result;
        }

        boost::python::object callable;
    };
    template <>
    struct pyobject_invoker<void(), void, 0 >
    {
        pyobject_invoker(boost::python::object o) : callable(o) {}

        void operator( )( )
        {
            PyGILState_STATE gstate = PyGILState_Ensure();
            callable( );
            PyGILState_Release( gstate );
        }

        boost::python::object callable;
    };

    template <typename T, bool isPointer, bool isReference>
    struct wrapType
    {
        static inline T wrap(T& t) { return t; }
    };

    template <typename T>
    struct wrapType<T, true, false>
    {
        static inline boost::python::pointer_wrapper<T> wrap(T t) { return boost::python::ptr(t); }
    };
    template <typename T, bool B>
    struct wrapHelper
    {
        typedef boost::reference_wrapper<typename boost::remove_reference<T>::type> return_type;
        static inline return_type wrap(T t) { return boost::ref(t); }
    };
    template<typename T>
    struct wrapHelper<T, true>
    {
        typedef typename boost::remove_reference<T>::type return_type;
        static inline return_type wrap(T t) { return t; }
    };
    template <typename T>
    struct wrapType<T, false, true>
    {
        typedef boost::is_fundamental<typename boost::remove_cv<typename boost::remove_reference<T>::type>::type> isFunType;

        static inline typename wrapHelper<T, isFunType::value>::return_type wrap(T t)
        {
            return wrapHelper<T, isFunType::value>::wrap(t);
        }
    };
#define WRAP_TYPE_ARG(z,n,FT) \
    BOOST_PP_COMMA_IF(BOOST_PP_DEC(n)) wrapType<\
        typename boost::function_traits<FT>::arg##n##_type, \
        boost::is_pointer<typename boost::function_traits<FT>::arg##n##_type>::value, \
        boost::is_reference<typename boost::function_traits<FT>::arg##n##_type>::value \
        >::wrap(arg##n)
#define WRAP_BINARY_TYPE_ARG(z,n,FT) \
    BOOST_PP_COMMA_IF(BOOST_PP_DEC(n)) typename boost::function_traits<FT>::arg##n##_type arg##n

#define MAKE_PYOBJECT_INVOKER(z, n, data) \
    template <typename FT, typename RT>                                                           \
    struct pyobject_invoker<FT,RT,n>                                                              \
    {                                                                                             \
        pyobject_invoker(boost::python::object o) : callable(o) {}                                \
                                                                                                  \
        RT operator()(BOOST_PP_REPEAT_FROM_TO( 1, BOOST_PP_INC(n), WRAP_BINARY_TYPE_ARG, FT ) )   \
        {                                                                                         \
            PyGILState_STATE gstate = PyGILState_Ensure();                                        \
            RT result = boost::python::extract<RT>(callable(                                      \
                BOOST_PP_REPEAT_FROM_TO( 1, BOOST_PP_INC(n), WRAP_TYPE_ARG, FT ) ) );             \
            PyGILState_Release( gstate );                                                         \
            return result;                                                                        \
        }                                                                                         \
                                                                                                  \
        boost::python::object callable;                                                           \
    };                                                                                            \
    template <typename FT>                                                                        \
    struct pyobject_invoker<FT,void,n>                                                            \
    {                                                                                             \
        pyobject_invoker(boost::python::object o) : callable(o) {}                                \
                                                                                                  \
        void operator()(BOOST_PP_REPEAT_FROM_TO( 1, BOOST_PP_INC(n), WRAP_BINARY_TYPE_ARG, FT ) ) \
        {                                                                                         \
            PyGILState_STATE gstate = PyGILState_Ensure();                                        \
            callable(BOOST_PP_REPEAT_FROM_TO( 1, BOOST_PP_INC(n), WRAP_TYPE_ARG, FT ) );          \
            PyGILState_Release( gstate );                                                         \
        }                                                                                         \
                                                                                                  \
        boost::python::object callable;                                                           \
    };

BOOST_PP_REPEAT_FROM_TO( 1, 6, MAKE_PYOBJECT_INVOKER, nothing );

#undef MAKE_PYOBJECT_INVOKER
} // namespace detail

template <typename FT>
void def_function(const char* func_name, const char* func_doc)
{
    BOOST_STATIC_ASSERT( boost::function_types::is_function< FT >::value ) ;
    namespace bp = boost::python;
    typedef boost::function<FT> function_t;
    bp::class_< function_t >
    (func_name, func_doc, bp::no_init)
        .def("__call__", &function_t::operator() )
    ;
} // def_function

#define PYDECLARE_FUNCTION(FT, func_name)                                                      \
namespace detail                                                                               \
{                                                                                              \
    boost::function<FT> func_name(boost::python::object o)                                     \
    {                                                                                          \
        return detail::pyobject_invoker<FT, boost::function_traits<FT>::result_type,           \
            boost::function_traits< FT >::arity>(o);                                           \
    }                                                                                          \
}

#define PYREGISTER_FUNCTION(FT,func_name,func_doc)                                             \
    BOOST_STATIC_ASSERT( boost::function_types::is_function< FT >::value );                    \
    boost::python::def(BOOST_PP_STRINGIZE(func_name), &detail::func_name, func_doc);           \
    def_function<FT>(BOOST_PP_STRINGIZE(func_name##_t), func_doc);                             \
    boost::python::implicitly_convertible<boost::python::object,                               \
        detail::pyobject_invoker<FT, boost::function_traits<FT>::result_type,                  \
            boost::function_traits< FT >::arity> >();

#endif // PY_BINDINGS_PY_BOOST_FUNCTION_

