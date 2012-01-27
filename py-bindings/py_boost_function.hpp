/******************************************************************************
 * How to wrap boost.function objects with boost.python and use them both from
 * C++ and Python.
 *
 *   AUTHOR: Alcides Viamontes Esquivel
 *
 *   LICENSE: You're free to use this file for whatever you want. You're
 *            specially welcome to improve it in any way and give the
 *            changes back.
 *
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
 *
 *     boost::function< void( std::string const& ) > module_greeter(
 *          module_greeter_f
 *     ) ;
 *     ...
 *
 *     BOOST_PYTHON_MODULE( foo ) {
 *
 *          using namespace boost.python;
 *          ...
 *
 *          def_function< void(string const&) >(
 *              "greeter_function_t",
 *              "A greeting function"
 *          );
 *
 *          ...
 *          scope().attr("module_greeter") = module_greeter;
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
 *          >>> grfunc = foo.greeter_function_t.from_callable( my_greetings )
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

#define PYDECLARE_FUNCTION(FT, func_name)                                                              \
namespace detail                                                                                       \
{                                                                                                      \
    boost::function<FT> func_name(boost::python::object o)                                             \
    {                                                                                                  \
        return detail::pyobject_invoker<FT, boost::function_traits<FT>::result_type,                   \
            boost::function_traits< FT >::arity>(o);                                                   \
    }                                                                                                  \
}

#define PYREGISTER_FUNCTION(FT,func_name,func_doc)                                                     \
    BOOST_STATIC_ASSERT( boost::function_types::is_function< FT >::value );                            \
    boost::python::def(BOOST_PP_STRINGIZE(func_name), &detail::func_name, func_doc);                   \
    def_function<FT>(BOOST_PP_STRINGIZE(func_name##_t), func_doc);

#endif // PY_BINDINGS_PY_BOOST_FUNCTION_

