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

#include <boost/preprocessor/repetition/enum_binary_params.hpp>
#include <boost/preprocessor/repetition/enum_params.hpp>
#include <boost/preprocessor/repetition/repeat.hpp>
#include <boost/preprocessor/repetition/repeat_from_to.hpp>


namespace detail {

    template <typename RT, int arity>
    struct pyobject_invoker;

    template <typename RT>
    struct pyobject_invoker<RT, 0 >
    {
        boost::python::object callable;

        RT operator( )( )
        {
            PyGILState_STATE gstate = PyGILState_Ensure();
            RT result = boost::python::extract<RT>( callable( ) );
            PyGILState_Release( gstate );
            return result;
        }
    };
    template <>
    struct pyobject_invoker<void, 0 >
    {
        boost::python::object callable;

        void operator( )( )
        {
            PyGILState_STATE gstate = PyGILState_Ensure();
            callable( );
            PyGILState_Release( gstate );
        }
    };

    template <
        typename T,
        bool isPointer=boost::is_pointer<T>::value,
        bool isReference=boost::is_reference<T>::value >
    struct wrapType
    {
        static inline T wrap(T& t) { return t; }
    };

    template <typename T>
    struct wrapType<T, true, false>
    {
        static inline boost::python::pointer_wrapper<T> wrap(T t) { return boost::python::ptr(t); }
    };
    template <typename T>
    struct wrapType<T, false, true>
    {
        static inline T wrap(T t) { return t; }
    };
#define WRAP_TYPE_ARG(z,n,T) \
    BOOST_PP_COMMA_IF(n) wrapType<Arg##n>::wrap(arg##n)

#define MAKE_PYOBJECT_INVOKER(z, n, data) \
    template <typename RT> \
    struct pyobject_invoker<RT, n > \
    { \
        boost::python::object callable; \
        \
        template < BOOST_PP_ENUM_PARAMS(n, typename Arg) > \
        RT operator( )(BOOST_PP_ENUM_BINARY_PARAMS(n, Arg, arg ) )\
        {\
            PyGILState_STATE gstate = PyGILState_Ensure();\
            RT result = boost::python::extract<RT>(callable( \
            BOOST_PP_REPEAT( n, WRAP_TYPE_ARG, _ ) ) );\
            PyGILState_Release( gstate );\
            return result;\
        }\
    }; \
    template <> \
    struct pyobject_invoker<void, n > \
    { \
        boost::python::object callable; \
        \
        template < BOOST_PP_ENUM_PARAMS(n, typename Arg) > \
        void operator( )(BOOST_PP_ENUM_BINARY_PARAMS(n, Arg, arg ) )\
        {\
            PyGILState_STATE gstate = PyGILState_Ensure();\
            callable( BOOST_PP_REPEAT(n, WRAP_TYPE_ARG, _ ) );\
            PyGILState_Release( gstate );\
        }\
    };

BOOST_PP_REPEAT_FROM_TO( 1, 6, MAKE_PYOBJECT_INVOKER, nothing );

#undef MAKE_PYOBJECT_INVOKER

    template <typename FT>
    boost::function< FT > function_frompyobj( boost::python::object o )
    {
        const int arity = boost::function_traits< FT >::arity;
        typedef
            typename  boost::function_types::result_type< FT >::type
            result_type;
        pyobject_invoker<result_type, arity > inv;
        inv.callable = o;
        return inv;
    }

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
        .def("from_callable", &detail::function_frompyobj<FT> )
        .staticmethod("from_callable")
    ;
} // def_function

#define PYDECLARE_FUNCTION(FT, func_name)                                                              \
namespace detail                                                                                       \
{                                                                                                      \
    boost::function<FT> func_name(boost::python::object o)                                             \
    {                                                                                                  \
        return detail::function_frompyobj<FT>(o);                                                      \
    }                                                                                                  \
}

#define PYREGISTER_FUNCTION(FT,func_name,func_doc)                                                     \
    BOOST_STATIC_ASSERT( boost::function_types::is_function< FT >::value );                            \
    boost::python::def(BOOST_PP_STRINGIZE(func_name), &detail::func_name, func_doc);                   \
    def_function<FT>(BOOST_PP_STRINGIZE(func_name##_t), func_doc);

#endif // PY_BINDINGS_PY_BOOST_FUNCTION_

