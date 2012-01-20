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



#ifndef YOUR_INCLUDE_GUARD
#define YOUR_INCLUDE_GUARD 

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


namespace detail{

    template <typename RT, int arity>
    struct pyobject_invoker;

    template <typename RT> 
    struct pyobject_invoker<RT, 0 > 
    { 
        boost::python::object callable; 
        
        RT operator( )( )
        {
            return boost::python::extract<RT>( callable( ) );
        }
    };

#define MAKE_PYOBJECT_INVOKER(z, n, data) \
    template <typename RT> \
    struct pyobject_invoker<RT, n > \
    { \
        boost::python::object callable; \
        \
        template < BOOST_PP_ENUM_PARAMS(n, typename Arg) > \
        RT operator( )(BOOST_PP_ENUM_BINARY_PARAMS(n, Arg, arg ) )\
        {\
            boost::python::object o = callable( BOOST_PP_ENUM_PARAMS(n, arg) );\
            return boost::python::extract<RT>( o );\
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
} // def_function0




#endif // YOUR_INCLUDE_GUARD

