/*
 [auto_generated]
 boost/numeric/odeint/util/is_resizeable.hpp

 [begin_description]
 Metafunction to determine if a state type can resized. For usage in the steppers.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef OMPLEXT_BOOST_NUMERIC_ODEINT_UTIL_IS_RESIZEABLE_HPP_INCLUDED
#define OMPLEXT_BOOST_NUMERIC_ODEINT_UTIL_IS_RESIZEABLE_HPP_INCLUDED


#include <vector>

#include <boost/type_traits/integral_constant.hpp>


namespace boost {
namespace numeric {
namespace omplext_odeint {

/*
 * by default any type is not resizable
 */
template< class Container >
struct is_resizeable
{
    //struct type : public boost::false_type { };
    typedef boost::false_type type;
    const static bool value = type::value;
};

/*
 * specialization for std::vector
 */
template< class V, class A >
struct is_resizeable< std::vector< V , A  > >
{
    //struct type : public boost::true_type { };
    typedef boost::true_type type;
    const static bool value = type::value;
};



} // namespace omplext_odeint
} // namespace numeric
} // namespace boost



#endif // BOOST_NUMERIC_ODEINT_UTIL_IS_RESIZEABLE_HPP_INCLUDED
