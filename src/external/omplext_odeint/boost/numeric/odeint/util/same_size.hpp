/*
 [auto_generated]
 boost/numeric/odeint/util/state_wrapper.hpp

 [begin_description]
 State wrapper for the state type in all stepper. The state wrappers are responsible for contruction,
 destruction, copying contruction, assignment and resizing.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef OMPLEXT_BOOST_NUMERIC_ODEINT_UTIL_SAME_SIZE_HPP_INCLUDED
#define OMPLEXT_BOOST_NUMERIC_ODEINT_UTIL_SAME_SIZE_HPP_INCLUDED

#include <boost/range.hpp>

namespace boost {
namespace numeric {
namespace omplext_odeint {

// same_size function
// standard implementation relies on boost.range
template< class State1 , class State2 >
struct same_size_impl
{
    static bool same_size( const State1 &x1 , const State2 &x2 )
    {
        return ( boost::size( x1 ) == boost::size( x2 ) );
    }
};


// do not overload or specialize this function, specialize resize_impl<> instead
template< class State1 , class State2 >
bool same_size( const State1 &x1 , const State2 &x2 )
{
    return same_size_impl< State1 , State2 >::same_size( x1 , x2 );
}



}
}
}



#endif // BOOST_NUMERIC_ODEINT_UTIL_SAME_SIZE_HPP_INCLUDED
