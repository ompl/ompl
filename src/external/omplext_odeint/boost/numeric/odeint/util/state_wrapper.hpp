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


#ifndef OMPLEXT_BOOST_NUMERIC_ODEINT_UTIL_STATE_WRAPPER_HPP_INCLUDED
#define OMPLEXT_BOOST_NUMERIC_ODEINT_UTIL_STATE_WRAPPER_HPP_INCLUDED


#include <boost/type_traits/integral_constant.hpp>

#include <omplext_odeint/boost/numeric/odeint/util/is_resizeable.hpp>
#include <omplext_odeint/boost/numeric/odeint/util/resize.hpp>
#include <omplext_odeint/boost/numeric/odeint/util/same_size.hpp>


namespace boost {
namespace numeric {
namespace omplext_odeint {




template< class V , bool resizeable = is_resizeable< V >::value >
struct state_wrapper;



//two standard implementations, with and without resizing depending on is_resizeable< StateType >

template< class V >
struct state_wrapper< V , true > // with resizing
{
    typedef state_wrapper< V , true > state_wrapper_type;
    //typedef typename V::value_type value_type;
    typedef boost::true_type is_resizeable;

    V m_v;

//    template< class StateIn >
//    bool same_size( const StateIn &x ) const
//    {
//        return boost::numeric::omplext_odeint::same_size( m_v , x );
//    }
//
//    template< class StateIn >
//    bool resize( const StateIn &x )
//    {
//        if( !this->same_size( x ) )
//        {
//            boost::numeric::omplext_odeint::resize( m_v , x );
//            return true;
//        }
//        else
//        {
//            return false;
//        }
//    }
};


template< class V >
struct state_wrapper< V , false > // without resizing
{
    typedef state_wrapper< V , false > state_wrapper_type;
    //typedef typename V::value_type value_type;
    typedef boost::false_type is_resizeable;

    V m_v;

    //no resize method
};

}
}
}



#endif // BOOST_NUMERIC_ODEINT_UTIL_STATE_WRAPPER_HPP_INCLUDED
