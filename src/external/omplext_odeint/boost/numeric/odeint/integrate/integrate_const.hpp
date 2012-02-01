/*
 [auto_generated]
 boost/numeric/odeint/integrate/integrate_const.hpp

 [begin_description]
 Constant integration of ODEs, meaning that the state of the ODE is observed on constant time intervals.
 The routines makes full use of adaptive and dense-output methods.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef OMPLEXT_BOOST_NUMERIC_ODEINT_INTEGRATE_INTEGRATE_CONST_HPP_INCLUDED
#define OMPLEXT_BOOST_NUMERIC_ODEINT_INTEGRATE_INTEGRATE_CONST_HPP_INCLUDED

#include <boost/type_traits/is_same.hpp>

#include <omplext_odeint/boost/numeric/odeint/stepper/stepper_categories.hpp>
#include <omplext_odeint/boost/numeric/odeint/integrate/null_observer.hpp>
#include <omplext_odeint/boost/numeric/odeint/integrate/detail/integrate_const.hpp>
#include <omplext_odeint/boost/numeric/odeint/integrate/detail/integrate_adaptive.hpp>

namespace boost {
namespace numeric {
namespace omplext_odeint {





/*
 * Integrates with constant time step dt.
 */
template< class Stepper , class System , class State , class Time , class Observer >
size_t integrate_const(
        Stepper stepper , System system , State &start_state ,
        Time start_time , Time end_time , Time dt ,
        Observer observer
)
{
    // we want to get as fast as possible to the end
    if( boost::is_same< null_observer , Observer >::value )
    {
        return detail::integrate_adaptive(
                stepper , system , start_state ,
                start_time , end_time  , dt ,
                observer , typename Stepper::stepper_category() );
    }
    else
    {
        return detail::integrate_const(
                stepper , system , start_state ,
                start_time , end_time  , dt ,
                observer , typename Stepper::stepper_category() );
    }
}


template< class Stepper , class System , class State , class Time , class Observer >
size_t integrate_const(
        Stepper stepper , System system , const State &start_state ,
        Time start_time , Time end_time , Time dt ,
        Observer observer
)
{
    // we want to get as fast as possible to the end
    if( boost::is_same< null_observer , Observer >::value )
    {
        return detail::integrate_adaptive(
                stepper , system , start_state ,
                start_time , end_time  , dt ,
                observer , typename Stepper::stepper_category() );
    }
    else
    {
        return detail::integrate_const(
                stepper , system , start_state ,
                start_time , end_time  , dt ,
                observer , typename Stepper::stepper_category() );
    }
}





/*
 * Without observers
 */
template< class Stepper , class System , class State , class Time >
size_t integrate_const(
        Stepper stepper , System system , State &start_state ,
        Time start_time , Time end_time , Time dt
)
{
    return integrate_const( stepper , system , start_state , start_time , end_time , dt , null_observer() );
}

template< class Stepper , class System , class State , class Time >
size_t integrate_const(
        Stepper stepper , System system , const State &start_state ,
        Time start_time , Time end_time , Time dt
)
{
    return integrate_const( stepper , system , start_state , start_time , end_time , dt , null_observer() );
}







} // namespace omplext_odeint
} // namespace numeric
} // namespace boost



#endif // BOOST_NUMERIC_ODEINT_INTEGRATE_INTEGRATE_CONST_HPP_INCLUDED
