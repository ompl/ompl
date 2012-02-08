/*
 [auto_generated]
 boost/numeric/odeint/integrate/integrate.hpp

 [begin_description]
 Convenience methods which choose the stepper for the current ODE.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef OMPLEXT_BOOST_NUMERIC_ODEINT_INTEGRATE_INTEGRATE_HPP_INCLUDED
#define OMPLEXT_BOOST_NUMERIC_ODEINT_INTEGRATE_INTEGRATE_HPP_INCLUDED

#include <omplext_odeint/boost/numeric/odeint/stepper/runge_kutta_cash_karp54_classic.hpp>
#include <omplext_odeint/boost/numeric/odeint/stepper/controlled_runge_kutta.hpp>
#include <omplext_odeint/boost/numeric/odeint/integrate/null_observer.hpp>
#include <omplext_odeint/boost/numeric/odeint/integrate/integrate_adaptive.hpp>



namespace boost {
namespace numeric {
namespace omplext_odeint {


/*
 * ToDo :
 *
 * determine type of dxdt for units
 *
 * the two overloads are needed in order to solve the forwarding problem
 */
template< class System , class State , class Time , class Observer >
size_t integrate( System system , State &start_state , Time start_time , Time end_time , Time dt , Observer observer )
{
    return integrate_adaptive( controlled_runge_kutta< runge_kutta_dopri5< State > >() , system , start_state , start_time , end_time , dt , observer );
}

template< class System , class State , class Time , class Observer >
size_t integrate( System system , const State &start_state , Time start_time , Time end_time , Time dt , Observer observer )
{
    return integrate_adaptive( controlled_runge_kutta< runge_kutta_dopri5< State > >() , system , start_state , start_time , end_time , dt , observer );
}







/*
 * the two overloads are needed in order to solve the forwarding problem
 */
template< class System , class State , class Time >
size_t integrate( System system , State &start_state , Time start_time , Time end_time , Time dt )
{
    return integrate( system , start_state , start_time , end_time , dt , null_observer() );
}

template< class System , class State , class Time >
size_t integrate( System system , const State &start_state , Time start_time , Time end_time , Time dt )
{
    return integrate( system , start_state , start_time , end_time , dt , null_observer() );
}




} // namespace omplext_odeint
} // namespace numeric
} // namespace boost



#endif // BOOST_NUMERIC_ODEINT_INTEGRATE_INTEGRATE_HPP_INCLUDED
