/*
 [auto_generated]
 boost/numeric/odeint/integrate/detail/integrate_adaptive.hpp

 [begin_description]
 Default Integrate adaptive implementation.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef OMPLEXT_BOOST_NUMERIC_ODEINT_INTEGRATE_DETAIL_INTEGRATE_ADAPTIVE_HPP_INCLUDED
#define OMPLEXT_BOOST_NUMERIC_ODEINT_INTEGRATE_DETAIL_INTEGRATE_ADAPTIVE_HPP_INCLUDED

#include <stdexcept>

#include <omplext_odeint/boost/numeric/odeint/stepper/controlled_step_result.hpp>
#include <omplext_odeint/boost/numeric/odeint/integrate/detail/integrate_const.hpp>

#include <boost/ref.hpp>

#include <iostream>

namespace boost {
namespace numeric {
namespace omplext_odeint {
namespace detail {


//forward declaration of detail::integrate const, 
//required in integrate_const for normal stepper
template< class Stepper , class System , class State , class Time , class Observer >
size_t integrate_const(        
        Stepper stepper , System system , State &start_state ,
        Time start_time , Time end_time , Time dt ,
        Observer observer , stepper_tag
                             );



/*
 * integrate_adaptive for simple stepper is basically an integrate_const + some last step
 */
template< class Stepper , class System , class State , class Time , class Observer >
size_t integrate_adaptive(
        Stepper stepper , System system , State &start_state ,
        Time start_time , Time end_time , Time dt ,
        Observer observer , stepper_tag
)
{
    size_t steps = boost::numeric::omplext_odeint::detail::integrate_const( 
                       stepper , system , start_state , start_time , 
                       end_time , dt , observer , stepper_tag() );
    if( steps*dt < end_time )
    {   //make a last step to end exactly at end_time
        stepper.do_step( system , start_state , steps*dt , end_time-steps*dt );
        steps++;
        typename boost::unwrap_reference< Observer >::type &obs = observer;
        obs( start_state , end_time );
    }
    return steps;
}


/*
 * classical integrate adpative
 */
template< class Stepper , class System , class State , class Time , class Observer >
size_t integrate_adaptive(
        Stepper stepper , System system , State &start_state ,
        Time &start_time , Time end_time , Time &dt ,
        Observer observer , controlled_stepper_tag
)
{
    typename boost::unwrap_reference< Observer >::type &obs = observer;

    const size_t max_attempts = 1000;
    const char *error_string = "Integrate adaptive : Maximal number of iterations reached. A step size could not be found.";
    size_t count = 0;
    while( start_time < end_time )
    {
        obs( start_state , start_time );
        if( ( start_time + dt ) > end_time )
        {
            dt = end_time - start_time;
        }

        size_t trials = 0;
        controlled_step_result res = success;
        do
        {
            res = stepper.try_step( system , start_state , start_time , dt );
            ++trials;
        }
        while( ( res == fail ) && ( trials < max_attempts ) );
        if( trials == max_attempts ) throw std::overflow_error( error_string );

        ++count;
    }
    obs( start_state , start_time );
    return count;
}


/*
 * integrate adaptive for dense output steppers
 *
 * step size control is used if the stepper supports it
 */
template< class Stepper , class System , class State , class Time , class Observer >
size_t integrate_adaptive(
        Stepper stepper , System system , State &start_state ,
        Time start_time , Time end_time , Time dt ,
        Observer observer , dense_output_stepper_tag )
{
    typename boost::unwrap_reference< Observer >::type &obs = observer;

    size_t count = 0;
    stepper.initialize( start_state , start_time , dt );

    while( stepper.current_time() < end_time )
    {
        while( stepper.current_time() + stepper.current_time_step() <= end_time )
        {   //make sure we don't go beyond the end_time
            obs( stepper.current_state() , stepper.current_time() );
            stepper.do_step( system );
            ++count;
        }
        stepper.initialize( stepper.current_state() , stepper.current_time() , end_time - stepper.current_time() );
    }
    obs( stepper.current_state() , stepper.current_time() );
    return count;
}




} // namespace detail
} // namespace omplext_odeint
} // namespace numeric
} // namespace boost


#endif // BOOST_NUMERIC_ODEINT_INTEGRATE_DETAIL_INTEGRATE_ADAPTIVE_HPP_INCLUDED
