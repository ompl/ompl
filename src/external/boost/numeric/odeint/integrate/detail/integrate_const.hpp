/*
 [auto_generated]
 boost/numeric/odeint/integrate/detail/integrate_const.hpp

 [begin_description]
 Default integrate_const implementaiton
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef BOOST_NUMERIC_ODEINT_INTEGRATE_DETAIL_INTEGRATE_CONST_HPP_INCLUDED
#define BOOST_NUMERIC_ODEINT_INTEGRATE_DETAIL_INTEGRATE_CONST_HPP_INCLUDED

#include <boost/numeric/odeint/integrate/detail/integrate_adaptive.hpp>

#include <boost/ref.hpp>

namespace boost {
namespace numeric {
namespace odeint {
namespace detail {


/*
 * integrates with constant time step using simple steppers

 * No step size control
 */
template< class Stepper , class System , class State , class Time , class Observer >
size_t integrate_const(
        Stepper stepper , System system , State &start_state ,
        Time start_time , Time end_time , Time dt ,
        Observer observer , stepper_tag
)
{
    typename boost::unwrap_reference< Observer >::type &obs = observer;

    size_t count = 0;
    while( start_time+dt <= end_time )
    {
        obs( start_state , start_time );
        stepper.do_step( system , start_state , start_time , dt );
        start_time += dt;
        ++count;
    }
    obs( start_state , start_time );
    return count;
}


/*
 * integrates with constant time step using a controlled stepper
 *
 * step size control is used
 */
template< class Stepper , class System , class State , class Time , class Observer >
size_t integrate_const(
        Stepper stepper , System system , State &start_state ,
        Time &start_time , Time end_time , Time &dt ,
        Observer observer , controlled_stepper_tag
)
{
    typename boost::unwrap_reference< Observer >::type &obs = observer;

    size_t count = 0;
    Time time_step = dt;
    while( start_time+time_step <= end_time )
    {
        obs( start_state , start_time );
        Time next_time = start_time + time_step;
        count += detail::integrate_adaptive(
                stepper , system , start_state , start_time , next_time , dt ,
                null_observer() , controlled_stepper_tag() );
    }
    obs( start_state , start_time );
    return count;
}



/*
 * integrates with constant time step using a dense output stepper
 */
template< class Stepper , class System , class State , class Time , class Observer >
size_t integrate_const(
        Stepper stepper , System system , State &start_state ,
        Time start_time , Time end_time , Time dt ,
        Observer observer , dense_output_stepper_tag )
{
    typename boost::unwrap_reference< Observer >::type &obs = observer;

    stepper.initialize( start_state , start_time , dt );

    size_t count = 0;
    while( start_time < end_time )
    {
        while( ( start_time < stepper.current_time() ) && ( start_time < end_time ) )
        {
            stepper.calc_state( start_time , start_state );
            obs( start_state , start_time );
            start_time += dt;
        }

        // we have not reached the end, do another real step
        if( stepper.current_time() + stepper.current_time_step() <= end_time )
        {
            stepper.do_step( system );
            ++count;
        }
        else if( start_time < end_time )
        { // do the last step ending exactly on the end point
            stepper.initialize( stepper.current_state() , stepper.current_time() , end_time - stepper.current_time() );
            stepper.do_step( system );
            ++count;
        }
    }

    return count;
}


} // namespace detail
} // namespace odeint
} // namespace numeric
} // namespace boost

#endif // BOOST_NUMERIC_ODEINT_INTEGRATE_DETAIL_INTEGRATE_CONST_HPP_INCLUDED
