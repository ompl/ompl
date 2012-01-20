/*
 [auto_generated]
 boost/numeric/odeint/integrate/detail/integrate_times.hpp

 [begin_description]
 Default integrate times implementation.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef BOOST_NUMERIC_ODEINT_INTEGRATE_DETAIL_INTEGRATE_TIMES_HPP_INCLUDED
#define BOOST_NUMERIC_ODEINT_INTEGRATE_DETAIL_INTEGRATE_TIMES_HPP_INCLUDED

#include <stdexcept>
#include <iostream>

#include <boost/numeric/odeint/stepper/controlled_step_result.hpp>

#include <boost/ref.hpp>

namespace boost {
namespace numeric {
namespace odeint {
namespace detail {



/*
 * integrate_times for simple stepper
 */
template< class Stepper , class System , class State , class TimeIterator , class Time , class Observer >
size_t integrate_times(
        Stepper stepper , System system , State &start_state ,
        TimeIterator start_time , TimeIterator end_time , Time dt ,
        Observer observer , stepper_tag
)
{
    typename boost::unwrap_reference< Observer >::type &obs = observer;

    size_t steps = 0;
    Time current_dt = dt;
    while( start_time != end_time )
    {
        Time current_time = *start_time++;
        obs( start_state , current_time );
        while( current_time < *start_time )
        {
            current_dt = std::min( dt , *start_time - current_time );
            stepper.do_step( system , start_state , current_time , current_dt );
            current_time += current_dt;
            steps++;
        }
    }
    return steps;
}

/*
 * integrate_times for controlled stepper
 */
template< class Stepper , class System , class State , class TimeIterator , class Time , class Observer >
size_t integrate_times(
        Stepper stepper , System system , State &start_state ,
        TimeIterator start_time , TimeIterator end_time , Time dt ,
        Observer observer , controlled_stepper_tag
)
{
    typename boost::unwrap_reference< Observer >::type &obs = observer;

    const size_t max_attempts = 1000;
    const char *error_string = "Integrate adaptive : Maximal number of iterations reached. A step size could not be found.";
    size_t steps = 0;
    while( start_time != end_time )
    {
        size_t fail_steps = 0;
        Time current_time = *start_time++;
        obs( start_state , current_time );
        while( current_time < *start_time )
        {
            dt = std::min( dt , *start_time - current_time );
            if( stepper.try_step( system , start_state , current_time , dt ) == success )
            {
                current_time += dt;
                steps++;
            } else
                fail_steps++;
            if( fail_steps == max_attempts ) throw std::overflow_error( error_string );
        }
    }
    return steps;
}

/*
 * integrate_times for dense output stepper
 */
template< class Stepper , class System , class State , class TimeIterator , class Time , class Observer >
size_t integrate_times(
        Stepper stepper , System system , State &start_state ,
        TimeIterator start_time , TimeIterator end_time , Time dt ,
        Observer observer , dense_output_stepper_tag
)
{
     typename boost::unwrap_reference< Observer >::type &obs = observer;

     Time last_time_point = *(end_time-1);

     stepper.initialize( start_state , *start_time , dt );
     obs( start_state , *start_time++ );

     size_t count = 0;
     while( start_time != end_time )
     {
         while( ( *start_time <= stepper.current_time() ) && ( start_time != end_time ) )
         {
             stepper.calc_state( *start_time , start_state );
             obs( start_state , *start_time );
             start_time++;
         }

         // we have not reached the end, do another real step
         if( stepper.current_time() + stepper.current_time_step() <= last_time_point )
         {
             stepper.do_step( system );
             ++count;
         }
         else if( start_time != end_time )
         { // do the last step ending exactly on the end point
             stepper.initialize( stepper.current_state() , stepper.current_time() , last_time_point - stepper.current_time() );
             stepper.do_step( system );
             ++count;
         }
     }
     // do the last step

     return count;
}


} // namespace detail
} // namespace odeint
} // namespace numeric
} // namespace boost


#endif // BOOST_NUMERIC_ODEINT_INTEGRATE_DETAIL_INTEGRATE_ADAPTIVE_HPP_INCLUDED
