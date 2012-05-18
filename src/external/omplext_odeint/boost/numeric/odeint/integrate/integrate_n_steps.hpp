/*
 [auto_generated]
 boost/numeric/odeint/integrate/integrate_n_steps.hpp

 [begin_description]
 Integration of n steps with constant time size. Adaptive and dense-output methods are fully supported.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef OMPLEXT_BOOST_NUMERIC_ODEINT_INTEGRATE_INTEGRATE_N_STEPS_HPP_INCLUDED
#define OMPLEXT_BOOST_NUMERIC_ODEINT_INTEGRATE_INTEGRATE_N_STEPS_HPP_INCLUDED

#include <boost/type_traits/is_same.hpp>

#include <omplext_odeint/boost/numeric/odeint/stepper/stepper_categories.hpp>
#include <omplext_odeint/boost/numeric/odeint/integrate/null_observer.hpp>
#include <omplext_odeint/boost/numeric/odeint/integrate/detail/integrate_n_steps.hpp>

namespace boost {
namespace numeric {
namespace omplext_odeint {


/*
 * Integrates n steps
 *
 * the two overloads are needed in order to solve the forwarding problem
 */
template< class Stepper , class System , class State , class Time , class Observer>
Time integrate_n_steps(
        Stepper stepper , System system , State &start_state ,
        Time start_time , Time dt , size_t num_of_steps ,
        Observer observer )
{

    return detail::integrate_n_steps(
                stepper , system , start_state ,
                start_time , dt , num_of_steps ,
                observer , typename Stepper::stepper_category() );
}

template< class Stepper , class System , class State , class Time , class Observer >
Time integrate_n_steps(
        Stepper stepper , System system , const State &start_state ,
        Time start_time , Time dt , size_t num_of_steps ,
        Observer observer )
{
    return detail::integrate_n_steps(
                 stepper , system , start_state ,
                 start_time , dt , num_of_steps ,
                 observer , typename Stepper::stepper_category() );
}








/*
 * the two overloads are needed in order to solve the forwarding problem
 */
template< class Stepper , class System , class State , class Time >
Time integrate_n_steps(
        Stepper stepper , System system , State &start_state ,
        Time start_time , Time dt , size_t num_of_steps )
{
    return integrate_n_steps( stepper , system , start_state , start_time , dt , num_of_steps , null_observer() );
}

template< class Stepper , class System , class State , class Time >
Time integrate_n_steps(
        Stepper stepper , System system , const State &start_state ,
        Time start_time , Time dt , size_t num_of_steps )
{
    return integrate_n_steps( stepper , system , start_state , start_time , dt , num_of_steps , null_observer() );
}




} // namespace omplext_odeint
} // namespace numeric
} // namespace boost



#endif // BOOST_NUMERIC_ODEINT_INTEGRATE_INTEGRATE_N_STEPS_HPP_INCLUDED
