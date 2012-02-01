/*
 [auto_generated]
 boost/numeric/odeint/stepper/euler.hpp

 [begin_description]
 Implementation of the classical explicit Euler stepper. This method is really simple and should only
 be used for demonstration purposes.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef OMPLEXT_BOOST_NUMERIC_ODEINT_STEPPER_EULER_HPP_INCLUDED
#define OMPLEXT_BOOST_NUMERIC_ODEINT_STEPPER_EULER_HPP_INCLUDED


#include <omplext_odeint/boost/numeric/odeint/stepper/base/explicit_stepper_base.hpp>
#include <omplext_odeint/boost/numeric/odeint/util/resizer.hpp>
#include <omplext_odeint/boost/numeric/odeint/algebra/range_algebra.hpp>
#include <omplext_odeint/boost/numeric/odeint/algebra/default_operations.hpp>
#include <omplext_odeint/boost/numeric/odeint/stepper/detail/macros.hpp>

namespace boost {
namespace numeric {
namespace omplext_odeint {

template< class State , class Value , class Deriv , class Time , class Algebra , class Operations , class Resizer >
class dense_output_explicit_euler;

template<
class State ,
class Value = double ,
class Deriv = State ,
class Time = Value ,
class Algebra = range_algebra ,
class Operations = default_operations ,
class Resizer = initially_resizer
>
class euler
: public explicit_stepper_base<
  euler< State , Value , Deriv , Time , Algebra , Operations , Resizer > ,
  1 , State , Value , Deriv , Time , Algebra , Operations , Resizer >
{
public :

    friend class dense_output_explicit_euler< State , Value , Deriv , Time , Algebra , Operations , Resizer >;

    BOOST_ODEINT_EXPLICIT_STEPPERS_TYPEDEFS( euler , 1 );

    typedef euler< State , Value , Deriv , Time , Algebra , Operations , Resizer > stepper_type;

    euler( const algebra_type &algebra = algebra_type() ) : stepper_base_type( algebra )
    { }

    template< class System , class StateIn , class DerivIn , class StateOut >
    void do_step_impl( System system , const StateIn &in , const DerivIn &dxdt , const time_type &t , StateOut &out , const time_type &dt )
    {
        stepper_base_type::m_algebra.for_each3( out , in , dxdt ,
                typename operations_type::template scale_sum2< value_type , time_type >( 1.0 , dt ) );

    }

    template< class StateOut , class StateIn1 , class StateIn2 >
    void calc_state( StateOut &x , const time_type &t ,  const StateIn1 &old_state , const time_type &t_old , const StateIn2 &current_state , const time_type &t_new )
    {
        time_type delta = t - t_old;
        stepper_base_type::m_algebra.for_each3( x , old_state , stepper_base_type::m_dxdt.m_v ,
                typename operations_type::template scale_sum2< value_type , time_type >( 1.0 , delta ) );
    }


};




} // odeint
} // numeric
} // boost


#endif // BOOST_NUMERIC_ODEINT_STEPPER_EULER_HPP_INCLUDED
