/*
 [auto_generated]
 boost/numeric/odeint/stepper/adams_bashforth_moulton.hpp

 [begin_description]
 Implementation of the Adams-Bashforth-Moulton method, a predictor-corrector multistep method.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef BOOST_NUMERIC_ODEINT_STEPPER_ADAMS_BASHFORTH_MOULTON_HPP_INCLUDED
#define BOOST_NUMERIC_ODEINT_STEPPER_ADAMS_BASHFORTH_MOULTON_HPP_INCLUDED


#include <boost/ref.hpp>
#include <boost/bind.hpp>

#include <boost/numeric/odeint/stepper/stepper_categories.hpp>
#include <boost/numeric/odeint/algebra/range_algebra.hpp>
#include <boost/numeric/odeint/algebra/default_operations.hpp>
//#include <boost/numeric/odeint/util/size_adjuster.hpp>

#include <boost/numeric/odeint/util/state_wrapper.hpp>
#include <boost/numeric/odeint/util/resizer.hpp>

#include <boost/numeric/odeint/stepper/adams_bashforth.hpp>
#include <boost/numeric/odeint/stepper/adams_moulton.hpp>

/*
 * # Introduce the number of states
 */

namespace boost {
namespace numeric {
namespace odeint {


/*
 * Static Adams-Bashforth-Moulton multistep-solver without step size control and without dense output.
 *
 * # Define the number of steps
 * # Define the explicit method
 * # Define the implicit method
 * # Bring the explicit method and the implicit method together
 */
template<
size_t Steps ,
class State ,
class Value = double ,
class Deriv = State ,
class Time = Value ,
class Algebra = range_algebra ,
class Operations = default_operations ,
class Resizer = initially_resizer
>
class adams_bashforth_moulton
{

public :

    typedef State state_type;
    typedef state_wrapper< state_type > wrapped_state_type;
    typedef Value value_type;
    typedef Deriv deriv_type;
    typedef state_wrapper< deriv_type > wrapped_deriv_type;
    typedef Time time_type;
    typedef Algebra algebra_type;
    typedef Operations operations_type;
    typedef Resizer resizer_type;
    typedef stepper_tag stepper_category;

    static const size_t steps = Steps;

    typedef adams_bashforth< steps , state_type , value_type , deriv_type , time_type , algebra_type , operations_type , resizer_type > adams_bashforth_type;
    typedef adams_moulton< steps , state_type , value_type , deriv_type , time_type , algebra_type , operations_type , resizer_type > adams_moulton_type;

    typedef unsigned short order_type;
    static const order_type order_value = steps + 1;

    adams_bashforth_moulton( ) : m_adams_moulton( m_adams_bashforth.algebra() )
    { }

    adams_bashforth_moulton( const algebra_type &algebra )
    : m_adams_bashforth( algebra ) , m_adams_moulton( m_adams_bashforth.algebra() )
    { }

    order_type order( void ) const { return order_value; }

    template< class System , class StateInOut >
    void do_step( System system , StateInOut &x , const time_type &t , const time_type &dt )
    {
        m_adams_bashforth.do_step( system , x , t , dt );
        m_adams_moulton.do_step( system , x , t , dt , m_adams_bashforth.step_storage() );
    }

    template< class System , class StateInOut >
    void do_step( System system , const StateInOut &x , const time_type &t , const time_type &dt )
    {
        m_adams_bashforth.do_step( system , x , t , dt );
        m_adams_moulton.do_step( system , x , t , dt , m_adams_bashforth.step_storage() );
    }

    template< class System , class StateIn , class StateOut >
    void do_step( System system , const StateIn &in , const time_type &t , const StateOut &out , const time_type &dt )
    {
        m_adams_bashforth.do_step( system , in , t , out , dt );
        m_adams_moulton.do_step( system , out , t , dt , m_adams_bashforth.step_storage() );
    }

    template< class System , class StateIn , class StateOut >
    void do_step( System system , const StateIn &in , const time_type &t , StateOut &out , const time_type &dt )
    {
        m_adams_bashforth.do_step( system , in , t , out , dt );
        m_adams_moulton.do_step( system , out , t , dt , m_adams_bashforth.step_storage() );
    }

    template< class StateType >
    void adjust_size( const StateType &x )
    {
        m_adams_bashforth.adjust_size( x );
        m_adams_moulton.adjust_size( x );
    }


    template< class ExplicitStepper , class System , class StateIn >
    void initialize( ExplicitStepper explicit_stepper , System system , StateIn &x , time_type &t , const time_type &dt )
    {
        m_adams_bashforth.initialize( explicit_stepper , system , x , t , dt );
    }

    template< class System , class StateIn >
    void initialize( System system , StateIn &x , time_type &t , const time_type &dt )
    {
        m_adams_bashforth.initialize( system , x , t , dt );
    }



private:

    adams_bashforth_type m_adams_bashforth;
    adams_moulton_type m_adams_moulton;
};




} // odeint
} // numeric
} // boost



#endif // BOOST_NUMERIC_ODEINT_STEPPER_ADAMS_BASHFORTH_MOULTON_HPP_INCLUDED
