/*
 [auto_generated]
 boost/numeric/odeint/stepper/base/explicit_stepper_base.hpp

 [begin_description]
 Base class for all explicit Runge Kutta steppers.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef OMPLEXT_BOOST_NUMERIC_ODEINT_STEPPER_BASE_EXPLICIT_STEPPER_BASE_HPP_INCLUDED
#define OMPLEXT_BOOST_NUMERIC_ODEINT_STEPPER_BASE_EXPLICIT_STEPPER_BASE_HPP_INCLUDED


#include <iostream>

#include <boost/ref.hpp>
#include <boost/bind.hpp>


#include <omplext_odeint/boost/numeric/odeint/util/state_wrapper.hpp>
#include <omplext_odeint/boost/numeric/odeint/util/resizer.hpp>
#include <omplext_odeint/boost/numeric/odeint/util/is_resizeable.hpp>

#include <omplext_odeint/boost/numeric/odeint/stepper/stepper_categories.hpp>

#include <omplext_odeint/boost/numeric/odeint/stepper/base/algebra_stepper_base.hpp>

namespace boost {
namespace numeric {
namespace omplext_odeint {

/*
 * base class for explicit steppers
 * models the stepper concept
 */
template<
class Stepper ,
unsigned short Order ,
class State ,
class Value ,
class Deriv ,
class Time ,
class Algebra ,
class Operations ,
class Resizer
>
class explicit_stepper_base : public algebra_stepper_base< Algebra , Operations >
{
public:

    typedef explicit_stepper_base< Stepper , Order , State , Value , Deriv , Time , Algebra , Operations , Resizer > internal_stepper_base_type;
    typedef algebra_stepper_base< Algebra , Operations > algebra_stepper_base_type;

    typedef State state_type;
    typedef Value value_type;
    typedef Deriv deriv_type;
    typedef Time time_type;
    typedef Resizer resizer_type;
    typedef Stepper stepper_type;
    typedef stepper_tag stepper_category;
    typedef state_wrapper< state_type > wrapped_state_type;
    typedef state_wrapper< deriv_type > wrapped_deriv_type;

    typedef typename algebra_stepper_base_type::algebra_type algebra_type;

    typedef unsigned short order_type;
    static const order_type order_value = Order;


    explicit_stepper_base( const algebra_type &algebra = algebra_type() )
    : algebra_stepper_base_type( algebra )
    { }

    order_type order( void ) const
    {
        return order_value;
    }


    /*
     * Version 1 : do_step( sys , x , t , dt )
     *
     * the two overloads are needed in order to solve the forwarding problem
     */
    template< class System , class StateInOut >
    void do_step( System system , StateInOut &x , const time_type &t , const time_type &dt )
    {
        do_step_v1( system , x , t , dt );
    }

    template< class System , class StateInOut >
    void do_step( System system , const StateInOut &x , const time_type &t , const time_type &dt )
    {
        do_step_v1( system , x , t , dt );
    }


    /*
     * Version 2 : do_step( sys , x , dxdt , t , dt )
     *
     * this version does not solve the forwarding problem, boost.range can not be used
     */
    template< class System , class StateInOut , class DerivIn >
    void do_step( System system , StateInOut &x , const DerivIn &dxdt , const time_type &t , const time_type &dt )
    {
        this->stepper().do_step_impl( system , x , dxdt , t , x , dt );
    }


    /*
     * Version 3 : do_step( sys , in , t , out , dt )
     *
     * this version does not solve the forwarding problem, boost.range can not be used
     */
    template< class System , class StateIn , class StateOut >
    void do_step( System system , const StateIn &in , const time_type &t , StateOut &out , const time_type &dt )
    {
        typename boost::unwrap_reference< System >::type &sys = system;
        m_resizer.adjust_size( in , boost::bind( &internal_stepper_base_type::template resize_impl<StateIn> , boost::ref( *this ) , _1 ) );
        sys( in , m_dxdt.m_v ,t );
        this->stepper().do_step_impl( system , in , m_dxdt.m_v , t , out , dt );
    }


    /*
     * Version 4 : do_step( sys , in , dxdt , t , out , dt )
     *
     * this version does not solve the forwarding problem, boost.range can not be used
     */
    template< class System , class StateIn , class DerivIn , class StateOut >
    void do_step( System system , const StateIn &in , const DerivIn &dxdt , const time_type &t , StateOut &out , const time_type &dt )
    {
        this->stepper().do_step_impl( system , in , dxdt , t , out , dt );
    }



private:

    stepper_type& stepper( void )
    {
        return *static_cast< stepper_type* >( this );
    }

    const stepper_type& stepper( void ) const
    {
        return *static_cast< const stepper_type* >( this );
    }


    template< class StateIn >
    bool resize_impl( const StateIn &x )
    {
        return adjust_size_by_resizeability( m_dxdt , x , typename wrapped_deriv_type::is_resizeable() );
    }


    template< class System , class StateInOut >
    void do_step_v1( System system , StateInOut &x , const time_type &t , const time_type &dt )
    {
        typename boost::unwrap_reference< System >::type &sys = system;
        m_resizer.adjust_size( x , boost::bind( &internal_stepper_base_type::template resize_impl< StateInOut > , boost::ref( *this ) , _1 ) );
        sys( x , m_dxdt.m_v ,t );
        this->stepper().do_step_impl( system , x , m_dxdt.m_v , t , x , dt );
    }


    resizer_type m_resizer;

protected:

    wrapped_deriv_type m_dxdt;
};


} // odeint
} // numeric
} // boost

#endif // BOOST_NUMERIC_ODEINT_STEPPER_BASE_EXPLICIT_STEPPER_BASE_HPP_INCLUDED
