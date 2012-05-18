/*
 [auto_generated]
 boost/numeric/odeint/stepper/base/explicit_error_stepper_base.hpp

 [begin_description]
 Base class for all explicit runge error steppers.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef OMPLEXT_BOOST_NUMERIC_ODEINT_STEPPER_BASE_EXPLICIT_ERROR_STEPPER_BASE_HPP_INCLUDED
#define OMPLEXT_BOOST_NUMERIC_ODEINT_STEPPER_BASE_EXPLICIT_ERROR_STEPPER_BASE_HPP_INCLUDED

#include <omplext_odeint/boost/numeric/odeint/util/bind.hpp>
#include <omplext_odeint/boost/numeric/odeint/util/unwrap_reference.hpp>

#include <omplext_odeint/boost/numeric/odeint/util/state_wrapper.hpp>
#include <omplext_odeint/boost/numeric/odeint/util/resizer.hpp>
#include <omplext_odeint/boost/numeric/odeint/util/copy.hpp>

#include <omplext_odeint/boost/numeric/odeint/stepper/stepper_categories.hpp>

#include <omplext_odeint/boost/numeric/odeint/stepper/base/algebra_stepper_base.hpp>

namespace boost {
namespace numeric {
namespace omplext_odeint {

/*
 * base class for explicit error steppers
 * models the error stepper concept
 */
template<
class ErrorStepper ,
unsigned short StepperOrder ,
unsigned short ErrorOrder ,
class State ,
class Value ,
class Deriv ,
class Time ,
class Algebra ,
class Operations ,
class Resizer
>
class explicit_error_stepper_base : public algebra_stepper_base< Algebra , Operations >
{
public:

    typedef algebra_stepper_base< Algebra , Operations > algebra_stepper_base_type;
    typedef typename algebra_stepper_base_type::algebra_type algebra_type;

    typedef State state_type;
    typedef state_wrapper< state_type > wrapped_state_type;
    typedef Value value_type;
    typedef Deriv deriv_type;
    typedef state_wrapper< deriv_type > wrapped_deriv_type;
    typedef Time time_type;
    typedef Resizer resizer_type;
    typedef ErrorStepper stepper_type;
    typedef explicit_error_stepper_tag stepper_category;

    typedef unsigned short order_type;
    static const order_type stepper_order_value = StepperOrder;
    static const order_type error_order_value = ErrorOrder;

    explicit_error_stepper_base( const algebra_type &algebra = algebra_type() )
    : m_algebra( algebra )
    { }

    order_type stepper_order( void ) const
    {
        return stepper_order_value;
    }

    order_type error_order( void ) const
    {
        return error_order_value;
    }

    //    explicit_error_stepper_base( const algebra_type &algebra ) : m_algebra( algebra ) { }

    /*
     * Version 1 : do_step( system , x , t , dt , xerr )
     *
     * the two overloads are needed in order to solve the forwarding problem
     */
    template< class System , class StateInOut , class Err >
    void do_step( System system , StateInOut &x , const time_type &t , const time_type &dt , Err &xerr )
    {
        do_step_v1( system , x , t , dt , xerr );
    }

    template< class System , class StateInOut , class Err >
    void do_step( System system , StateInOut &x , const time_type &t , const time_type &dt , Err &xerr )
    {
        do_step_v1( system , x , t , dt , xerr );
    }


    /*
     * Version 2 : do_step( system , x , dxdt , t , dt , xerr )
     *
     * this version does not solve the forwarding problem, boost.range can not be used
     */
    template< class System , class StateInOut , class DerivIn , class Err >
    void do_step( System system , StateInOut &x , const DerivIn &dxdt , const time_type &t , const time_type &dt , Err &xerr )
    {
        this->stepper().do_step_impl( system , x , dxdt , t , x , dt , xerr );
    }


    /*
     * Version 3 : do_step( system , in , t , out , dt , xerr )
     *
     * this version does not solve the forwarding problem, boost.range can not be used
     */
    template< class System , class StateIn , class StateOut , class Err >
    void do_step( System system , const StateIn &in , const time_type &t , StateOut &out , const time_type &dt , Err &xerr )
    {
        typename omplext_odeint::unwrap_reference< System >::type &sys = system;
        m_resizer.adjust_size( in , detail::bind( &internal_stepper_base_type::template resize_impl<StateIn> , detail::ref( *this ) , detail::_1 ) );
        sys( in , m_dxdt.m_v ,t );
        this->stepper().do_step_impl( system , in , m_dxdt.m_v , t , out , dt , xerr );
    }


    /*
     * Version 4 : do_step( system , in , dxdt , t , out , dt , xerr )
     *
     * this version does not solve the forwarding problem, boost.range can not be used
     */
    template< class System , class StateIn , class DerivIn , class StateOut , class Err >
    void do_step( System system , const StateIn &in , const DerivIn &dxdt , const time_type &t , StateOut &out , const time_type &dt , Err &xerr )
    {
        this->stepper().do_step_impl( system , in , dxdt , t , out , dt , xerr );
    }


    template< class StateType >
    void adjust_size( const StateType &x )
    {
        resize_impl( x );
    }


private:

    template< class StateIn >
    bool resize_impl( const StateIn &x )
    {
        return adjust_size_by_resizeability( m_dxdt , x , typename is_resizeable<deriv_type>::type() );
    }



    template< class System , class StateInOut , class Err >
    void do_step_v1( System system , StateInOut &x , const time_type &t , const time_type &dt , Err &xerr )
    {
        typename omplext_odeint::unwrap_reference< System >::type &sys = system;
        m_resizer.adjust_size( in , detail::bind( &internal_stepper_base_type::template resize_impl<StateIn> , detail::ref( *this ) , detail::_1 ) );
        sys( x , m_dxdt.m_v ,t );
        this->stepper().do_step_impl( system , x , m_dxdt.m_v , t , x , dt , xerr );
    }


    // ToDo : make the next two methods private?
    stepper_type& stepper( void )
    {
        return *static_cast< stepper_type* >( this );
    }

    const stepper_type& stepper( void ) const
    {
        return *static_cast< const stepper_type* >( this );
    }


    resizer_type m_resizer;

protected:

    wrapped_deriv_type m_dxdt;
};


} // odeint
} // numeric
} // boost

#endif // BOOST_NUMERIC_ODEINT_STEPPER_BASE_EXPLICIT_ERROR_STEPPER_BASE_HPP_INCLUDED
