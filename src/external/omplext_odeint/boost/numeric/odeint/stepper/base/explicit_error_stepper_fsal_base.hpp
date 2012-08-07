/*
 [auto_generated]
 boost/numeric/odeint/stepper/base/explicit_error_stepper_fsal_base.hpp

 [begin_description]
 Base class for all explicit first-same-as-last Runge Kutta steppers.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef OMPLEXT_BOOST_NUMERIC_ODEINT_STEPPER_BASE_EXPLICIT_ERROR_STEPPER_FSAL_BASE_HPP_INCLUDED
#define OMPLEXT_BOOST_NUMERIC_ODEINT_STEPPER_BASE_EXPLICIT_ERROR_STEPPER_FSAL_BASE_HPP_INCLUDED

#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_same.hpp>


#include <omplext_odeint/boost/numeric/odeint/util/bind.hpp>
#include <omplext_odeint/boost/numeric/odeint/util/unwrap_reference.hpp>
#include <omplext_odeint/boost/numeric/odeint/util/state_wrapper.hpp>
#include <omplext_odeint/boost/numeric/odeint/util/is_resizeable.hpp>
#include <omplext_odeint/boost/numeric/odeint/util/resizer.hpp>
#include <omplext_odeint/boost/numeric/odeint/util/copy.hpp>

#include <omplext_odeint/boost/numeric/odeint/stepper/stepper_categories.hpp>

#include <omplext_odeint/boost/numeric/odeint/stepper/base/algebra_stepper_base.hpp>

namespace boost {
namespace numeric {
namespace omplext_odeint {

/*
 * base class for explicit stepper and error steppers with the fsal property
 * models the stepper AND the error stepper fsal concept
 * 
 * this class provides the following do_step overloads
    * do_step( sys , x , t , dt )
    * do_step( sys , x , dxdt , t , dt )
    * do_step( sys , in , t , out , dt )
    * do_step( sys , in , dxdt_in , t , out , dxdt_out , dt )
    * do_step( sys , x , t , dt , xerr )
    * do_step( sys , x , dxdt , t , dt , xerr )
    * do_step( sys , in , t , out , dt , xerr )
    * do_step( sys , in , dxdt_in , t , out , dxdt_out , dt , xerr )
 */
template<
class Stepper ,
unsigned short Order ,
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
class explicit_error_stepper_fsal_base : public algebra_stepper_base< Algebra , Operations >
{
public:

    typedef algebra_stepper_base< Algebra , Operations > algebra_stepper_base_type;
    typedef typename algebra_stepper_base_type::algebra_type algebra_type;

    typedef State state_type;
    typedef Value value_type;
    typedef Deriv deriv_type;
    typedef Time time_type;
    typedef Resizer resizer_type;
    typedef Stepper stepper_type;
    typedef explicit_error_stepper_fsal_tag stepper_category;
    typedef state_wrapper< state_type > wrapped_state_type;
    typedef state_wrapper< deriv_type > wrapped_deriv_type;

    typedef explicit_error_stepper_fsal_base< Stepper , Order , StepperOrder , ErrorOrder ,
            State , Value , Deriv , Time , Algebra , Operations , Resizer > internal_stepper_base_type;

    typedef unsigned short order_type;
    static const order_type order_value = Order;
    static const order_type stepper_order_value = StepperOrder;
    static const order_type error_order_value = ErrorOrder;


    explicit_error_stepper_fsal_base( const algebra_type &algebra = algebra_type() )
    : algebra_stepper_base_type( algebra ) , m_first_call( true )
    { }

    order_type order( void ) const
    {
        return order_value;
    }

    order_type stepper_order( void ) const
    {
        return stepper_order_value;
    }

    order_type error_order( void ) const
    {
        return error_order_value;
    }


    /*
     * version 1 : do_step( sys , x , t , dt )
     *
     * the two overloads are needed in order to solve the forwarding problem
     */
    template< class System , class StateInOut >
    void do_step( System system , StateInOut &x , time_type t , time_type dt )
    {
        do_step_v1( system , x , t , dt );
    }

    template< class System , class StateInOut >
    void do_step( System system , const StateInOut &x , time_type t , time_type dt )
    {
        do_step_v1( system , x , t , dt );
    }


    /*
     * version 2 : do_step( sys , x , dxdt , t , dt )
     *
     * this version does not solve the forwarding problem, boost.range can not be used
     *
     * the disable is needed to avoid ambiguous overloads if state_type = time_type
     */
    template< class System , class StateInOut , class DerivInOut >
    typename boost::disable_if< boost::is_same< StateInOut , time_type > , void >::type
    do_step( System system , StateInOut &x , DerivInOut &dxdt , time_type t , time_type dt )
    {
        m_first_call = true;
        this->stepper().do_step_impl( system , x , dxdt , t , x , dxdt , dt );
    }


    /*
     * version 3 : do_step( sys , in , t , out , dt )
     *
     * this version does not solve the forwarding problem, boost.range can not be used
     *
     * the disable is needed to avoid ambiguous overloads if state_type = time_type
     */
    template< class System , class StateIn , class StateOut >
    typename boost::disable_if< boost::is_same< StateIn , time_type > , void >::type
    do_step( System system , const StateIn &in , time_type t , StateOut &out , time_type dt )
    {
        if( m_resizer.adjust_size( in , detail::bind( &internal_stepper_base_type::template resize_impl< StateIn > , detail::ref( *this ) , detail::_1 ) ) || m_first_call )
        {
            initialize( system , in , t );
        }
        this->stepper().do_step_impl( system , in , m_dxdt.m_v , t , out , m_dxdt.m_v , dt );
    }


    /*
     * version 4 : do_step( sys , in , dxdt_in , t , out , dxdt_out , dt )
     *
     * this version does not solve the forwarding problem, boost.range can not be used
     */
    template< class System , class StateIn , class DerivIn , class StateOut , class DerivOut >
    void do_step( System system , const StateIn &in , const DerivIn &dxdt_in , time_type t ,
            StateOut &out , DerivOut &dxdt_out , time_type dt )
    {
        m_first_call = true;
        this->stepper().do_step_impl( system , in , dxdt_in , t , out , dxdt_out , dt );
    }





    /*
     * version 5 : do_step( sys , x , t , dt , xerr )
     *
     * the two overloads are needed in order to solve the forwarding problem
     */
    template< class System , class StateInOut , class Err >
    void do_step( System system , StateInOut &x , time_type t , time_type dt , Err &xerr )
    {
        do_step_v5( system , x , t , dt , xerr );
    }

    template< class System , class StateInOut , class Err >
    void do_step( System system , const StateInOut &x , time_type t , time_type dt , Err &xerr )
    {
        do_step_v5( system , x , t , dt , xerr );
    }


    /*
     * version 6 : do_step( sys , x , dxdt , t , dt , xerr )
     *
     * this version does not solve the forwarding problem, boost.range can not be used
     *
     * the disable is needed to avoid ambiguous overloads if state_type = time_type
         *
     * the disable is needed to avoid ambiguous overloads if state_type = time_type
 */
    template< class System , class StateInOut , class DerivInOut , class Err >
    typename boost::disable_if< boost::is_same< StateInOut , time_type > , void >::type
    do_step( System system , StateInOut &x , DerivInOut &dxdt , time_type t , time_type dt , Err &xerr )
    {
        m_first_call = true;
        this->stepper().do_step_impl( system , x , dxdt , t , x , dxdt , dt , xerr );
    }


    /*
     * version 7 : do_step( sys , in , t , out , dt , xerr )
     *
     * this version does not solve the forwarding problem, boost.range can not be used
     */
    template< class System , class StateIn , class StateOut , class Err >
    void do_step( System system , const StateIn &in , time_type t , StateOut &out , time_type dt , Err &xerr )
    {
        if( m_resizer.adjust_size( in , detail::bind( &internal_stepper_base_type::template resize_impl< StateIn > , detail::ref( *this ) , detail::_1 ) ) || m_first_call )
        {
            initialize( system , in , t );
        }
        this->stepper().do_step_impl( system , in , m_dxdt.m_v , t , out , m_dxdt.m_v , dt , xerr );
    }


    /*
     * version 8 : do_step( sys , in , dxdt_in , t , out , dxdt_out , dt , xerr )
     *
     * this version does not solve the forwarding problem, boost.range can not be used
     */
    template< class System , class StateIn , class DerivIn , class StateOut , class DerivOut , class Err >
    void do_step( System system , const StateIn &in , const DerivIn &dxdt_in , time_type t ,
            StateOut &out , DerivOut &dxdt_out , time_type dt , Err &xerr )
    {
        m_first_call = true;
        this->stepper().do_step_impl( system , in , dxdt_in , t , out , dxdt_out , dt , xerr );
    }


    template< class StateIn >
    void adjust_size( const StateIn &x )
    {
        resize_impl( x );
    }


    void reset( void )
    {
        m_first_call = true;
    }

    template< class DerivIn >
    void initialize( const DerivIn &deriv )
    {
        boost::numeric::omplext_odeint::copy( deriv , m_dxdt.m_v );
        m_first_call = false;
    }

    template< class System , class StateIn >
    void initialize( System system , const StateIn &x , time_type t )
    {
        typename omplext_odeint::unwrap_reference< System >::type &sys = system;
        sys( x , m_dxdt.m_v , t );
        m_first_call = false;
    }

    bool is_initialized( void ) const
    {
        return ! m_first_call;
    }



private:

    template< class System , class StateInOut >
    void do_step_v1( System system , StateInOut &x , time_type t , time_type dt )
    {
        if( m_resizer.adjust_size( x , detail::bind( &internal_stepper_base_type::template resize_impl< StateInOut > , detail::ref( *this ) , detail::_1 ) ) || m_first_call )
        {
            initialize( system , x , t );
        }
        this->stepper().do_step_impl( system , x , m_dxdt.m_v , t , x , m_dxdt.m_v , dt );
    }

    template< class System , class StateInOut , class Err >
    void do_step_v5( System system , StateInOut &x , time_type t , time_type dt , Err &xerr )
    {
        if( m_resizer.adjust_size( x , detail::bind( &internal_stepper_base_type::template resize_impl< StateInOut > , detail::ref( *this ) , detail::_1 ) ) || m_first_call )
        {
            initialize( system , x , t );
        }
        this->stepper().do_step_impl( system , x , m_dxdt.m_v , t , x , m_dxdt.m_v , dt , xerr );
    }

    template< class StateIn >
    bool resize_impl( const StateIn &x )
    {
        return adjust_size_by_resizeability( m_dxdt , x , typename is_resizeable<deriv_type>::type() );
    }


    stepper_type& stepper( void )
    {
        return *static_cast< stepper_type* >( this );
    }

    const stepper_type& stepper( void ) const
    {
        return *static_cast< const stepper_type* >( this );
    }


    resizer_type m_resizer;
    bool m_first_call;

protected:


    wrapped_deriv_type m_dxdt;
};


} // odeint
} // numeric
} // boost

#endif // BOOST_NUMERIC_ODEINT_STEPPER_BASE_EXPLICIT_ERROR_STEPPER_FSAL_BASE_HPP_INCLUDED
