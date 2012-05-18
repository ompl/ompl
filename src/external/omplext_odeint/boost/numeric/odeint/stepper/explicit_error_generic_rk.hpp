/*
 [auto_generated]
 boost/numeric/odeint/stepper/explicit_error_generic_rk.hpp

 [begin_description]
 Implementation of the generic Runge Kutta error stepper. Base class for many RK error steppers.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef OMPLEXT_BOOST_NUMERIC_ODEINT_STEPPER_EXPLICIT_ERROR_GENERIC_RK_HPP_INCLUDED
#define OMPLEXT_BOOST_NUMERIC_ODEINT_STEPPER_EXPLICIT_ERROR_GENERIC_RK_HPP_INCLUDED

#include <omplext_odeint/boost/numeric/odeint/stepper/base/explicit_stepper_and_error_stepper_base.hpp>

#include <omplext_odeint/boost/numeric/odeint/algebra/default_operations.hpp>
#include <omplext_odeint/boost/numeric/odeint/algebra/range_algebra.hpp>
#include <omplext_odeint/boost/numeric/odeint/stepper/detail/generic_rk_algorithm.hpp>
#include <omplext_odeint/boost/numeric/odeint/stepper/detail/generic_rk_call_algebra.hpp>
#include <omplext_odeint/boost/numeric/odeint/stepper/detail/generic_rk_operations.hpp>

#include <omplext_odeint/boost/numeric/odeint/util/state_wrapper.hpp>
#include <omplext_odeint/boost/numeric/odeint/util/is_resizeable.hpp>
#include <omplext_odeint/boost/numeric/odeint/util/resizer.hpp>


namespace mpl = boost::mpl;
namespace fusion = boost::fusion;


namespace boost {
namespace numeric {
namespace omplext_odeint {

template<
size_t StageCount,
size_t Order,
size_t StepperOrder ,
size_t ErrorOrder ,
class State ,
class Value = double ,
class Deriv = State ,
class Time = Value ,
class Algebra = range_algebra ,
class Operations = default_operations ,
class Resizer = initially_resizer
>
class explicit_error_generic_rk
: public explicit_stepper_and_error_stepper_base<
  explicit_error_generic_rk< StageCount , Order , StepperOrder , ErrorOrder , State ,
  Value , Deriv , Time , Algebra , Operations , Resizer > ,
  Order , StepperOrder , ErrorOrder , State , Value , Deriv , Time , Algebra ,
  Operations , Resizer >
{

public:

    typedef explicit_stepper_and_error_stepper_base<
            explicit_error_generic_rk< StageCount , Order , StepperOrder , ErrorOrder , State ,
            Value , Deriv , Time , Algebra , Operations , Resizer > ,
            Order , StepperOrder , ErrorOrder , State , Value , Deriv , Time , Algebra ,
            Operations , Resizer > stepper_base_type;

    typedef typename stepper_base_type::state_type state_type;
    typedef typename stepper_base_type::wrapped_state_type wrapped_state_type;
    typedef typename stepper_base_type::value_type value_type;
    typedef typename stepper_base_type::deriv_type deriv_type;
    typedef typename stepper_base_type::wrapped_deriv_type wrapped_deriv_type;
    typedef typename stepper_base_type::time_type time_type;
    typedef typename stepper_base_type::algebra_type algebra_type;
    typedef typename stepper_base_type::operations_type operations_type;
    typedef typename stepper_base_type::resizer_type resizer_type;
    //typedef typename stepper_base_type::stepper_type stepper_type;
    typedef explicit_error_generic_rk< StageCount , Order , StepperOrder , ErrorOrder , State ,
            Value , Deriv , Time , Algebra , Operations , Resizer > stepper_type;

    typedef detail::generic_rk_algorithm< StageCount , Value , Algebra , Operations > rk_algorithm_type;

    typedef typename rk_algorithm_type::coef_a_type coef_a_type;
    typedef typename rk_algorithm_type::coef_b_type coef_b_type;
    typedef typename rk_algorithm_type::coef_c_type coef_c_type;

    static const size_t stage_count = StageCount;

private:


public:

    // we use an explicit_generic_rk to do the normal rk step
    // and add a separate calculation of the error estimate afterwards
    explicit_error_generic_rk( const coef_a_type &a ,
            const coef_b_type &b ,
            const coef_b_type &b2 ,
            const coef_c_type &c ,
            const algebra_type &algebra = algebra_type() )
    : stepper_base_type( algebra ) , m_rk_algorithm( a , b , c ) , m_b2( b2 )
    { }


    template< class System , class StateIn , class DerivIn , class StateOut , class Err >
    void do_step_impl( System system , const StateIn &in , const DerivIn &dxdt ,
            const time_type &t , StateOut &out , const time_type &dt , Err &xerr )
    {
        // normal step
        do_step_impl( system , in , dxdt , t , out , dt );

        // additionally, perform the error calculation
        detail::template generic_rk_call_algebra< StageCount , algebra_type >()( stepper_base_type::m_algebra ,
                xerr , dxdt , m_F , detail::generic_rk_scale_sum_err< StageCount , operations_type , value_type , time_type >( m_b2 , dt) );
    }

    template< class System , class StateIn , class DerivIn , class StateOut >
    void do_step_impl( System system , const StateIn &in , const DerivIn &dxdt ,
            const time_type &t , StateOut &out , const time_type &dt )
    {
        m_resizer.adjust_size( in , detail::bind( &stepper_type::template resize_impl< StateIn > , detail::ref( *this ) , detail::_1 ) );

        // actual calculation done in generic_rk.hpp
        m_rk_algorithm.do_step( stepper_base_type::m_algebra , system , in , dxdt , t , out , dt , m_x_tmp.m_v , m_F );
    }

    template< class StateType >
    void adjust_size( const StateType &x )
    {
        resize_impl( x );
        stepper_base_type::adjust_size( x );
    }


private:

    template< class StateIn >
    bool resize_impl( const StateIn &x )
    {
        bool resized( false );
        resized |= adjust_size_by_resizeability( m_x_tmp , x , typename is_resizeable<state_type>::type() );
        for( size_t i = 0 ; i < StageCount-1 ; ++i )
        {
            resized |= adjust_size_by_resizeability( m_F[i] , x , typename is_resizeable<deriv_type>::type() );
        }
        return resized;
    }


    rk_algorithm_type m_rk_algorithm;
    coef_b_type m_b2;

    resizer_type m_resizer;

    wrapped_state_type m_x_tmp;
    wrapped_deriv_type m_F[StageCount-1];


};

}
}
}


#endif // BOOST_NUMERIC_ODEINT_STEPPER_EXPLICIT_ERROR_GENERIC_RK_HPP_INCLUDED
