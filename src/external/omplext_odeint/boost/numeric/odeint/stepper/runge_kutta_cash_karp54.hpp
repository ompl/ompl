/*
 [auto_generated]
 boost/numeric/odeint/stepper/runge_kutta_cash_karp54.hpp
 
 [begin_description]
 Implementation of the Runge Kutta Cash Karp 5(4) method. It uses the generic error stepper.
 [end_description]
 
 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky
 
 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
*/


#ifndef OMPLEXT_BOOST_NUMERIC_ODEINT_STEPPER_RUNGE_KUTTA_CASH_KARP54_HPP_INCLUDED
#define OMPLEXT_BOOST_NUMERIC_ODEINT_STEPPER_RUNGE_KUTTA_CASH_KARP54_HPP_INCLUDED

#include <boost/fusion/container.hpp>

#include <omplext_odeint/boost/numeric/odeint/stepper/explicit_error_generic_rk.hpp>
#include <omplext_odeint/boost/numeric/odeint/algebra/range_algebra.hpp>
#include <omplext_odeint/boost/numeric/odeint/algebra/default_operations.hpp>

#include <omplext_odeint/boost/numeric/odeint/util/state_wrapper.hpp>
#include <omplext_odeint/boost/numeric/odeint/util/is_resizeable.hpp>
#include <omplext_odeint/boost/numeric/odeint/util/resizer.hpp>

#include <boost/array.hpp>




namespace boost {
namespace numeric {
namespace omplext_odeint {

template< class Value = double >
struct rk54_ck_coefficients_a1 : boost::array< Value , 1 >
{
    rk54_ck_coefficients_a1( void )
    {
        (*this)[0] = static_cast< Value >( 1 )/static_cast< Value >( 5 );
    }
};

template< class Value = double >
struct rk54_ck_coefficients_a2 : boost::array< Value , 2 >
{
    rk54_ck_coefficients_a2( void )
    {
        (*this)[0] = static_cast<Value>( 3 )/static_cast<Value>( 40 );
        (*this)[1] = static_cast<Value>( 9 )/static_cast<Value>( 40 );
    }
};


template< class Value = double >
struct rk54_ck_coefficients_a3 : boost::array< Value , 3 >
{
    rk54_ck_coefficients_a3( void )
    {
        (*this)[0] = static_cast<Value>( 3 )/static_cast<Value>( 10 );
        (*this)[1] = static_cast<Value>( -9 )/static_cast<Value>( 10 );
        (*this)[2] = static_cast<Value>( 6 )/static_cast<Value>( 5 );
    }
};

template< class Value = double >
struct rk54_ck_coefficients_a4 : boost::array< Value , 4 >
{
    rk54_ck_coefficients_a4( void )
    {
        (*this)[0] = static_cast<Value>( -11 )/static_cast<Value>( 54 );
        (*this)[1] = static_cast<Value>( 5 )/static_cast<Value>( 2 );
        (*this)[2] = static_cast<Value>( -70 )/static_cast<Value>( 27 );
        (*this)[3] = static_cast<Value>( 35 )/static_cast<Value>( 27 );
    }
};

template< class Value = double >
struct rk54_ck_coefficients_a5 : boost::array< Value , 5 >
{
    rk54_ck_coefficients_a5( void )
    {
        (*this)[0] = static_cast<Value>( 1631 )/static_cast<Value>( 55296 );
        (*this)[1] = static_cast<Value>( 175 )/static_cast<Value>( 512 );
        (*this)[2] = static_cast<Value>( 575 )/static_cast<Value>( 13824 );
        (*this)[3] = static_cast<Value>( 44275 )/static_cast<Value>( 110592 );
        (*this)[4] = static_cast<Value>( 253 )/static_cast<Value>( 4096 );
    }
};

template< class Value = double >
struct rk54_ck_coefficients_b : boost::array< Value , 6 >
{
    rk54_ck_coefficients_b( void )
    {
        (*this)[0] = static_cast<Value>( 37 )/static_cast<Value>( 378 );
        (*this)[1] = static_cast<Value>( 0 );
        (*this)[2] = static_cast<Value>( 250 )/static_cast<Value>( 621 );
        (*this)[3] = static_cast<Value>( 125 )/static_cast<Value>( 594 );
        (*this)[4] = static_cast<Value>( 0 );
        (*this)[5] = static_cast<Value>( 512 )/static_cast<Value>( 1771 );
    }
};

template< class Value = double >
struct rk54_ck_coefficients_db : boost::array< Value , 6 >
{
    rk54_ck_coefficients_db( void )
    {
        (*this)[0] = static_cast<Value>( 37 )/static_cast<Value>( 378 ) - static_cast<Value>( 2825 )/static_cast<Value>( 27648 );
        (*this)[1] = static_cast<Value>( 0 );
        (*this)[2] = static_cast<Value>( 250 )/static_cast<Value>( 621 ) - static_cast<Value>( 18575 )/static_cast<Value>( 48384 );
        (*this)[3] = static_cast<Value>( 125 )/static_cast<Value>( 594 ) - static_cast<Value>( 13525 )/static_cast<Value>( 55296 );
        (*this)[4] = static_cast<Value>( -277 )/static_cast<Value>( 14336 );
        (*this)[5] = static_cast<Value>( 512 )/static_cast<Value>( 1771 ) - static_cast<Value>( 1 )/static_cast<Value>( 4 );
    }
};


template< class Value = double >
struct rk54_ck_coefficients_c : boost::array< Value , 6 >
{
    rk54_ck_coefficients_c( void )
    {
        (*this)[0] = static_cast<Value>(0);
        (*this)[1] = static_cast<Value>( 1 )/static_cast<Value>( 5 );
        (*this)[2] = static_cast<Value>( 3 )/static_cast<Value>( 10 );
        (*this)[3] = static_cast<Value>( 3 )/static_cast<Value>( 5 );
        (*this)[4] = static_cast<Value>( 1 );
        (*this)[5] = static_cast<Value>( 7 )/static_cast<Value>( 8 );
    }
};

template<
    class State ,
    class Value = double ,
    class Deriv = State ,
    class Time = Value ,
    class Algebra = range_algebra ,
    class Operations = default_operations ,
    class Resizer = initially_resizer
    >
class runge_kutta_cash_karp54 : public explicit_error_generic_rk< 6 , 5 , 5 , 4 ,
        State , Value , Deriv , Time , Algebra , Operations , Resizer >
{

public:

    typedef explicit_error_generic_rk< 6 , 5 , 5 , 4 , State , Value , Deriv , Time ,
                               Algebra , Operations , Resizer > stepper_base_type;

    typedef typename stepper_base_type::state_type state_type;
    typedef typename stepper_base_type::wrapped_state_type wrapped_state_type;
    typedef typename stepper_base_type::value_type value_type;
    typedef typename stepper_base_type::deriv_type deriv_type;
    typedef typename stepper_base_type::wrapped_deriv_type wrapped_deriv_type;
    typedef typename stepper_base_type::time_type time_type;
    typedef typename stepper_base_type::algebra_type algebra_type;
    typedef typename stepper_base_type::operations_type operations_type;
    typedef typename stepper_base_type::resizer_type resizer_typ;
    typedef typename stepper_base_type::stepper_type stepper_type;

    runge_kutta_cash_karp54( const algebra_type &algebra = algebra_type() ) : stepper_base_type(
	boost::fusion::make_vector( rk54_ck_coefficients_a1<Value>() ,
                                 rk54_ck_coefficients_a2<Value>() ,
                                 rk54_ck_coefficients_a3<Value>() ,
                                 rk54_ck_coefficients_a4<Value>() ,
                                 rk54_ck_coefficients_a5<Value>() ) ,
            rk54_ck_coefficients_b<Value>() , rk54_ck_coefficients_db<Value>() , rk54_ck_coefficients_c<Value>() ,
            algebra )
    { }
};

}
}
}

#endif // BOOST_NUMERIC_ODEINT_STEPPER_RUNGE_KUTTA_CASH_KARP54_HPP_INCLUDED
