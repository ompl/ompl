/*
 [auto_generated]
 boost/numeric/odeint/stepper/bulirsch_stoer_dense_out.hpp

 [begin_description]
 Implementaiton of the Burlish-Stoer method with dense output
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef OMPLEXT_BOOST_NUMERIC_ODEINT_STEPPER_BULIRSCH_STOER_DENSE_OUT_HPP_INCLUDED
#define OMPLEXT_BOOST_NUMERIC_ODEINT_STEPPER_BULIRSCH_STOER_DENSE_OUT_HPP_INCLUDED


#include <iostream>

#include <algorithm>

#include <omplext_odeint/boost/numeric/odeint/util/bind.hpp>

#include <boost/math/special_functions/binomial.hpp>

#include <omplext_odeint/boost/numeric/odeint/stepper/controlled_runge_kutta.hpp>
#include <omplext_odeint/boost/numeric/odeint/stepper/modified_midpoint.hpp>
#include <omplext_odeint/boost/numeric/odeint/stepper/controlled_step_result.hpp>
#include <omplext_odeint/boost/numeric/odeint/algebra/range_algebra.hpp>
#include <omplext_odeint/boost/numeric/odeint/algebra/default_operations.hpp>
#include <omplext_odeint/boost/numeric/odeint/stepper/detail/macros.hpp>

#include <omplext_odeint/boost/numeric/odeint/util/state_wrapper.hpp>
#include <omplext_odeint/boost/numeric/odeint/util/is_resizeable.hpp>
#include <omplext_odeint/boost/numeric/odeint/util/resizer.hpp>

#include <boost/type_traits.hpp>


namespace boost {
namespace numeric {
namespace omplext_odeint {

/*

template< class T , size_t N >
std::ostream& operator<<( std::ostream& output , const boost::array< T , N >& a ) {
    output << "[ " << a[0] ;
    for( size_t n = 1 ; n<N ; ++n )
        output << " , " << a[n];
    output << " ]";
    return output;  // for multiple << operators.
}

*/




template<
class State ,
class Value = double ,
class Deriv = State ,
class Time = Value ,
class Algebra = range_algebra ,
class Operations = default_operations ,
class Resizer = initially_resizer
>
class bulirsch_stoer_dense_out {

private:

    void copy_pointers( const bulirsch_stoer_dense_out &bs )
    {
        if( bs.m_current_state == (&bs.m_x1.m_v ) )
        {
            m_current_state = &m_x1.m_v;
            m_old_state = &m_x2.m_v;
        }
        else
        {
            m_current_state = &m_x2.m_v;
            m_old_state = &m_x1.m_v;
        }
        if( bs.m_current_deriv == ( &bs.m_dxdt1.m_v ) )
        {
            m_current_deriv = &m_dxdt1.m_v;
            m_old_deriv = &m_dxdt2.m_v;
        }
        else
        {
            m_current_deriv = &m_dxdt2.m_v;
            m_old_deriv = &m_dxdt1.m_v;
        }
    }

public:

    typedef State state_type;
    typedef Value value_type;
    typedef Deriv deriv_type;
    typedef Time time_type;
    typedef Algebra algebra_type;
    typedef Operations operations_type;
    typedef Resizer resizer_type;
    typedef state_wrapper< state_type > wrapped_state_type;
    typedef state_wrapper< deriv_type > wrapped_deriv_type;
    typedef dense_output_stepper_tag stepper_category;

    typedef bulirsch_stoer_dense_out< State , Value , Deriv , Time , Algebra , Operations , Resizer > controlled_error_bs_type;

    typedef std::vector< time_type > value_vector;
    typedef std::vector< value_vector > value_matrix;
    typedef std::vector< size_t > int_vector;
    typedef std::vector< wrapped_state_type > state_vector_type;
    typedef std::vector< wrapped_deriv_type > deriv_vector_type;
    typedef std::vector< deriv_vector_type > deriv_table_type;

    bulirsch_stoer_dense_out(
            time_type eps_abs = 1E-6 , time_type eps_rel = 1E-6 ,
            time_type factor_x = 1.0 , time_type factor_dxdt = 1.0 ,
            bool control_interpolation = false )
    : m_error_checker( eps_abs , eps_rel , factor_x, factor_dxdt ) ,
      m_control_interpolation( control_interpolation) ,
      m_k_max(8) ,
      m_last_step_rejected( false ) , m_first( true ) ,
      m_dt_last( 1.0E30 ) ,
      m_k_final(0) ,
      m_current_state( &m_x1.m_v ) , m_old_state( &m_x2.m_v ) ,
      m_current_deriv( &m_dxdt1.m_v ) , m_old_deriv( &m_dxdt2.m_v ) ,
      m_error( m_k_max ) ,
      m_interval_sequence( m_k_max+1 ) ,
      m_coeff( m_k_max+1 ) ,
      m_cost( m_k_max+1 ) ,
      m_table( m_k_max ) ,
      m_mp_states( m_k_max+1 ) ,
      m_derivs( m_k_max+1 ) ,
      m_diffs( 2*m_k_max+1 ) ,
      STEPFAC1( 0.65 ) , STEPFAC2( 0.94 ) , STEPFAC3( 0.02 ) , STEPFAC4( 4.0 ) , KFAC1( 0.8 ) , KFAC2( 0.9 )
    {
        for( unsigned short i = 0; i < m_k_max+1; i++ )
        {
            m_interval_sequence[i] = 2 + 4*i;  // 2 6 10 14 ...
            m_derivs[i].resize( m_interval_sequence[i] );
            if( i == 0 )
                m_cost[i] = m_interval_sequence[i];
            else
                m_cost[i] = m_cost[i-1] + m_interval_sequence[i];
            m_coeff[i].resize(i);
            for( size_t k = 0 ; k < i ; ++k  )
            {
                const time_type r = static_cast< time_type >( m_interval_sequence[i] ) / static_cast< time_type >( m_interval_sequence[k] );
                m_coeff[i][k] = 1.0 / ( r*r - static_cast< time_type >( 1.0 ) ); // coefficients for extrapolation
                //std::cout << i << "," << k << " " << m_coeff[i][k] << '\t' ;
            }
            //std ::cout << std::endl;
            // crude estimate of optimal order
            const time_type logfact( -log10( std::max( eps_rel , 1.0E-12 ) ) * 0.6 + 0.5 );
            m_current_k_opt = std::max( 1 , std::min( static_cast<int>( m_k_max-1 ) , static_cast<int>( logfact ) ));
            //m_current_k_opt = m_k_max - 1;
            //std::cout << m_cost[i] << std::endl;
        }
        int num = 1;
        for( int i = 2*(m_k_max) ; i >=0  ; i-- )
        {
            m_diffs[i].resize( num );
            //std::cout << "m_diffs[" << i << "] size: " << num << std::endl;
            num += (i+1)%2;
        }
    }

    bulirsch_stoer_dense_out( const bulirsch_stoer_dense_out &bs )
    : m_error_checker( bs.m_error_checker ) ,
      m_midpoint( bs.m_midpoint ) ,
      m_control_interpolation( bs.m_control_interpolation) ,
      m_k_max( bs.m_k_max ) ,
      m_last_step_rejected( bs.m_last_step_rejected ) , m_first( bs.m_first ) ,
      m_t( bs.m_t ) , m_dt( bs.m_dt ) , m_dt_last( bs.m_dt_last ) , m_t_last( bs.m_t_last ) ,
      m_current_k_opt( bs.m_current_k_opt ) , m_k_final( bs.m_k_final ) ,
      m_x1( bs.m_x1 ) , m_x2( bs.m_x2 ) , m_dxdt1( bs.m_dxdt1 ) , m_dxdt2( bs.m_dxdt2 ) ,
      m_current_state( &m_x1.m_v ) , m_old_state( &m_x2.m_v ) ,
      m_current_deriv( &m_dxdt1.m_v ) , m_old_deriv( &m_dxdt2.m_v ) ,
      m_error( bs.m_error ) ,
      m_interval_sequence( bs.m_interval_sequence ) ,
      m_coeff( bs.m_coeff ) ,
      m_cost( bs.m_cost ) ,
      m_table( bs.m_table ) ,
      m_mp_states( bs.m_mp_states ) ,
      m_derivs( bs.m_derivs ) ,
      m_diffs( bs.m_diffs ) ,
      STEPFAC1( bs.STEPFAC1 ) , STEPFAC2( bs.STEPFAC2 ) , STEPFAC3( bs.STEPFAC3 ) , STEPFAC4( bs.STEPFAC4 ) , KFAC1( bs.KFAC1 ) , KFAC2( bs.KFAC2 )
    {
        copy_pointers( bs );
    }

    bulirsch_stoer_dense_out& operator = ( const bulirsch_stoer_dense_out &bs )
    {
        m_error_checker = bs.m_error_checker;
        m_midpoint = bs.m_midpoint;
        m_control_interpolation = bs.m_control_interpolation;
        m_k_max = bs.m_k_max;
        m_last_step_rejected = bs.m_last_step_rejected;
        m_first = bs.m_first;
        m_t = bs.m_t; m_dt = bs.m_dt; m_dt_last = bs.m_dt_last; m_t_last = bs.m_t_last;
        m_current_k_opt = bs.m_current_k_opt; m_k_final = bs.m_k_final;
        m_x1 = bs.m_x1; m_x2 = bs.m_x2; m_dxdt1 = bs.m_dxdt1; m_dxdt2 = bs.m_dxdt2;
        m_current_state = &m_x1.m_v; m_old_state = &m_x2.m_v;
        m_current_deriv = &m_dxdt1.m_v; m_old_deriv = &m_dxdt2.m_v;
        m_error = bs.m_error;
        m_interval_sequence = bs.m_interval_sequence;
        m_coeff = bs.m_coeff;
        m_cost = bs.m_cost;
        m_table = bs.m_table;
        m_mp_states = bs.m_mp_states;
        m_derivs = bs.m_derivs;
        m_diffs = bs.m_diffs;
        STEPFAC1 = bs.STEPFAC1; STEPFAC2 = bs.STEPFAC2; STEPFAC3 = bs.STEPFAC3; STEPFAC4 = bs.STEPFAC4; KFAC1 = bs.KFAC1; KFAC2 = bs.KFAC2;
    }

/*
    template< class System , class StateInOut >
    controlled_step_result try_step( System system , StateInOut &x , time_type &t , time_type &dt )
    {
        m_xnew_resizer.adjust_size( x , detail::bind( &controlled_error_bs_type::template resize_m_xnew< StateInOut > , detail::ref( *this ) , detail::_1 ) );
        controlled_step_result res = try_step( system , x , t , m_xnew.m_v , dt );
        if( ( res == success_step_size_increased ) || ( res == success_step_size_unchanged ) )
        {
            boost::numeric::omplext_odeint::copy( m_xnew.m_v , x );
        }
        return res;
    }
*/

    template< class System , class StateIn , class DerivIn , class StateOut , class DerivOut >
    controlled_step_result try_step( System system , const StateIn &in , const DerivIn &dxdt , time_type &t , StateOut &out , DerivOut &dxdt_new , time_type &dt )
    {
        static const time_type val1( static_cast< time_type >( 1.0 ) );

        typename omplext_odeint::unwrap_reference< System >::type &sys = system;
//        if( m_resizer.adjust_size( in , detail::bind( &controlled_error_bs_type::template resize_impl< StateIn > , detail::ref( *this ) , detail::_1 ) ) )
//            reset(); // system resized -> reset
//        if( dt != m_dt_last )
//            reset(); // step size changed from outside -> reset

        bool reject( true );

        value_vector h_opt( m_k_max+1 );
        value_vector work( m_k_max+1 );

        m_k_final = 0;
        time_type new_h = dt;

        //std::cout << "t=" << t <<", dt=" << dt << ", k_opt=" << m_current_k_opt << ", first: " << m_first << std::endl;

        for( size_t k = 0 ; k <= m_current_k_opt+1 ; k++ )
        {
            //std::cout << "k=" << k <<" (steps=" << m_interval_sequence[k] << "): " << std::endl;
            m_midpoint.set_steps( m_interval_sequence[k] );
            if( k == 0 )
            {
                m_midpoint.do_step( sys , in , dxdt , t , out , dt , m_mp_states[k].m_v , m_derivs[k]);
            }
            else
            {
                m_midpoint.do_step( sys , in , dxdt , t , m_table[k-1].m_v , dt , m_mp_states[k].m_v , m_derivs[k] );
                extrapolate( k , m_table , m_coeff , out );
                // get error estimate
                m_algebra.for_each3( m_err.m_v , out , m_table[0].m_v ,
                        typename operations_type::template scale_sum2< time_type , time_type >( val1 , -val1 ) );
                const time_type error = m_error_checker.error( m_algebra , in , dxdt , m_err.m_v , dt );
                h_opt[k] = calc_h_opt( dt , error , k );
                work[k] = m_cost[k]/h_opt[k];
                //std::cout << '\t' << "h_opt=" << h_opt[k] << ", work=" << work[k] << std::endl;
                //std::cout << '\t' << "error: " << error << std::endl;

                m_k_final = k;

                if( (k == m_current_k_opt-1) || m_first )
                { // convergence before k_opt ?
                    if( error < 1.0 )
                    {
                        //convergence
                        reject = false;
                        if( (work[k] < KFAC2*work[k-1]) || (m_current_k_opt <= 2) )
                        {
                            // leave order as is (except we were in first round)
                            m_current_k_opt = std::min( static_cast<int>(m_k_max)-1 , std::max( 2 , static_cast<int>(k)+1 ) );
                            new_h = h_opt[k] * m_cost[k+1]/m_cost[k];
                        } else {
                            m_current_k_opt = std::min( static_cast<int>(m_k_max)-1 , std::max( 2 , static_cast<int>(k) ) );
                            new_h = h_opt[k];
                        }
                        break;
                    }
                    else if( should_reject( error , k ) && !m_first )
                    {
                        reject = true;
                        new_h = h_opt[k];
                        break;
                    }
                }
                if( k == m_current_k_opt )
                { // convergence at k_opt ?
                    if( error < 1.0 )
                    {
                        //convergence
                        reject = false;
                        if( (work[k-1] < KFAC2*work[k]) )
                        {
                            m_current_k_opt = std::max( 2 , static_cast<int>(m_current_k_opt)-1 );
                            new_h = h_opt[m_current_k_opt];
                        }
                        else if( (work[k] < KFAC2*work[k-1]) && !m_last_step_rejected )
                        {
                            m_current_k_opt = std::min( static_cast<int>(m_k_max)-1 , static_cast<int>(m_current_k_opt)+1 );
                            new_h = h_opt[k]*m_cost[m_current_k_opt]/m_cost[k];
                        } else
                            new_h = h_opt[m_current_k_opt];
                        break;
                    }
                    else if( should_reject( error , k ) )
                    {
                        reject = true;
                        new_h = h_opt[m_current_k_opt];
                        break;
                    }
                }
                if( k == m_current_k_opt+1 )
                { // convergence at k_opt+1 ?
                    if( error < 1.0 )
                    {   //convergence
                        reject = false;
                        if( work[k-2] < KFAC2*work[k-1] )
                            m_current_k_opt = std::max( 2 , static_cast<int>(m_current_k_opt)-1 );
                        if( (work[k] < KFAC2*work[m_current_k_opt]) && !m_last_step_rejected )
                            m_current_k_opt = std::min( static_cast<int>(m_k_max)-1 , static_cast<int>(k) );
                        new_h = h_opt[m_current_k_opt];
                    } else
                    {
                        reject = true;
                        new_h = h_opt[m_current_k_opt];
                    }
                    break;
                }
            }
        }

        if( !reject )
        {

            //calculate dxdt for next step and dense output
            sys( out , dxdt_new , t+dt );

            //prepare dense output
            value_type error = prepare_dense_output( m_k_final , in , dxdt , out , dxdt_new , dt );

            if( error > static_cast<value_type>(10) ) // we are not as accurate for interpolation as for the steps
            {
                reject = true;
                new_h = dt * std::pow( error , static_cast<value_type>(-1)/(2*m_k_final+2) );
                //std::cout << "####### rejected #######! (t=" << t << ") interpolation error: " << error << " , new dt: " << new_h << std::endl;
            } else {
                //std::cout << "####### accepted ####### - new k: " << m_current_k_opt << ", new stepsize: " << new_h << std::endl;
                //increase time
                t += dt;
            }
        }// else  std::cout << "####### rejected #######!" << std::endl;

        //set next stepsize
        if( !m_last_step_rejected || (new_h < dt) )
            dt = new_h;

        m_last_step_rejected = reject;
        if( reject )
            return fail;
        else
            return success;
    }

    template< class StateType >
    void initialize( const StateType &x0 , const time_type &t0 , const time_type &dt0 )
    {
        m_resizer.adjust_size( x0 , detail::bind( &controlled_error_bs_type::template resize_impl< StateType > , detail::ref( *this ) , detail::_1 ) );
        boost::numeric::omplext_odeint::copy( x0 , *m_current_state );
        m_t = t0;
        m_dt = dt0;
        reset();
    }


    /*  =======================================================
     *  the actual step method that should be called from outside (maybe make try_step private?)
     */
    template< class System >
    std::pair< time_type , time_type > do_step( System system )
    {
        const size_t max_count = 1000;

        if( m_first )
        {
            typename omplext_odeint::unwrap_reference< System >::type &sys = system;
            sys( *m_current_state , *m_current_deriv , m_t );
        }

        controlled_step_result res = fail;
        m_t_last = m_t;
        size_t count = 0;
        while( res == fail )
        {
            res = try_step( system , *m_current_state , *m_current_deriv , m_t , *m_old_state , *m_old_deriv , m_dt );
            m_first = false;
            if( count++ == max_count )
                throw std::overflow_error( "bulirsch_stoer : too much iterations!");
        }
        std::swap( m_current_state , m_old_state );
        std::swap( m_current_deriv , m_old_deriv );
        return std::make_pair( m_t_last , m_t );
    }

    /* performs the interpolation from a calculated step */
    template< class StateOut >
    void calc_state( const time_type &t , StateOut &x )
    {
        //std::cout << "===========" << std::endl << "doing interpolation for t=" << t << std::endl;
        do_interpolation( t , x );
        //std::cout << "===========" << std::endl;
    }


    const state_type& current_state( void ) const
    {
        return *m_current_state;
    }

    const time_type& current_time( void ) const
    {
        return m_t;
    }

    const time_type& previous_state( void ) const
    {
        return *m_old_state;
    }

    const time_type& previous_time( void ) const
    {
        return m_t_last;
    }

    const time_type& current_time_step( void ) const
    {
        return m_dt;
    }


    void reset()
    {
        m_first = true;
        m_last_step_rejected = false;
    }


    template< class StateIn >
    void adjust_size( const StateIn &x )
    {
        resize_impl( x );
        m_midpoint.adjust_size();
    }


private:

    template< class StateInOut , class StateVector >
    void extrapolate( const size_t k , StateVector &table , const value_matrix &coeff , StateInOut &xest , const size_t order_start_index = 0 )
    //polynomial extrapolation, see http://www.nr.com/webnotes/nr3web21.pdf
    {
        static const time_type val1 = static_cast< time_type >( 1.0 );
        for( int j=k-1 ; j>0 ; --j )
        {
            m_algebra.for_each3( table[j-1].m_v , table[j].m_v , table[j-1].m_v ,
                    typename operations_type::template scale_sum2< time_type , time_type >( val1 + coeff[k + order_start_index][j + order_start_index] ,
                            -coeff[k + order_start_index][j + order_start_index] ) );
        }
        m_algebra.for_each3( xest , table[0].m_v , xest ,
                typename operations_type::template scale_sum2< time_type , time_type >( val1 + coeff[k + order_start_index][0 + order_start_index] ,
                        -coeff[k + order_start_index][0 + order_start_index]) );
    }


    template< class StateVector >
    void extrapolate_dense_out( const size_t k , StateVector &table , const value_matrix &coeff , const size_t order_start_index = 0 )
    //polynomial extrapolation, see http://www.nr.com/webnotes/nr3web21.pdf
    {
        // result is written into table[0]
        //std::cout << "extrapolate k=" << k << ":" << std::endl;
        static const time_type val1 = static_cast< time_type >( 1.0 );
        for( int j=k ; j>1 ; --j )
        {
            //std::cout << '\t' << coeff[k + order_start_index][j + order_start_index - 1];
            m_algebra.for_each3( table[j-1].m_v , table[j].m_v , table[j-1].m_v ,
                    typename operations_type::template scale_sum2< time_type , time_type >( val1 + coeff[k + order_start_index][j + order_start_index - 1] ,
                            -coeff[k + order_start_index][j + order_start_index - 1] ) );
        }
        //std::cout << std::endl << coeff[k + order_start_index][order_start_index] << std::endl;
        m_algebra.for_each3( table[0].m_v , table[1].m_v , table[0].m_v ,
                typename operations_type::template scale_sum2< time_type , time_type >( val1 + coeff[k + order_start_index][order_start_index] ,
                        -coeff[k + order_start_index][order_start_index]) );
    }

    time_type calc_h_opt( const time_type h , const value_type error , const size_t k ) const
    {
        time_type expo=1.0/(m_interval_sequence[k-1]);
        time_type facmin = std::pow( STEPFAC3 , expo );
        time_type fac;
        if (error == 0.0)
            fac=1.0/facmin;
        else
        {
            fac = STEPFAC2 / std::pow( error / STEPFAC1 , expo );
            fac = std::max( facmin/STEPFAC4 , std::min( 1.0/facmin , fac ) );
        }
        return std::abs(h*fac); //std::min( 0.1 , std::abs(h*fac) );
    }

    bool in_convergence_window( const size_t k ) const
    {
        if( (k == m_current_k_opt-1) && !m_last_step_rejected )
            return true; // decrease order only if last step was not rejected
        return ( (k == m_current_k_opt) || (k == m_current_k_opt+1) );
    }

    bool should_reject( const time_type error , const size_t k ) const
    {
        if( (k == m_current_k_opt-1) )
        {
            const time_type d = m_interval_sequence[m_current_k_opt] * m_interval_sequence[m_current_k_opt+1] /
                    (m_interval_sequence[0]*m_interval_sequence[0]);
            //step will fail, criterion 17.3.17 in NR
            return ( error > d*d );
        }
        else if( k == m_current_k_opt )
        {
            const time_type d = m_interval_sequence[m_current_k_opt+1] / m_interval_sequence[0];
            return ( error > d*d );
        } else
            return error > 1.0;
    }

    template< class StateIn1 , class DerivIn1 , class StateIn2 , class DerivIn2 >
    value_type prepare_dense_output( const int k , const StateIn1 &x_start , const DerivIn1 &dxdt_start ,
            const StateIn2 & /* x_end */ , const DerivIn2 & /*dxdt_end */ , const time_type dt )  // k is the order to which the result was approximated
    {

        // compute the coefficients of the interpolation polynomial
        // we parametrize the interval t .. t+dt by theta = -1 .. 1
        // we use 2k+3 values at the interval center theta=0 to obtain the interpolation coefficients
        // the values are x(t+dt/2) and the derivatives dx/dt , ... d^(2k+2) x / dt^(2k+2) at the midpoints
        // the derivatives are approximated via finite differences
        // all values are obtained from interpolation of the results from the increasing orders of the midpoint calls

        // calculate finite difference approximations to derivatives at the midpoint
        for( int j = 0 ; j<=k ; j++ )
        {
            const time_type d = m_interval_sequence[j] / static_cast<time_type>(2*dt);
            time_type f = 1.0; //factor 1/2 here because our interpolation interval has length 2 !!!
            for( int kappa = 0 ; kappa <= 2*j+1 ; ++kappa )
            {
                calculate_finite_difference( j , kappa , f , dxdt_start );
                f *= d;
            }
            //std::cout << "x_mp[" << j << "] = " << m_mp_states[j].m_v << std::endl;

            if( j > 0 )
                extrapolate_dense_out( j , m_mp_states , m_coeff );
        }

        //std::cout << "a_0 = " << m_mp_states[0].m_v << std::endl;

        time_type d = dt/2;

        // extrapolate finite differences
        for( int kappa = 0 ; kappa<=2*k+1 ; kappa++ )
        {
            for( int j=1 ; j<=(k-kappa/2) ; ++j )
                extrapolate_dense_out( j , m_diffs[kappa] , m_coeff , kappa/2 );

            // extrapolation results are now stored in m_diffs[kappa][0]
            //std::cout << "extrapolation result: " << m_diffs[kappa][0].m_v << std::endl;

            // divide kappa-th derivative by kappa because we need these terms for dense output interpolation
            m_algebra.for_each1( m_diffs[kappa][0].m_v , typename operations_type::template scale< value_type >( static_cast<value_type>(d) ) );

            d *= dt/(2*(kappa+2));

            //std::cout << "a_" << kappa+1 << " = " << m_diffs[kappa][0].m_v << std::endl;
        }

        // dense output coefficients a_0 is stored in m_mp_states[0], a_i for i = 1...2k are stored in m_diffs[i-1][0]

        // the error is just the highest order coefficient of the interpolation polynomial
        // this is because we use only the midpoint theta=0 as support for the interpolation (remember that theta = -1 .. 1)

        double error = 0.0;
        if( m_control_interpolation )
        {
            boost::numeric::omplext_odeint::copy( m_diffs[2*k+1][0].m_v , m_err.m_v );
            error = m_error_checker.error( m_algebra , x_start , dxdt_start , m_err.m_v , dt );
        }

        return error;

        // calculate coefficient a_{2k+1} = (2k+5)/4 * x_end - (2k+5)/4 * x_start - 1/4 * dxdt_end - 1/4 * dxdt_start + sum_i=0...k-1 (i-k-2)*a_{2i+1}


        //std::cout << std::endl << x_start << std::endl << x_end << std::endl;
        //std::cout << std::endl << dxdt_start << std::endl << dxdt_end << std::endl << std::endl;

        // we don't use additional terms in the polynomial, the following calculations are thus not required

/*
        m_algebra.for_each5( m_a1.m_v , x_end , x_start , dxdt_end , dxdt_start ,
                typename operations_type::template scale_sum4< value_type >( static_cast<value_type>(2*k+5)/static_cast<value_type>(4),
                                                                               static_cast<value_type>(-2*k-5)/static_cast<value_type>(4),
                                                                               static_cast<value_type>(-dt)/static_cast<value_type>(8) ,
                                                                               static_cast<value_type>(-dt)/static_cast<value_type>(8) ) );
        for( int i = 0 ; i<k ; ++i )
            m_algebra.for_each3( m_a1.m_v , m_a1.m_v , m_diffs[2*i][0].m_v ,
                    typename operations_type::template scale_sum2< value_type >( 1 , i-k-2 ) );

        //std::cout << "a_" << 2*k+1 << " = " << m_a1.m_v << std::endl;

        // calculate coefficient a_{2k+2} = (k+2)/2 * x_end + (k+2)/2 * x_start - 1/4 * dxdt_end + 1/4 * dxdt_start + (k+2)/2 * x_mp + sum_i=1...k (i-k-2)*a_{2i}
        m_algebra.for_each6( m_a2.m_v , x_end , x_start , dxdt_end , dxdt_start , m_mp_states[0].m_v ,
                typename operations_type::template scale_sum5< value_type >( static_cast<value_type>(k+2)/static_cast<value_type>(2),
                                                                               static_cast<value_type>(k+2)/static_cast<value_type>(2),
                                                                               static_cast<value_type>(-dt)/static_cast<value_type>(8) ,
                                                                               static_cast<value_type>(dt)/static_cast<value_type>(8) ,
                                                                               static_cast<value_type>(-k-2) ) );
        for( int i = 1 ; i<=k ; ++i )
            m_algebra.for_each3( m_a2.m_v , m_a2.m_v , m_diffs[2*i-1][0].m_v ,
                    typename operations_type::template scale_sum2< value_type >( 1 , i-k-2 ) );

        //std::cout << "a_" << 2*k+2 << " = " << m_a2.m_v << std::endl;

        // calculate coefficient a_{2k+3} = -(2k+3)/4 * x_end + (2k+3)/4 * x_start + 1/4 * dxdt_end + 1/4 * dxdt_start + sum_i=0...k-1 (k+1-i)*a_{2i+1}
        m_algebra.for_each5( m_a3.m_v , x_end , x_start , dxdt_end , dxdt_start ,
                typename operations_type::template scale_sum4< value_type >( static_cast<value_type>(-2*k-3)/static_cast<value_type>(4),
                                                                               static_cast<value_type>(2*k+3)/static_cast<value_type>(4),
                                                                               static_cast<value_type>(dt)/static_cast<value_type>(8) ,
                                                                               static_cast<value_type>(dt)/static_cast<value_type>(8) ) );
        for( int i = 0 ; i<k ; ++i )
            m_algebra.for_each3( m_a3.m_v , m_a3.m_v , m_diffs[2*i][0].m_v ,
                    typename operations_type::template scale_sum2< value_type >( 1 , k+1-i ) );

        //std::cout << "a_" << 2*k+3 << " = " << m_a3.m_v << std::endl;

        // calculate coefficient a_{2k+4} = -(k+1)/2 * x_end - (k+1)/2 * x_start + 1/4 * dxdt_end - 1/4 * dxdt_start - (k+1)/2 * x_mp + sum_i=0...k-1 (k+1-i)*a_{2i}
        m_algebra.for_each6( m_a4.m_v , x_end , x_start , dxdt_end , dxdt_start , m_mp_states[0].m_v ,
                typename operations_type::template scale_sum5< value_type >( static_cast<value_type>(-k-1)/static_cast<value_type>(2),
                                                                               static_cast<value_type>(-k-1)/static_cast<value_type>(2),
                                                                               static_cast<value_type>(dt)/static_cast<value_type>(8) ,
                                                                               static_cast<value_type>(-dt)/static_cast<value_type>(8),
                                                                               static_cast<value_type>(k+1) ) );
        for( int i = 1 ; i<=k ; ++i )
            m_algebra.for_each3( m_a4.m_v , m_a4.m_v , m_diffs[2*i-1][0].m_v ,
                    typename operations_type::template scale_sum2< value_type >( 1 , k+1-i ) );
*/
        //std::cout << "a_" << 2*k+4 << " = " << m_a4.m_v << std::endl;
    }

    template< class DerivIn >
    void calculate_finite_difference( const size_t j , const size_t kappa , const time_type fac , const DerivIn &dxdt )
    {
        const int m = m_interval_sequence[j]/2-1;
        if( kappa == 0) // no calculation required for 0th derivative of f
        {
            m_algebra.for_each2( m_diffs[0][j].m_v , m_derivs[j][m].m_v ,
                    typename operations_type::template scale_sum1< time_type >( fac ) );
            //std::cout << "j=" << j << ", kappa=" << kappa << ", m=" << m;
            //std::cout << ": m_diffs[" << kappa << "][" << j << "] = " << fac << " * f[" << m << "]  ";
            //std::cout << "(size(f)=" << m_derivs[j].size() << ") = " << m_diffs[0][j].m_v << std::endl;

        }
        else
        {
            //std::cout << m_derivs[j][1].m_v << " , " << dxdt << std::endl;

            // calculate the index of m_diffs for this kappa-j-combination
            const int j_diffs = j - kappa/2;

            //std::cout << "j=" << j << ", kappa=" << kappa << ", m=" << m << ": m_diffs[" << kappa << "][" << j_diffs << "] = " << fac << " ( 1*f[" << m+kappa << "]";

            m_algebra.for_each2( m_diffs[kappa][j_diffs].m_v , m_derivs[j][m+kappa].m_v ,
                    typename operations_type::template scale_sum1< time_type >( fac ) );
            time_type sign = -1.0;
            int c = 1;
             //computes the j-th order finite difference for the kappa-th derivative of f at t+dt/2 using function evaluations stored in m_derivs
            for( int i = m+static_cast<int>(kappa)-2 ; i >= m-static_cast<int>(kappa) ; i -= 2 )
            {
                if( i >= 0 )
                {
                    m_algebra.for_each3( m_diffs[kappa][j_diffs].m_v , m_diffs[kappa][j_diffs].m_v , m_derivs[j][i].m_v ,
                            typename operations_type::template scale_sum2< time_type , time_type >( 1.0 ,
                                    sign * fac * boost::math::binomial_coefficient< double >( kappa , c ) ) );
                    //std::cout << ( (sign > 0.0) ? " + " : " - " ) <<
                    //        boost::math::binomial_coefficient< double >( kappa , c ) << "*f[" << i << "]";
                }
                else
                {
                    m_algebra.for_each3( m_diffs[kappa][j_diffs].m_v , m_diffs[kappa][j_diffs].m_v , dxdt ,
                            typename operations_type::template scale_sum2< time_type , time_type >( 1.0 , sign * fac ) );
                    //std::cout << ( (sign > 0.0) ? " + " : " - " ) << "dxdt";
                }
                sign *= -1;
                ++c;
            }
            //std::cout << " ) = " << m_diffs[kappa][j_diffs].m_v << std::endl;
        }
    }

    template< class StateOut >
    void do_interpolation( const time_type t , StateOut &out )
    {
        // interpolation polynomial is defined for theta = -1 ... 1
        // m_k_final is the number of order-iterations done for the last step - it governs the order of the interpolation polynomial
        const time_type theta = 2*(t - m_t_last)/(m_t - m_t_last) - 1;
        //std::cout << "theta=" << theta << std::endl;
        //start with x = a0 + a_{2k+1} theta^{2k+1} + a_{2k+2} theta^{2k+2} + a_{2k+3} theta^{2k+3} + a_{2k+4} theta^{2k+4}
        //std::cout << "x = a_0 + ";

/*        m_algebra.for_each6( out , m_mp_states[0].m_v , m_a1.m_v , m_a2.m_v , m_a3.m_v , m_a4.m_v ,
                typename operations_type::template scale_sum5< time_type >(
                        static_cast<time_type>( 1 ) ,
                        std::pow( theta , 2*m_k_final+1 ) ,
                        std::pow( theta , 2*m_k_final+2 ) ,
                        std::pow( theta , 2*m_k_final+3 ) ,
                        std::pow( theta , 2*m_k_final+4 ) ) );
*/

        // we use only values at interval center, that is theta=0, for interpolation
        // our interpolation polynomial is thus of order 2k+2, hence we have 2k+3 terms

        boost::numeric::omplext_odeint::copy( m_mp_states[0].m_v , out );
        // add remaining terms: x += a_1 theta + a2 theta^2 + ... + a_{2k} theta^{2k}
        time_type theta_pow( theta );
        for( size_t i=0 ; i<=2*m_k_final+1 ; ++i )
        {
            //std::cout << "a_" << i+1 << " theta^" << i+1 << " = " << m_diffs[i][0].m_v[0] * std::pow( theta , i+1 ) << std::endl;
            m_algebra.for_each3( out , out , m_diffs[i][0].m_v ,
                typename operations_type::template scale_sum2< time_type >( static_cast<time_type>(1) , theta_pow ) );
            theta_pow *= theta;
        }
    }

    /* Resizer methods */
    template< class StateIn >
    bool resize_impl( const StateIn &x )
    {
        bool resized( false );

        resized |= adjust_size_by_resizeability( m_x1 , x , typename is_resizeable<state_type>::type() );
        resized |= adjust_size_by_resizeability( m_x2 , x , typename is_resizeable<state_type>::type() );
        resized |= adjust_size_by_resizeability( m_dxdt1 , x , typename is_resizeable<state_type>::type() );
        resized |= adjust_size_by_resizeability( m_dxdt2 , x , typename is_resizeable<state_type>::type() );
        resized |= adjust_size_by_resizeability( m_err , x , typename is_resizeable<state_type>::type() );

        for( size_t i = 0 ; i < m_k_max ; ++i )
            resized |= adjust_size_by_resizeability( m_table[i] , x , typename is_resizeable<state_type>::type() );
        for( size_t i = 0 ; i < m_k_max+1 ; ++i )
                    resized |= adjust_size_by_resizeability( m_mp_states[i] , x , typename is_resizeable<state_type>::type() );
        for( size_t i = 0 ; i < m_k_max+1 ; ++i )
            for( size_t j = 0 ; j < m_derivs[i].size() ; ++j )
                resized |= adjust_size_by_resizeability( m_derivs[i][j] , x , typename is_resizeable<deriv_type>::type() );
        for( size_t i = 0 ; i < 2*m_k_max+1 ; ++i )
            for( size_t j = 0 ; j < m_diffs[i].size() ; ++j )
                resized |= adjust_size_by_resizeability( m_diffs[i][j] , x , typename is_resizeable<deriv_type>::type() );

        return resized;
    }


    default_error_checker< value_type, algebra_type , operations_type > m_error_checker;
    modified_midpoint_dense_out< state_type , value_type , deriv_type , time_type , algebra_type , operations_type , resizer_type > m_midpoint;

    bool m_control_interpolation;

    const size_t m_k_max;

    bool m_last_step_rejected;
    bool m_first;

    time_type m_t;
    time_type m_dt;
    time_type m_dt_last;
    time_type m_t_last;

    size_t m_current_k_opt;
    size_t m_k_final;

    algebra_type m_algebra;

    resizer_type m_resizer;

    wrapped_state_type m_x1 , m_x2;
    wrapped_deriv_type m_dxdt1 , m_dxdt2;
    state_type *m_current_state , *m_old_state;
    deriv_type *m_current_deriv , *m_old_deriv;

    wrapped_state_type m_err;

    value_vector m_error; // errors of repeated midpoint steps and extrapolations
    int_vector m_interval_sequence; // stores the successive interval counts
    value_matrix m_coeff;
    int_vector m_cost; // costs for interval count

    state_vector_type m_table; // sequence of states for extrapolation

    //for dense output:
    state_vector_type m_mp_states; // sequence of approximations of x at distance center
    deriv_table_type m_derivs; // table of function values
    deriv_table_type m_diffs; // table of function values

    //wrapped_state_type m_a1 , m_a2 , m_a3 , m_a4;

    const time_type STEPFAC1 , STEPFAC2 , STEPFAC3 , STEPFAC4 , KFAC1 , KFAC2;
};

}
}
}

#endif // BOOST_NUMERIC_ODEINT_STEPPER_BULIRSCH_STOER_HPP_INCLUDED
