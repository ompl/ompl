/*
 [auto_generated]
 boost/numeric/odeint/stepper/old_controlled_stepper_bs.hpp
 
 [begin_description]
 Implementation of Burlish-Stoer from odeint-v1.
 [end_description]
 
 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky
 
 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
*/


#ifndef OMPLEXT_BOOST_NUMERIC_ODEINT_STEPPER_OLD_CONTROLLED_STEPPER_BS_HPP_INCLUDED
#define OMPLEXT_BOOST_NUMERIC_ODEINT_STEPPER_OLD_CONTROLLED_STEPPER_BS_HPP_INCLUDED


#include <limits>
#include <vector>

#include <omplext_odeint/boost/numeric/odeint/stepper_midpoint.hpp>
#include <omplext_odeint/boost/numeric/odeint/error_checker_standard.hpp>
#include <omplext_odeint/boost/numeric/odeint/container_traits.hpp>

#include <omplext_odeint/boost/numeric/odeint/detail/iterator_algebra.hpp>

namespace boost {
namespace numeric {
namespace omplext_odeint {


    template<
        class Container ,
        class Time = double ,
        class Traits = container_traits< Container >
        >
    class controlled_stepper_bs
    {
        // provide basic typedefs
    public:

        typedef unsigned short order_type;
        typedef Time time_type;
        typedef Traits traits_type;
        typedef typename traits_type::container_type container_type;
        typedef container_type state_type;
        typedef typename traits_type::value_type value_type;
        typedef typename traits_type::iterator iterator;
        typedef typename traits_type::const_iterator const_iterator;


        
        // private memebers
    private:

        stepper_midpoint< state_type, time_type, traits_type > m_stepper_mp;
        error_checker_standard< state_type, time_type , traits_type > m_error_checker;
        
        const unsigned short m_k_max;

        const time_type m_safety1;
        const time_type m_safety2;
        const time_type m_max_dt_factor;
        const time_type m_min_step_scale;
        const time_type m_max_step_scale;

        bool m_continuous_calls;
        bool m_decreased_step_during_last_call;

        time_type m_dt_last;
        time_type m_t_last;
        time_type m_current_eps;

        unsigned short m_current_k_max;
        unsigned short m_current_k_opt;

        state_type m_x0;
        state_type m_xerr;
        state_type m_x_mp;
        state_type m_x_scale;
        state_type m_dxdt;

        typedef std::vector< time_type > value_vector;
        typedef std::vector< std::vector< time_type > > value_matrix;
        typedef std::vector< unsigned short > us_vector;

        value_vector m_error; // errors of repeated midpoint steps and extrapolations
        value_vector m_a; // stores the work (number of f calls) required for the orders
        value_matrix m_alpha; // stores convergence factor for stepsize adjustment
        us_vector m_interval_sequence;

        value_vector m_times;
        std::vector< state_type > m_d;
        state_type m_c;
        
        // public functions
    public:
        
        // constructor
        controlled_stepper_bs( 
                time_type abs_err, time_type rel_err, 
                time_type factor_x, time_type factor_dxdt )
	    : m_error_checker( abs_err, rel_err, factor_x, factor_dxdt ),
              m_k_max(8),
              m_safety1(0.25), m_safety2(0.7),
              m_max_dt_factor( 0.1 ), m_min_step_scale(5E-5), m_max_step_scale(0.7),
              m_continuous_calls(false), m_decreased_step_during_last_call( false ),
              m_dt_last( 1.0E30),
              m_current_eps( -1.0 )
        {
            m_error.resize(m_k_max);
            m_a.resize(m_k_max+1);
            m_alpha.resize(m_k_max); // k_max * k_max matrix
            typename value_matrix::iterator it = m_alpha.begin();
            while( it != m_alpha.end() )
                (*it++).resize(m_k_max);
            m_interval_sequence.resize(m_k_max+1);
            for( unsigned short i = 1; i <= m_k_max+1; i++ )
                m_interval_sequence[i-1] = (2*i);

            m_times.resize(m_k_max);
            m_d.resize(m_k_max);
        }

        //constructor
        controlled_stepper_bs( 
                const state_type &x,
                time_type abs_err, time_type rel_err, 
                time_type factor_x, time_type factor_dxdt )
        {
            adjust_size(x);
            this(abs_err , rel_err , factor_x , factor_dxdt);
        }



        void adjust_size( const state_type &x )
        {
            traits_type::adjust_size(x, m_xerr);
            traits_type::adjust_size(x, m_x_mp);
            traits_type::adjust_size(x, m_x_scale);
            traits_type::adjust_size(x, m_dxdt);
            m_stepper_mp.adjust_size( x );
        }


        template< class DynamicalSystem >
        controlled_step_result try_step(
                DynamicalSystem &system ,
                state_type &x ,
                state_type &dxdt ,
                time_type &t ,
                time_type &dt )
        {

            // get error scale
            m_error_checker.fill_scale(x, dxdt, dt, m_x_scale);

            //std::clog << " x scale: " << m_x_scale[0] << '\t' << m_x_scale[1] << std::endl;

            m_x0 = x; // save starting state
            time_type max_eps = m_error_checker.get_epsilon();
            if( m_current_eps != max_eps ) 
            { // error changed -> recalculate tableau
                initialize_step_adjust_tableau( max_eps );
                m_current_eps = max_eps;
                m_dt_last = m_t_last = std::numeric_limits< value_type >::max(); // unrealistic
            }
            // if t and dt are exactly the parameters from last step we are called continuously
            bool continuous_call = ((t == m_t_last) || (dt == m_dt_last ));
            if( !continuous_call ) 
                m_current_k_opt = m_current_k_max;

            bool converged = false;
            value_type step_scale = 1.0;
            unsigned short k_conv = 0;
            
            for( unsigned short k=0; k<=m_current_k_max; k++ )
            {  // loop through interval numbers
                unsigned short step_number = m_interval_sequence[k];
                //out-of-place midpoint step
                m_stepper_mp.set_step_number(step_number);
                m_stepper_mp.midpoint_step(system, m_x0, dxdt, t, dt, m_x_mp); 
                //std::clog << "x_mp: " << k << '\t' << m_x_mp[0] << '\t' << m_x_mp[1] << std::endl;
                time_type t_est = (dt/step_number)*(dt/step_number);
                extrapolate(k, t_est, m_x_mp, x, m_xerr);
                //std::clog << "Error: " << k << '\t' << m_xerr[0] << '\t' << m_xerr[1] << std::endl;
                if( k != 0 ) 
                {
                    time_type max_err = m_error_checker.get_max_error_ratio(m_xerr, m_x_scale);
                    m_error[k-1] = std::pow( max_err/m_safety1, 1.0/(2*k+1) );
                    if( (k >= m_current_k_opt-1) || !continuous_call )
                    { //we're in the order window where convergence is expected
                        if( max_err < 1.0 )
                        {
                            k_conv = k;
                            converged = true;
                            //std::clog << "Converged at: " << k << '\t' << max_err << std::endl;
                            break;
                        } else {
                            converged = false;
                            if( (k == m_current_k_max) || (k == m_current_k_opt+1) )
                            {
                                step_scale = m_safety2/m_error[k-1];
                                break;
                            } else if( (k == m_current_k_opt) && 
                                       (m_alpha[m_current_k_opt-1][m_current_k_opt] < m_error[k-1] ) )
                            {
                                step_scale = static_cast<value_type>(1.0)/m_error[k-1];
                                break;
                            } else if( (m_current_k_opt == m_current_k_max) &&
                                       (m_alpha[k-1][m_current_k_max-1] < m_error[k-1]) )
                            {
                                step_scale = m_alpha[k-1][m_current_k_max-1]*m_safety2/m_error[k-1];
                                //std::clog << " Case 3 " << m_alpha[k-1][m_current_k_max-1] << '\t' << m_error[k-1] << '\t' << step_scale << std::endl;
                                break;
                            } else if( m_alpha[k-1][m_current_k_opt] < m_error[k-1] )
                            {
                                step_scale = m_alpha[k-1][m_current_k_opt-1]/m_error[k-1];
                                break;
                            }
                        }
                    }
                }
            }
            if( !converged ) { // dt was too large - no convergence up to order k_max

                // step_scale is always > 1 
                step_scale = std::max(step_scale, m_min_step_scale); // at least m_min ...
                step_scale = std::min(step_scale, m_max_step_scale); // at most m_max ...
                dt *= step_scale;
                m_dt_last = dt;
                m_t_last = t;
                m_decreased_step_during_last_call = true;
                x = m_x0; // copy the state back
                return step_size_decreased;

            } else { //converged
                
                t += dt; // step accomplished

                time_type work_min = std::numeric_limits< time_type >::max();
                for( unsigned short k=0; k < k_conv ; k++ )
                { // compute optimal convergence order and corresponding stepsize
                    time_type factor = std::max(m_error[k], m_max_dt_factor);
                    time_type work = factor * m_a[k+1];
                    if( work < work_min )
                    {
                        step_scale = factor;
                        work_min = work;
                        m_current_k_opt = k+1;
                    }
                }
                m_dt_last = dt/step_scale;

                //std::clog << "Internal: " << dt << '\t' << k_conv-1 << '\t' << m_current_k_opt << '\t' << m_current_k_max << '\t' << m_decreased_step_during_last_call << std::endl;

                if( (m_current_k_opt >= k_conv) && 
                    (m_current_k_opt != m_current_k_max) && 
                    !m_decreased_step_during_last_call )
                { // check possible order increase, if step was not decreased before
                    time_type factor = std::max(
                            step_scale/m_alpha[m_current_k_opt-1][m_current_k_opt],
                            m_max_dt_factor );
                    if( m_a[m_current_k_opt+1]*factor <= work_min )
                    {
                        m_dt_last = dt/factor;
                        m_current_k_opt++;
                    }
                }
                dt = m_dt_last;
                m_t_last = t;
                m_decreased_step_during_last_call = false;
                //std::clog << "Internal: " << dt << '\t' << m_current_k_opt << std::endl;
                return step_size_increased;
            }
        }
        
        template< class System >
        controlled_step_result try_step( 
                System &system,
                state_type &x,
                time_type &t,
                time_type &dt )
        {
            system(x, m_dxdt, t);
            return try_step(system, x, m_dxdt, t, dt );
        }


    private:   // private functions

        void initialize_step_adjust_tableau( time_type eps )
        {
            m_a[0] = m_interval_sequence[0]+1;
            for( unsigned short k=0; k<m_k_max; k++ )
                m_a[k+1] = m_a[k] + m_interval_sequence[k+1];
            for( unsigned short i=1; i<m_k_max; i++ )
            {
                for( unsigned short k=0; k<i; k++ )
                {
                    m_alpha[k][i] = std::pow( 
                            m_safety1*eps, 
                            (m_a[k+1]-m_a[i+1])/
                            ((m_a[i+1]-m_a[0]+1.0)*(2*k+3)) 
                                              );
                }
            }
            m_current_k_opt = m_k_max-1;
            for( unsigned short k=1; k<m_k_max-1; k++ )
            {
                if( m_a[k+1] > m_a[k]*m_alpha[k-1][k] )
                {
                    m_current_k_opt = k;
                    break;
                }
            }
            m_current_k_max = m_current_k_opt;
        }

        void extrapolate(
                unsigned short k_est, 
                time_type t_est, 
                state_type &x_est, 
                state_type &x, 
                state_type &x_err )
        {
            //traits_type::adjust_size(x, m_c);
            //std::vector< state_type > m_d_iter = m_d.begin();
            //while( m_d_iter != m_d.end() )
            //    traits_type::adjust_size(x, (*m_d_iter++));
            
            m_times[k_est] = t_est;
            x_err = x = x_est;

            const iterator x_end = traits_type::end(x);

            if( k_est == 0 )
            {
                m_d[0] = x_est;
            }
            else 
            {
               m_c = x_est;
               for( unsigned short k=0; k<k_est; k++ )
               {
                   value_type delta = static_cast<value_type>(1.0) / 
                       (m_times[k_est-k-1]-static_cast<value_type>(t_est));
                   value_type val1 = static_cast<value_type>(t_est)*delta;
                   value_type val2 = m_times[k_est-k-1]*delta;

                   //std::clog << " values: " << delta << '\t' << val1 << '\t' << val2 << std::endl; 

                   iterator x_iter = traits_type::begin(x);
                   iterator x_err_iter = traits_type::begin(x_err);
                   iterator d_k_iter = m_d[k].begin();
                   iterator c_iter = m_c.begin();
                   while( x_iter != x_end )
                   {
                       //std::clog << " extrapolate: " << '\t' << *x_iter << '\t' << *x_err_iter << '\t' << *d_k_iter << std::endl;
                       value_type q = *d_k_iter;
                       *d_k_iter++ = *x_err_iter;
                       delta = *c_iter - q;
                       *x_err_iter = val1 * delta;
                       *c_iter++ = val2 * delta;
                       *x_iter++ += *x_err_iter++;
                   }
                   //std::clog << " extrapolate: " << '\t' << x[1] << '\t' << x_err[1] << '\t' << m_d[k][1] << std::endl;
                   m_d[k_est] = x_err;
               }
            }
        }

    };
}
}
}

#endif // BOOST_NUMERIC_ODEINT_STEPPER_OLD_CONTROLLED_STEPPER_BS_HPP_INCLUDED
