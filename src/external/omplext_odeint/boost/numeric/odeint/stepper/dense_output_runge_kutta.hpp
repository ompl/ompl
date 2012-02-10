/*
 [auto_generated]
 boost/numeric/odeint/stepper/dense_output_runge_kutta.hpp

 [begin_description]
 Implementation of the Dense-output stepper for all steppers. Note, that this class does
 not computes the result but serves as an interface.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef OMPLEXT_BOOST_NUMERIC_ODEINT_STEPPER_DENSE_OUTPUT_RUNGE_KUTTA_HPP_INCLUDED
#define OMPLEXT_BOOST_NUMERIC_ODEINT_STEPPER_DENSE_OUTPUT_RUNGE_KUTTA_HPP_INCLUDED


#include <utility>
#include <stdexcept>

#include <boost/ref.hpp>
#include <boost/bind.hpp>

#include <omplext_odeint/boost/numeric/odeint/util/copy.hpp>

#include <omplext_odeint/boost/numeric/odeint/util/state_wrapper.hpp>
#include <omplext_odeint/boost/numeric/odeint/util/resizer.hpp>

#include <omplext_odeint/boost/numeric/odeint/stepper/controlled_step_result.hpp>
#include <omplext_odeint/boost/numeric/odeint/stepper/stepper_categories.hpp>

namespace boost {
namespace numeric {
namespace omplext_odeint {

template
<
class Stepper ,
class StepperCategory = typename Stepper::stepper_category
>
class dense_output_runge_kutta;


template
<
class Stepper
>
class dense_output_runge_kutta< Stepper , stepper_tag >
{

private:

    void copy_pointers( const dense_output_runge_kutta &dense_output )
    {
        if( dense_output.m_current_state == (&dense_output.m_x1.m_v ) )
        {
            m_current_state = &m_x1.m_v;
            m_old_state = &m_x2.m_v;
        }
        else
        {
            m_current_state = &m_x2.m_v;
            m_old_state = &m_x1.m_v;
        }
    }

public:

    /*
     * We do not need all typedefs.
     */
    typedef Stepper stepper_type;
    typedef typename stepper_type::state_type state_type;
    typedef typename stepper_type::wrapped_state_type wrapped_state_type;
    typedef typename stepper_type::value_type value_type;
    typedef typename stepper_type::deriv_type deriv_type;
    typedef typename stepper_type::wrapped_deriv_type wrapped_deriv_type;
    typedef typename stepper_type::time_type time_type;
    typedef typename stepper_type::algebra_type algebra_type;
    typedef typename stepper_type::operations_type operations_type;
    typedef typename stepper_type::resizer_type resizer_type;
    typedef dense_output_stepper_tag stepper_category;
    typedef dense_output_runge_kutta< Stepper > dense_output_stepper_type;



    dense_output_runge_kutta( const stepper_type &stepper = stepper_type() )
    : m_stepper( stepper ) ,
      m_current_state( &m_x1.m_v ) , m_old_state( &m_x2.m_v )
    { }


    dense_output_runge_kutta( const dense_output_runge_kutta &dense_output )
    : m_stepper( dense_output.m_stepper ) , m_x1( dense_output.m_x1 ) , m_x2( dense_output.m_x2 ) ,
      m_current_state( &m_x1.m_v ) , m_old_state( &m_x2.m_v ) ,
      m_t( dense_output.m_t ) , m_t_old( dense_output.m_t_old ) , m_dt( dense_output.m_dt )
    {
        copy_pointers( dense_output );
    }

    dense_output_runge_kutta& operator=( const dense_output_runge_kutta &dense_output )
    {
        m_stepper = dense_output.m_stepper;
        m_x1 = dense_output.m_x1;
        m_x2 = dense_output.m_x2;
        m_t = dense_output.m_t;
        m_t_old = dense_output.m_t_old;
        m_dt = dense_output.m_dt;
        copy_pointers( dense_output );
        return *this;
    }

    template< class StateType >
    void initialize( const StateType &x0 , const time_type &t0 , const time_type &dt0 )
    {
        m_resizer.adjust_size( x0 , boost::bind( &dense_output_stepper_type::template resize_impl< StateType > , boost::ref( *this ) , _1 ) );
        boost::numeric::omplext_odeint::copy( x0 , *m_current_state );
        m_t = t0;
        m_dt = dt0;
    }

    template< class System >
    std::pair< time_type , time_type > do_step( System system )
    {
        m_stepper.do_step( system , *m_current_state , m_t , *m_old_state , m_dt );
        m_t_old = m_t;
        m_t += m_dt;
        std::swap( m_current_state , m_old_state );
        return std::make_pair( m_t_old , m_dt );
    }

    /*
     * The next two overloads are needed to solve the forwarding problem
     */
    template< class StateOut >
    void calc_state( const time_type &t , StateOut &x )
    {
        m_stepper.calc_state( x , t , *m_old_state , m_t_old , *m_current_state , m_t );
    }

    template< class StateOut >
    void calc_state( const time_type &t , const StateOut &x )
    {
        m_stepper.calc_state( x , t , *m_old_state , m_t_old , *m_current_state , m_t );
    }

    template< class StateType >
    void adjust_size( const StateType &x )
    {
        resize_impl( x );
        m_stepper.stepper().resize( x );
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
        return m_t_old;
    }


private:

    template< class StateIn >
    bool resize_impl( const StateIn &x )
    {
        bool resized = false;
        resized |= adjust_size_by_resizeability( m_x1 , x , typename wrapped_state_type::is_resizeable() );
        resized |= adjust_size_by_resizeability( m_x2 , x , typename wrapped_state_type::is_resizeable() );
        return resized;
    }


    stepper_type m_stepper;
    resizer_type m_resizer;
    wrapped_state_type m_x1 , m_x2;
    state_type *m_current_state , *m_old_state;
    time_type m_t , m_t_old , m_dt;

};






template
<
class Stepper
>
class dense_output_runge_kutta< Stepper , explicit_controlled_stepper_fsal_tag >
{
private:

    void copy_pointers( const dense_output_runge_kutta &dense_output )
    {
        if( dense_output.m_current_state == (&dense_output.m_x1.m_v ) )
        {
            m_current_state = &m_x1.m_v;
            m_old_state = &m_x2.m_v;
        }
        else
        {
            m_current_state = &m_x2.m_v;
            m_old_state = &m_x1.m_v;
        }
        if( dense_output.m_current_deriv == ( &dense_output.m_dxdt1.m_v ) )
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

    /*
     * We do not need all typedefs.
     */
    typedef Stepper controlled_stepper_type;

    typedef typename controlled_stepper_type::stepper_type stepper_type;
    typedef typename stepper_type::state_type state_type;
    typedef typename stepper_type::wrapped_state_type wrapped_state_type;
    typedef typename stepper_type::value_type value_type;
    typedef typename stepper_type::deriv_type deriv_type;
    typedef typename stepper_type::wrapped_deriv_type wrapped_deriv_type;
    typedef typename stepper_type::time_type time_type;
    typedef typename stepper_type::algebra_type algebra_type;
    typedef typename stepper_type::operations_type operations_type;
    typedef typename stepper_type::resizer_type resizer_type;
    typedef dense_output_stepper_tag stepper_category;
    typedef dense_output_runge_kutta< Stepper > dense_output_stepper_type;

    dense_output_runge_kutta( const controlled_stepper_type &stepper = controlled_stepper_type() )
    : m_stepper( stepper ) ,
      m_current_state( &m_x1.m_v ) , m_old_state( &m_x2.m_v ) ,
      m_current_deriv( &m_dxdt1.m_v ) , m_old_deriv( &m_dxdt2.m_v ) ,
      m_is_deriv_initialized( false )
    { }

    dense_output_runge_kutta( const dense_output_runge_kutta &dense_output )
    : m_stepper( dense_output.m_stepper ) ,
      m_x1( dense_output.m_x1 ) , m_x2( dense_output.m_x2 ) ,
      m_dxdt1( dense_output.m_dxdt1 ) , m_dxdt2( dense_output.m_dxdt2 ) ,
      m_current_state( &m_x1.m_v ) , m_old_state( &m_x2.m_v ) ,
      m_current_deriv( &m_dxdt1.m_v ) , m_old_deriv( &m_dxdt2.m_v ) ,
      m_t( dense_output.m_t ) , m_t_old( dense_output.m_t_old ) , m_dt( dense_output.m_dt ) ,
      m_is_deriv_initialized( dense_output.m_is_deriv_initialized )
    {
        copy_pointers( dense_output );
    }

    dense_output_runge_kutta& operator=( const dense_output_runge_kutta &dense_output )
    {
        m_stepper = dense_output.m_stepper;
        m_x1 = dense_output.m_x1;
        m_x2 = dense_output.m_x2;
        m_dxdt1 = dense_output.m_dxdt1;
        m_dxdt2 = dense_output.m_dxdt2;
        m_current_state = &m_x1.m_v;
        m_old_state = &m_x2.m_v;
        m_current_deriv = &m_dxdt1.m_v;
        m_old_deriv = &m_dxdt2.m_v;
        copy_pointers( dense_output );
        return *this;
    }

    template< class StateType >
    void initialize( const StateType &x0 , const time_type &t0 , const time_type &dt0 )
    {
        m_resizer.adjust_size( x0 , boost::bind( &dense_output_stepper_type::template resize< StateType > , boost::ref( *this ) , _1 ) );
        boost::numeric::omplext_odeint::copy( x0 , *m_current_state );
        m_t = t0;
        m_dt = dt0;
        m_is_deriv_initialized = false;
    }

    template< class System >
    std::pair< time_type , time_type > do_step( System system )
    {
        const size_t max_count = 1000;

        if( !m_is_deriv_initialized )
        {
            typename boost::unwrap_reference< System >::type &sys = system;
            sys( *m_current_state , *m_current_deriv , m_t );
            m_is_deriv_initialized = true;
        }

        controlled_step_result res = fail;
        m_t_old = m_t;
        size_t count = 0;
        do
        {
            res = m_stepper.try_step( system , *m_current_state , *m_current_deriv , m_t , *m_old_state , *m_old_deriv , m_dt );
            if( count++ == max_count )
                throw std::overflow_error( "dense_output_controlled_explicit : too much iterations!");
        }
        while( res == fail );
        std::swap( m_current_state , m_old_state );
        std::swap( m_current_deriv , m_old_deriv );
        return std::make_pair( m_t_old , m_t );
    }


    /*
     * The two overloads are needed in order to solve the forwarding problem.
     */
    template< class StateOut >
    void calc_state( const time_type &t , StateOut &x )
    {
        m_stepper.stepper().calc_state( t , x , *m_old_state , *m_old_deriv , m_t_old , *m_current_state , *m_current_deriv , m_t );
    }

    template< class StateOut >
    void calc_state( const time_type &t , const StateOut &x )
    {
        m_stepper.stepper().calc_state( t , x , *m_old_state , *m_old_deriv , m_t_old , *m_current_state , *m_current_deriv , m_t );
    }


    template< class StateIn >
    bool resize( const StateIn &x )
    {
        bool resized = false;
        resized |= adjust_size_by_resizeability( m_x1 , x , typename wrapped_state_type::is_resizeable() );
        resized |= adjust_size_by_resizeability( m_x2 , x , typename wrapped_state_type::is_resizeable() );
        resized |= adjust_size_by_resizeability( m_dxdt1 , x , typename wrapped_deriv_type::is_resizeable() );
        resized |= adjust_size_by_resizeability( m_dxdt2 , x , typename wrapped_deriv_type::is_resizeable() );
        return resized;
    }


    template< class StateType >
    void adjust_size( const StateType &x )
    {
        resize( x );
        m_stepper.stepper().resize( x );
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
        return m_t_old;
    }

    const time_type& current_time_step( void ) const
    {
        return m_dt;
    }


private:

    controlled_stepper_type m_stepper;
    resizer_type m_resizer;
    wrapped_state_type m_x1 , m_x2;
    wrapped_deriv_type m_dxdt1 , m_dxdt2;
    state_type *m_current_state , *m_old_state;
    deriv_type *m_current_deriv , *m_old_deriv;
    time_type m_t , m_t_old , m_dt;
    bool m_is_deriv_initialized;

};

} // namespace omplext_odeint
} // namespace numeric
} // namespace boost



#endif // BOOST_NUMERIC_ODEINT_STEPPER_DENSE_OUTPUT_RUNGE_KUTTA_HPP_INCLUDED
