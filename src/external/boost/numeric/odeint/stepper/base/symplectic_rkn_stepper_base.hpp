/*
 [auto_generated]
 boost/numeric/odeint/stepper/base/symplectic_rkn_stepper_base.hpp

 [begin_description]
 Base class for symplectic Runge-Kutta-Nystrom steppers.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef BOOST_NUMERIC_ODEINT_STEPPER_BASE_SYMPLECTIC_RKN_STEPPER_BASE_HPP_INCLUDED
#define BOOST_NUMERIC_ODEINT_STEPPER_BASE_SYMPLECTIC_RKN_STEPPER_BASE_HPP_INCLUDED

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>

#include <boost/numeric/odeint/util/copy.hpp>
#include <boost/numeric/odeint/util/is_pair.hpp>

#include <boost/numeric/odeint/util/state_wrapper.hpp>
#include <boost/numeric/odeint/util/resizer.hpp>

#include <boost/numeric/odeint/stepper/stepper_categories.hpp>

#include <boost/numeric/odeint/stepper/base/algebra_stepper_base.hpp>



namespace boost {
namespace numeric {
namespace odeint {


/*
 * Symplectic Runge Kutta Nystroem base
 */
template<
size_t NumOfStages ,
class Stepper ,
class Coor ,
class Momentum ,
class Value ,
class CoorDeriv ,
class MomentumDeriv ,
class Time ,
class Algebra ,
class Operations ,
class Resizer
>
class symplectic_nystroem_stepper_base : public algebra_stepper_base< Algebra , Operations >
{

public:

    typedef algebra_stepper_base< Algebra , Operations > algebra_stepper_base_type;
    typedef typename algebra_stepper_base_type::algebra_type algebra_type;
    typedef typename algebra_stepper_base_type::operations_type operations_type;

    const static size_t num_of_stages = NumOfStages;
    typedef Coor coor_type;
    typedef Momentum momentum_type;
    typedef std::pair< coor_type , momentum_type > state_type;
    typedef CoorDeriv coor_deriv_type;
    typedef state_wrapper< coor_deriv_type> wrapped_coor_deriv_type;
    typedef MomentumDeriv momentum_deriv_type;
    typedef state_wrapper< momentum_deriv_type > wrapped_momentum_deriv_type;
    typedef std::pair< coor_deriv_type , momentum_deriv_type > deriv_type;
    typedef Value value_type;
    typedef Time time_type;
    typedef Resizer resizer_type;
    typedef Stepper stepper_type;
    typedef stepper_tag stepper_category;
    typedef symplectic_nystroem_stepper_base< NumOfStages , Stepper , Coor , Momentum , Value ,
            CoorDeriv , MomentumDeriv , Time , Algebra , Operations , Resizer > internal_stepper_base_type;

    typedef boost::array< value_type , num_of_stages > coef_type;

    symplectic_nystroem_stepper_base( const coef_type &coef_a , const coef_type &coef_b , const algebra_type &algebra = algebra_type() )
    : algebra_stepper_base_type( algebra ) , m_coef_a( coef_a ) , m_coef_b( coef_b )
    { }

    symplectic_nystroem_stepper_base( const symplectic_nystroem_stepper_base &stepper )
    : m_coef_a( stepper.m_coef_a ) , m_coef_b( stepper.m_coef_b ) ,
      m_dqdt_resizer( stepper.m_dqdt_resizer ) , m_dpdt_resizer( stepper.m_dpdt_resizer ) ,
      m_dqdt( stepper.m_dqdt ) , m_dpdt( stepper.m_dpdt )
    { }

    symplectic_nystroem_stepper_base& operator = ( const symplectic_nystroem_stepper_base &stepper )
    {
        m_dqdt_resizer = stepper.m_dqdt_resizer;
        m_dpdt_resizer = stepper.m_dpdt_resizer;
        m_dqdt = stepper.m_dqdt;
        m_dpdt = stepper.m_dpdt;
        return *this;
    }

    /*
     * Version 1 : do_step( system , x , t , dt )
     *
     * This version does not solve the forwarding problem, boost.range can not be used.
     */
    template< class System , class StateInOut >
    void do_step( System system , const StateInOut &state , const time_type &t , const time_type &dt )
    {
        typedef typename boost::unwrap_reference< System >::type system_type;
        do_step_impl( system , state , t , state , dt , typename is_pair< system_type >::type() );
    }

    template< class System , class StateInOut >
    void do_step( System system , StateInOut &state , const time_type &t , const time_type &dt )
    {
        typedef typename boost::unwrap_reference< System >::type system_type;
        do_step_impl( system , state , t , state , dt , typename is_pair< system_type >::type() );
    }


    /*
     * Version 2 : do_step( system , q , p , t , dt );
     *
     * For Convenience
     *
     * The two overloads are needed in order to solve the forwarding problem.
     */
    template< class System , class CoorInOut , class MomentumInOut >
    void do_step( System system , CoorInOut &q , MomentumInOut &p , const time_type &t , const time_type &dt )
    {
        do_step( system , std::make_pair( boost::ref( q ) , boost::ref( p ) ) , t , dt );
    }

    template< class System , class CoorInOut , class MomentumInOut >
    void do_step( System system , const CoorInOut &q , const MomentumInOut &p , const time_type &t , const time_type &dt )
    {
        do_step( system , std::make_pair( boost::ref( q ) , boost::ref( p ) ) , t , dt );
    }





    /*
     * Version 3 : do_step( system , in , t , out , dt )
     *
     * The forwarding problem is not solved in this version
     */
    template< class System , class StateIn , class StateOut >
    void do_step( System system , const StateIn &in , const time_type &t , StateOut &out , const time_type &dt )
    {
        typedef typename boost::unwrap_reference< System >::type system_type;
        do_step_impl( system , in , t , out , dt , typename is_pair< system_type >::type() );
    }


    template< class StateType >
    void adjust_size( const StateType &x )
    {
        resize_dqdt( x );
        resize_dpdt( x );
    }

    const coef_type& coef_a( void ) const { return m_coef_a; }
    const coef_type& coef_b( void ) const { return m_coef_b; }

private:

    // stepper for systems with function for dq/dt = f(p) and dp/dt = -f(q)
    template< class System , class StateIn , class StateOut >
    void do_step_impl( System system , const StateIn &in , const time_type &t , StateOut &out , const time_type &dt , boost::mpl::true_ )
    {
        typedef typename boost::unwrap_reference< System >::type system_type;
        typedef typename boost::unwrap_reference< typename system_type::first_type >::type coor_deriv_func_type;
        typedef typename boost::unwrap_reference< typename system_type::second_type >::type momentum_deriv_func_type;
        system_type &sys = system;
        coor_deriv_func_type &coor_func = sys.first;
        momentum_deriv_func_type &momentum_func = sys.second;

        typedef typename boost::unwrap_reference< StateIn >::type state_in_type;
        typedef typename boost::unwrap_reference< typename state_in_type::first_type >::type coor_in_type;
        typedef typename boost::unwrap_reference< typename state_in_type::second_type >::type momentum_in_type;
        const state_in_type &state_in = in;
        const coor_in_type &coor_in = state_in.first;
        const momentum_in_type &momentum_in = state_in.second;

        typedef typename boost::unwrap_reference< StateOut >::type state_out_type;
        typedef typename boost::unwrap_reference< typename state_out_type::first_type >::type coor_out_type;
        typedef typename boost::unwrap_reference< typename state_out_type::second_type >::type momentum_out_type;
        state_out_type &state_out = out;
        coor_out_type &coor_out = state_out.first;
        momentum_out_type &momentum_out = state_out.second;

        m_dqdt_resizer.adjust_size( coor_in , boost::bind( &internal_stepper_base_type::template resize_dqdt< coor_in_type > , boost::ref( *this ) , _1 ) );
        m_dpdt_resizer.adjust_size( momentum_in , boost::bind( &internal_stepper_base_type::template resize_dpdt< momentum_in_type > , boost::ref( *this ) , _1 ) );

        // ToDo: check sizes?

        for( size_t l=0 ; l<num_of_stages ; ++l )
        {
            if( l == 0 )
            {
                coor_func( momentum_in , m_dqdt.m_v );
                this->m_algebra.for_each3( coor_out , coor_in , m_dqdt.m_v ,
                        typename operations_type::template scale_sum2< value_type , time_type >( 1.0 , m_coef_a[l] * dt ) );
                momentum_func( coor_out , m_dpdt.m_v );
                this->m_algebra.for_each3( momentum_out , momentum_in , m_dpdt.m_v ,
                        typename operations_type::template scale_sum2< value_type , time_type >( 1.0 , m_coef_b[l] * dt ) );
            }
            else
            {
                coor_func( momentum_out , m_dqdt.m_v );
                this->m_algebra.for_each3( coor_out , coor_out , m_dqdt.m_v ,
                        typename operations_type::template scale_sum2< value_type , time_type >( 1.0 , m_coef_a[l] * dt ) );
                momentum_func( coor_out , m_dpdt.m_v );
                this->m_algebra.for_each3( momentum_out , momentum_out , m_dpdt.m_v ,
                        typename operations_type::template scale_sum2< value_type , time_type >( 1.0 , m_coef_b[l] * dt ) );
            }
        }
    }


    // stepper for systems with only function dp /dt = -f(q), dq/dt = p
    template< class System , class StateIn , class StateOut >
    void do_step_impl( System system , const StateIn &in , const time_type &t , StateOut &out , const time_type &dt , boost::mpl::false_ )
    {
        typedef typename boost::unwrap_reference< System >::type momentum_deriv_func_type;
        momentum_deriv_func_type &momentum_func = system;

        typedef typename boost::unwrap_reference< StateIn >::type state_in_type;
        typedef typename boost::unwrap_reference< typename state_in_type::first_type >::type coor_in_type;
        typedef typename boost::unwrap_reference< typename state_in_type::second_type >::type momentum_in_type;
        const state_in_type &state_in = in;
        const coor_in_type &coor_in = state_in.first;
        const momentum_in_type &momentum_in = state_in.second;

        typedef typename boost::unwrap_reference< StateOut >::type state_out_type;
        typedef typename boost::unwrap_reference< typename state_out_type::first_type >::type coor_out_type;
        typedef typename boost::unwrap_reference< typename state_out_type::second_type >::type momentum_out_type;
        state_out_type &state_out = out;
        coor_out_type &coor_out = state_out.first;
        momentum_out_type &momentum_out = state_out.second;


        m_dqdt_resizer.adjust_size( coor_in , boost::bind( &internal_stepper_base_type::template resize_dqdt< coor_in_type > , boost::ref( *this ) , _1 ) );
        m_dpdt_resizer.adjust_size( momentum_in , boost::bind( &internal_stepper_base_type::template resize_dpdt< momentum_in_type > , boost::ref( *this ) , _1 ) );


        // ToDo: check sizes?


        for( size_t l=0 ; l<num_of_stages ; ++l )
        {
            if( l == 0 )
            {
                this->m_algebra.for_each3( coor_out  , coor_in , momentum_in ,
                        typename operations_type::template scale_sum2< value_type , time_type >( 1.0 , m_coef_a[l] * dt ) );
                momentum_func( coor_out , m_dqdt.m_v );
                this->m_algebra.for_each3( momentum_out , momentum_in , m_dqdt.m_v ,
                        typename operations_type::template scale_sum2< value_type , time_type >( 1.0 , m_coef_b[l] * dt ) );
            }
            else
            {
                this->m_algebra.for_each3( coor_out , coor_out , momentum_out ,
                        typename operations_type::template scale_sum2< value_type , time_type >( 1.0 , m_coef_a[l] * dt ) );
                momentum_func( coor_out , m_dqdt.m_v );
                this->m_algebra.for_each3( momentum_out , momentum_out , m_dqdt.m_v ,
                        typename operations_type::template scale_sum2< value_type , time_type >( 1.0 , m_coef_b[l] * dt ) );
            }
        }
    }

    template< class StateIn >
    bool resize_dqdt( const StateIn &x )
    {
        return adjust_size_by_resizeability( m_dqdt , x , typename wrapped_coor_deriv_type::is_resizeable() );
    }

    template< class StateIn >
    bool resize_dpdt( const StateIn &x )
    {
        return adjust_size_by_resizeability( m_dpdt , x , typename wrapped_momentum_deriv_type::is_resizeable() );
    }


    const coef_type m_coef_a;
    const coef_type m_coef_b;

    resizer_type m_dqdt_resizer;
    resizer_type m_dpdt_resizer;
    wrapped_coor_deriv_type m_dqdt;
    wrapped_momentum_deriv_type m_dpdt;

};

} // namespace odeint
} // namespace numeric
} // namespace boost


#endif // BOOST_NUMERIC_ODEINT_STEPPER_BASE_SYMPLECTIC_RKN_STEPPER_BASE_HPP_INCLUDED
