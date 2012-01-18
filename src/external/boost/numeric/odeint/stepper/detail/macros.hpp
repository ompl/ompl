/*
 [auto_generated]
 boost/numeric/odeint/stepper/detail/macros.hpp

 [begin_description]
 Macros to simplify the implementation of specific steppers.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef BOOST_NUMERIC_ODEINT_STEPPER_DETAIL_MACROS_HPP_INCLUDED
#define BOOST_NUMERIC_ODEINT_STEPPER_DETAIL_MACROS_HPP_INCLUDED



#define BOOST_ODEINT_EXPLICIT_STEPPERS_TYPEDEFS( STEPPER , ORDER ) \
        typedef explicit_stepper_base< \
                STEPPER< State , Value , Deriv ,Time , Algebra , Operations , Resizer > , \
                ORDER , State , Value , Deriv , Time , Algebra , Operations , Resizer > stepper_base_type; \
                typedef typename stepper_base_type::state_type state_type; \
                typedef typename stepper_base_type::value_type value_type; \
                typedef typename stepper_base_type::deriv_type deriv_type; \
                typedef typename stepper_base_type::time_type time_type; \
                typedef typename stepper_base_type::algebra_type algebra_type; \
                typedef typename stepper_base_type::operations_type operations_type; \
                typedef typename stepper_base_type::resizer_type resizer_type; \
                typedef typename stepper_base_type::wrapped_state_type wrapped_state_type; \
                typedef typename stepper_base_type::wrapped_deriv_type wrapped_deriv_type


#define BOOST_ODEINT_EXPLICIT_ERROR_STEPPERS_TYPEDEFS( STEPPER , STEPPER_ORDER , ERROR_ORDER ) \
        typedef explicit_error_stepper_base< \
                STEPPER< State , Value , Deriv , Time , Algebra , Operations , AdjustSizePolicy > , \
                STEPPER_ORDER , ERROR_ORDER , State , Value , Deriv , Time , Algebra , Operations , AdjustSizePolicy > stepper_base_type; \
                typedef typename stepper_base_type::state_type state_type; \
                typedef typename stepper_base_type::value_type value_type; \
                typedef typename stepper_base_type::deriv_type deriv_type; \
                typedef typename stepper_base_type::time_type time_type; \
                typedef typename stepper_base_type::algebra_type algebra_type; \
                typedef typename stepper_base_type::operations_type operations_type; \
                typedef typename stepper_base_type::adjust_size_policy adjust_size_policy; \
                typedef typename stepper_base_type::wrapped_state_type wrapped_state_type; \
                typedef typename stepper_base_type::wrapped_deriv_type wrapped_deriv_type


#define BOOST_ODEINT_EXPLICIT_STEPPERS_AND_ERROR_STEPPERS_TYPEDEFS( STEPPER , ORDER , STEPPER_ORDER , ERROR_ORDER ) \
        typedef explicit_stepper_and_error_stepper_base< \
                STEPPER< State , Value , Deriv , Time , Algebra , Operations , Resizer > , \
                ORDER , STEPPER_ORDER , ERROR_ORDER , State , Value , Deriv , Time , Algebra , Operations , Resizer > stepper_base_type; \
                typedef typename stepper_base_type::state_type state_type; \
                typedef typename stepper_base_type::value_type value_type; \
                typedef typename stepper_base_type::deriv_type deriv_type; \
                typedef typename stepper_base_type::time_type time_type; \
                typedef typename stepper_base_type::algebra_type algebra_type; \
                typedef typename stepper_base_type::operations_type operations_type; \
                typedef typename stepper_base_type::resizer_type resizer_type; \
                typedef typename stepper_base_type::wrapped_state_type wrapped_state_type; \
                typedef typename stepper_base_type::wrapped_deriv_type wrapped_deriv_type

#define BOOST_ODEINT_EXPLICIT_STEPPERS_AND_ERROR_STEPPERS_FSAL_TYPEDEFS( STEPPER , ORDER , STEPPER_ORDER , ERROR_ORDER ) \
        typedef explicit_stepper_and_error_stepper_fsal_base< \
                STEPPER< State , Value , Deriv , Time , Algebra , Operations , Resizer > , \
                ORDER , STEPPER_ORDER , ERROR_ORDER , State , Value , Deriv , Time , Algebra , Operations , Resizer > stepper_base_type; \
                typedef typename stepper_base_type::state_type state_type; \
                typedef typename stepper_base_type::value_type value_type; \
                typedef typename stepper_base_type::deriv_type deriv_type; \
                typedef typename stepper_base_type::time_type time_type; \
                typedef typename stepper_base_type::algebra_type algebra_type; \
                typedef typename stepper_base_type::operations_type operations_type; \
                typedef typename stepper_base_type::resizer_type resizer_type; \
                typedef typename stepper_base_type::wrapped_state_type wrapped_state_type; \
                typedef typename stepper_base_type::wrapped_deriv_type wrapped_deriv_type


#define BOOST_ODEINT_SYMPLECTIC_NYSTROEM_STEPPER_TYPEDEFS( STEPPER , STAGES ) \
        typedef symplectic_nystroem_stepper_base< \
                STAGES , \
                STEPPER< Coor , Momentum , Value , CoorDeriv , MomentumDeriv , Time , Algebra , Operations , Resizer > , \
                Coor , Momentum , Value , CoorDeriv , MomentumDeriv , Time , Algebra , Operations , Resizer > stepper_base_type; \
                typedef typename stepper_base_type::coor_type coor_type; \
                typedef typename stepper_base_type::momentum_type momentum_type; \
                typedef typename stepper_base_type::state_type state_type; \
                typedef typename stepper_base_type::value_type value_type; \
                typedef typename stepper_base_type::coor_deriv_type coor_deriv_type; \
                typedef typename stepper_base_type::wrapped_coor_deriv_type wrapped_coor_deriv_type; \
                typedef typename stepper_base_type::momentum_deriv_type momentum_deriv_type; \
                typedef typename stepper_base_type::wrapped_momentum_deriv_type wrapped_momentum_deriv_type; \
                typedef typename stepper_base_type::deriv_type deriv_type; \
                typedef typename stepper_base_type::time_type time_type; \
                typedef typename stepper_base_type::algebra_type algebra_type; \
                typedef typename stepper_base_type::operations_type operations_type; \
                typedef typename stepper_base_type::resizer_type resizer_type; \
                typedef typename stepper_base_type::coef_type coef_type


#endif // BOOST_NUMERIC_ODEINT_STEPPER_DETAIL_MACROS_HPP_INCLUDED
