/*
 [auto_generated]
 boost/numeric/odeint.hpp

 [begin_description]
 Forward include for odeint. Includes nearly everything.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef OMPLEXT_BOOST_NUMERIC_ODEINT_HPP_INCLUDED
#define OMPLEXT_BOOST_NUMERIC_ODEINT_HPP_INCLUDED

#include <omplext_odeint/boost/numeric/odeint/version.hpp>
#include <omplext_odeint/boost/numeric/odeint/config.hpp>

// start with ublas wrapper because we need its specializations before including state_wrapper.hpp
#include <omplext_odeint/boost/numeric/odeint/util/ublas_wrapper.hpp>

#include <omplext_odeint/boost/numeric/odeint/stepper/euler.hpp>
#include <omplext_odeint/boost/numeric/odeint/stepper/runge_kutta4_classic.hpp>
#include <omplext_odeint/boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <omplext_odeint/boost/numeric/odeint/stepper/runge_kutta_cash_karp54.hpp>
#include <omplext_odeint/boost/numeric/odeint/stepper/runge_kutta_cash_karp54_classic.hpp>
#include <omplext_odeint/boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <omplext_odeint/boost/numeric/odeint/stepper/runge_kutta_fehlberg78.hpp>

#include <omplext_odeint/boost/numeric/odeint/stepper/controlled_runge_kutta.hpp>

#include <omplext_odeint/boost/numeric/odeint/stepper/dense_output_runge_kutta.hpp>

#include <omplext_odeint/boost/numeric/odeint/stepper/bulirsch_stoer.hpp>

#ifndef __CUDACC__
/* Bulirsch Stoer with Dense Output does not compile with nvcc
 * because of the binomial library used there which relies on unsupported SSE functions
 */
#include <omplext_odeint/boost/numeric/odeint/stepper/bulirsch_stoer_dense_out.hpp>
#endif

#include <omplext_odeint/boost/numeric/odeint/stepper/symplectic_euler.hpp>
#include <omplext_odeint/boost/numeric/odeint/stepper/symplectic_rkn_sb3a_mclachlan.hpp>

#include <omplext_odeint/boost/numeric/odeint/stepper/adams_bashforth_moulton.hpp>

#include <omplext_odeint/boost/numeric/odeint/stepper/implicit_euler.hpp>
#include <omplext_odeint/boost/numeric/odeint/stepper/rosenbrock4.hpp>
#include <omplext_odeint/boost/numeric/odeint/stepper/rosenbrock4_controller.hpp>
#include <omplext_odeint/boost/numeric/odeint/stepper/rosenbrock4_dense_output.hpp>

/*
 * Including this algebra slows down the compilation time
 */
// #include <omplext_odeint/boost/numeric/odeint/algebra/fusion_algebra.hpp>
#include <omplext_odeint/boost/numeric/odeint/algebra/vector_space_algebra.hpp>

#include <omplext_odeint/boost/numeric/odeint/integrate/integrate.hpp>
#include <omplext_odeint/boost/numeric/odeint/integrate/integrate_adaptive.hpp>
#include <omplext_odeint/boost/numeric/odeint/integrate/integrate_const.hpp>
#include <omplext_odeint/boost/numeric/odeint/integrate/integrate_n_steps.hpp>
#include <omplext_odeint/boost/numeric/odeint/integrate/integrate_times.hpp>

#include <omplext_odeint/boost/numeric/odeint/integrate/observer_collection.hpp>

#include <omplext_odeint/boost/numeric/odeint/stepper/generation.hpp>


#endif // BOOST_NUMERIC_ODEINT_HPP_INCLUDED
