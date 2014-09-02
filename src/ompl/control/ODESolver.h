/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ryan Luna */

#ifndef OMPL_CONTROL_ODESOLVER_
#define OMPL_CONTROL_ODESOLVER_

// Boost.OdeInt needs Boost version >= 1.44
#include <boost/version.hpp>
#if BOOST_VERSION < 104400
#warning Boost version >=1.44 is needed for ODESolver classes
#else

#include "ompl/control/Control.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/StatePropagator.h"
#include "ompl/util/Console.h"
#include "ompl/util/ClassForward.h"

#if BOOST_VERSION >= 105300
#include <boost/numeric/odeint.hpp>
namespace odeint = boost::numeric::odeint;
#else
#include <omplext_odeint/boost/numeric/odeint.hpp>
namespace odeint = boost::numeric::omplext_odeint;
#endif
#include <boost/function.hpp>
#include <cassert>
#include <vector>

namespace ompl
{

    namespace control
    {

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(ODESolver);
        /// @endcond

        /// \class ompl::control::ODESolverPtr
        /// \brief A boost shared pointer wrapper for ompl::control::ODESolver

        /// \brief Abstract base class for an object that can solve ordinary differential
        /// equations (ODE) of the type q' = f(q,u) using numerical integration.  Classes
        /// deriving from this must implement the solve method.  The user must supply
        /// the ODE to solve.
        class ODESolver
        {
        public:
            /// \brief Portable data type for the state values
            typedef std::vector<double> StateType;

            /// \brief Callback function that defines the ODE.  Accepts
            /// the current state, input control, and output state.
            typedef boost::function<void(const StateType &, const Control*, StateType &)> ODE;

            /// \brief Callback function to perform an event at the end of numerical
            /// integration.  This functionality is optional.
            typedef boost::function<void(const base::State *state, const Control *control, const double duration, base::State *result)> PostPropagationEvent;

            /// \brief Parameterized constructor.  Takes a reference to SpaceInformation,
            /// an ODE to solve, and the integration step size.
            ODESolver (const SpaceInformationPtr& si, const ODE& ode, double intStep) : si_(si), ode_(ode), intStep_(intStep)
            {
            }

            /// \brief Destructor.
            virtual ~ODESolver ()
            {
            }

            /// \brief Set the ODE to solve
            void setODE (const ODE &ode)
            {
                ode_ = ode;
            }

            /// \brief Return the size of a single numerical integration step
            double getIntegrationStepSize () const
            {
                return intStep_;
            }

            /// \brief Set the size of a single numerical integration step
            void setIntegrationStepSize (double intStep)
            {
                intStep_ = intStep;
            }

            /** \brief Get the current instance of the space information */
            const SpaceInformationPtr& getSpaceInformation() const
            {
                return si_;
            }

            /// \brief Retrieve a StatePropagator object that solves a system of ordinary
            /// differential equations defined by an ODESolver.
            /// An optional PostPropagationEvent can also be specified as a callback after
            /// numerical integration is finished for further operations on the resulting
            /// state.
            static StatePropagatorPtr getStatePropagator (ODESolverPtr solver,
                const PostPropagationEvent &postEvent = NULL)
            {
                class ODESolverStatePropagator : public StatePropagator
                {
                    public:
                        ODESolverStatePropagator (ODESolverPtr solver, const PostPropagationEvent &pe) : StatePropagator (solver->si_), solver_(solver), postEvent_(pe)
                        {
                            if (!solver.get())
                                OMPL_ERROR("ODESolverPtr does not reference a valid ODESolver object");
                        }

                        virtual void propagate (const base::State *state, const Control *control, const double duration, base::State *result) const
                        {
                            ODESolver::StateType reals;
                            si_->getStateSpace()->copyToReals(reals, state);
                            solver_->solve (reals, control, duration);
                            si_->getStateSpace()->copyFromReals(result, reals);

                            if (postEvent_)
                                postEvent_ (state, control, duration, result);
                        }

                    protected:
                        ODESolverPtr solver_;
                        ODESolver::PostPropagationEvent postEvent_;
                };
                return StatePropagatorPtr(dynamic_cast<StatePropagator*>(new ODESolverStatePropagator(solver, postEvent)));
            }

        protected:

            /// \brief Solve the ODE given the initial state, and a control to apply for some duration.
            virtual void solve (StateType &state, const Control *control, const double duration) const = 0;

            /// \brief The SpaceInformation that this ODESolver operates in.
            const SpaceInformationPtr     si_;

            /// \brief Definition of the ODE to find solutions for.
            ODE                           ode_;

            /// \brief The size of the numerical integration step.  Should be small to minimize error.
            double                        intStep_;

            /// @cond IGNORE
            // Functor used by the boost::numeric::odeint stepper object
            struct ODEFunctor
            {
                ODEFunctor (const ODE &o, const Control *ctrl) : ode(o), control(ctrl) {}

                // boost::numeric::odeint will callback to this method during integration to evaluate the system
                void operator () (const StateType &current, StateType &output, double /*time*/)
                {
                    ode (current, control, output);
                }

                ODE ode;
                const Control *control;
            };
            /// @endcond
        };

        /// \brief Basic solver for ordinary differential equations of the type q' = f(q, u),
        /// where q is the current state of the system and u is a control applied to the
        /// system.  StateType defines the container object describing the state of the system.
        /// Solver is the numerical integration method used to solve the equations.  The default
        /// is a fourth order Runge-Kutta method.  This class wraps around the simple stepper
        /// concept from boost::numeric::odeint.
        template <class Solver = odeint::runge_kutta4<ODESolver::StateType> >
        class ODEBasicSolver : public ODESolver
        {
        public:

            /// \brief Parameterized constructor.  Takes a reference to the SpaceInformation,
            /// an ODE to solve, and an optional integration step size - default is 0.01
            ODEBasicSolver (const SpaceInformationPtr &si, const ODESolver::ODE &ode, double intStep = 1e-2) : ODESolver(si, ode, intStep)
            {
            }

        protected:

            /// \brief Solve the ODE using boost::numeric::odeint.
            virtual void solve (StateType &state, const Control *control, const double duration) const
            {
                Solver solver;
                ODESolver::ODEFunctor odefunc (ode_, control);
                odeint::integrate_const (solver, odefunc, state, 0.0, duration, intStep_);
            }
        };

        /// \brief Solver for ordinary differential equations of the type q' = f(q, u),
        /// where q is the current state of the system and u is a control applied to the
        /// system.  StateType defines the container object describing the state of the system.
        /// Solver is the numerical integration method used to solve the equations.  The default
        /// is a fifth order Runge-Kutta Cash-Karp method with a fourth order error bound.
        /// This class wraps around the error stepper concept from boost::numeric::odeint.
        template <class Solver = odeint::runge_kutta_cash_karp54<ODESolver::StateType> >
        class ODEErrorSolver : public ODESolver
        {
        public:
            /// \brief Parameterized constructor.  Takes a reference to the SpaceInformation,
            /// an ODE to solve, and the integration step size - default is 0.01
            ODEErrorSolver (const SpaceInformationPtr &si, const ODESolver::ODE &ode, double intStep = 1e-2) : ODESolver(si, ode, intStep)
            {
            }

            /// \brief Retrieves the error values from the most recent integration
            ODESolver::StateType getError ()
            {
                return error_;
            }

        protected:
            /// \brief Solve the ODE using boost::numeric::odeint.  Save the resulting error values into error_.
            virtual void solve (StateType &state, const Control *control, const double duration) const
            {
                ODESolver::ODEFunctor odefunc (ode_, control);

                if (error_.size () != state.size ())
                    error_.assign (state.size (), 0.0);

                Solver solver;
                solver.adjust_size (state);

                double time = 0.0;
                while (time < duration + std::numeric_limits<float>::epsilon())
                {
                    solver.do_step (odefunc, state, time, intStep_, error_);
                    time += intStep_;
                }
            }

            /// \brief The error values calculated during numerical integration
            mutable ODESolver::StateType error_;
        };

        /// \brief Adaptive step size solver for ordinary differential equations of the type
        /// q' = f(q, u), where q is the current state of the system and u is a control applied
        /// to the system.  The maximum integration error is bounded in this approach.
        /// Solver is the numerical integration method used to solve the equations, and must implement
        /// the error stepper concept from boost::numeric::odeint.  The default
        /// is a fifth order Runge-Kutta Cash-Karp method with a fourth order error bound.
        template <class Solver = odeint::runge_kutta_cash_karp54<ODESolver::StateType> >
        class ODEAdaptiveSolver : public ODESolver
        {
        public:
            /// \brief Parameterized constructor.  Takes a reference to the SpaceInformation,
            /// an ODE to solve, and an optional integration step size - default is 0.01
            ODEAdaptiveSolver (const SpaceInformationPtr &si, const ODESolver::ODE &ode, double intStep = 1e-2) : ODESolver(si, ode, intStep), maxError_(1e-6), maxEpsilonError_(1e-7)
            {
            }

            /// \brief Retrieve the total error allowed during numerical integration
            double getMaximumError () const
            {
                return maxError_;
            }

            /// \brief Set the total error allowed during numerical integration
            void setMaximumError (double error)
            {
                maxError_ = error;
            }

            /// \brief Retrieve the error tolerance during one step of numerical integration (local truncation error)
            double getMaximumEpsilonError () const
            {
                return maxEpsilonError_;
            }

            /// \brief Set the error tolerance during one step of numerical integration (local truncation error)
            void setMaximumEpsilonError (double error)
            {
                maxEpsilonError_ = error;
            }

        protected:

            /// \brief Solve the ordinary differential equation given the input state
            /// of the system, a control to apply to the system, and the duration to
            /// apply the control.  The value of \e state will contain the final
            /// values for the system after integration.
            virtual void solve (StateType &state, const Control *control, const double duration) const
            {
                ODESolver::ODEFunctor odefunc (ode_, control);

#if BOOST_VERSION < 105600
                odeint::controlled_runge_kutta< Solver > solver (odeint::default_error_checker<double>(maxError_, maxEpsilonError_));
#else
                typename boost::numeric::odeint::result_of::make_controlled< Solver >::type solver = make_controlled( 1.0e-6 , 1.0e-6 , Solver() );
#endif
                odeint::integrate_adaptive (solver, odefunc, state, 0.0, duration, intStep_);
            }

            /// \brief The maximum error allowed when performing numerical integration
            double maxError_;

            /// \brief The maximum error allowed during one step of numerical integration
            double maxEpsilonError_;
        };
    }
}

#endif

#endif
