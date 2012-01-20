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

#include "ompl/control/Control.h"
#include "ompl/control/StatePropagator.h"
#include "ompl/util/Console.h"
#include "ompl/base/StateSpace.h"

#include <boost/numeric/odeint.hpp>
#include <boost/function.hpp>
#include <cassert>
#include <vector>

namespace ompl
{

    namespace control
    {
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
            typedef boost::function3<void, const StateType &, const Control*, StateType &> ODE;

            /// \brief Callback function to perform an event at the end of numerical
            /// integration.  This functionality is optional.
            typedef boost::function2<void, const Control*, base::State*> PostPropagationEvent;

            /// \brief Parameterized constructor.  Takes a reference to SpaceInformation,
            /// an ODE to solve, and the integration step size.
            ODESolver (const SpaceInformationPtr &si, const ODE &ode, double intStep) : si_(si), space_(si->getStateSpace()), ode_(ode), intStep_(intStep), msg_("ODESolver")
            {
            }

            /// \brief Destructor.
            virtual ~ODESolver (void)
            {
            }

            /// \brief Set the ODE to solve
            void setODE (const ODE &ode)
            {
                ode_ = ode;
            }

            /// \brief Return the size of a single numerical integration step
            double getIntegrationStepSize (void) const
            {
                return intStep_;
            }

            /// \brief Set the size of a single numerical integration step
            void setIntegrationStepSize (double intStep)
            {
                intStep_ = intStep;
            }

            /// \brief Retrieve a StatePropagator object that solves the system of ordinary
            /// differential equations defined by this ODESolver.
            /// An optional PostPropagationEvent can also be specified as a callback after
            /// numerical integration is finished for further operations on the resulting
            /// state. If enforceBounds is true, StateSpace::enforceBounds will be invoked on
            /// the resulting state.
            StatePropagatorPtr getStatePropagator (const PostPropagationEvent &postEvent = NULL, bool enforceBounds = true)
            {
                class ODESolverStatePropagatorAndBoundsEnforcer : public StatePropagator
                {
                    public:
                        ODESolverStatePropagatorAndBoundsEnforcer (const SpaceInformationPtr& si, ODESolver *solver, const ODESolver::PostPropagationEvent &pe) : StatePropagator (si), solver_(solver), space_(si->getStateSpace()), postEvent_(pe)
                        {
                        }

                        virtual void propagate (const base::State *state, const Control* control, const double duration, base::State *result) const
                        {
                            ODESolver::StateType reals;

                            solver_->getReals (reals, state);
                            solver_->solve (reals, control, duration);
                            solver_->setReals (reals, result);

                            space_->enforceBounds (result);

                            if (postEvent_)
                                postEvent_ (control, result);
                        }

                    protected:
                        ODESolver *solver_;
                        base::StateSpacePtr space_;
                        ODESolver::PostPropagationEvent postEvent_;
                };

                class ODESolverStatePropagator : public StatePropagator
                {
                    public:
                        ODESolverStatePropagator (const SpaceInformationPtr& si, ODESolver *solver, const PostPropagationEvent &pe) : StatePropagator (si), solver_(solver), postEvent_(pe)
                        {
                        }

                        virtual void propagate (const base::State *state, const Control* control, const double duration, base::State *result) const
                        {
                            ODESolver::StateType reals;

                            solver_->getReals (reals, state);
                            solver_->solve (reals, control, duration);
                            solver_->setReals (reals, result);

                            if (postEvent_)
                                postEvent_ (control, result);
                        }

                    protected:
                        ODESolver *solver_;
                        ODESolver::PostPropagationEvent postEvent_;
                };

                if (enforceBounds)
                    return StatePropagatorPtr(dynamic_cast<StatePropagator*>(new ODESolverStatePropagatorAndBoundsEnforcer(si_, this, postEvent)));
                else
                    return StatePropagatorPtr(dynamic_cast<StatePropagator*>(new ODESolverStatePropagator(si_, this, postEvent)));
            }

        protected:

            /// \brief Solve the ODE given the initial state, and a control to apply for some duration.
            virtual void solve (StateType &state, const Control* control, const double duration) = 0;

            /// \brief Get a container that holds all real values of the input state
            void getReals (StateType &reals, const base::State *state) const
            {
                reals.clear ();

                unsigned int index = 0;
                while (double *val = space_->getValueAddressAtIndex(const_cast<base::State *>(state), index++))
                    reals.push_back(*val);
            }

            /// \brief Sets the values of the state to those contained in the reals container.
            void setReals (const StateType &reals, base::State *state)
            {
                for (size_t i = 0; i < reals.size (); ++i)
                {
                    double *val = space_->getValueAddressAtIndex(state, i);
                    if (val)
                    {
                        *val = reals[i];
                    }
                }
            }

            /// \brief The SpaceInformation that this ODESolver operates in.
            const SpaceInformationPtr     si_;

            /// \brief Reference to the state space the system operates in.
            const base::StateSpacePtr     space_;

            /// \brief Definition of the ODE to find solutions for.
            ODE                           ode_;

            /// \brief The size of the numerical integration step.  Should be small to minimize error.
            double                        intStep_;

            /// \brief Interface used for reporting errors
            msg::Interface                msg_;

            /// @cond IGNORE
            // Functor used by the boost::numeric::odeint stepper object
            struct ODEFunctor
            {
                ODEFunctor (const ODE &o, const Control* ctrl) : ode(o), control(ctrl) {}

                // boost::numeric::odeint will callback to this method during integration to evaluate the system
                void operator () (const StateType &current, StateType &output, double /*time*/)
                {
                    ode (current, control, output);
                }

                ODE ode;
                const Control* control;
            };
            /// @endcond
        };

        /// \brief Basic solver for ordinary differential equations of the type q' = f(q, u),
        /// where q is the current state of the system and u is a control applied to the
        /// system.  StateType defines the container object describing the state of the system.
        /// Solver is the numerical integration method used to solve the equations.  The default
        /// is a fourth order Runge-Kutta method.  This class wraps around the simple stepper
        /// concept from boost::numeric::odeint.
        template <class Solver = boost::numeric::odeint::runge_kutta4<ODESolver::StateType> >
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
            virtual void solve (StateType &state, const Control* control, const double duration)
            {
                ODESolver::ODEFunctor odefunc (ode_, control);
                boost::numeric::odeint::integrate_const (solver_, odefunc, state, 0.0, duration, intStep_);
            }

            /// \brief The numerical method used to compute a solution to the given ODE.
            Solver solver_;
        };

        /// \brief Solver for ordinary differential equations of the type q' = f(q, u),
        /// where q is the current state of the system and u is a control applied to the
        /// system.  StateType defines the container object describing the state of the system.
        /// Solver is the numerical integration method used to solve the equations.  The default
        /// is a fifth order Runge-Kutta Cash-Karp method with a fourth order error bound.
        /// This class wraps around the error stepper concept from boost::numeric::odeint.
        template <class Solver = boost::numeric::odeint::runge_kutta_cash_karp54<ODESolver::StateType> >
        class ODEErrorSolver : public ODESolver
        {
        public:
            /// \brief Parameterized constructor.  Takes a reference to the SpaceInformation,
            /// an ODE to solve, and the integration step size - default is 0.01
            ODEErrorSolver (const SpaceInformationPtr &si, const ODESolver::ODE &ode, double intStep = 1e-2) : ODESolver(si, ode, intStep)
            {
            }

            /// \brief Retrieves the error values from the most recent integration
            ODESolver::StateType getError (void)
            {
                ODESolver::StateType error (error_.begin (), error_.end ());
                return error;
            }

        protected:
            /// \brief Solve the ODE using boost::numeric::odeint.  Save the resulting error values into error_.
            virtual void solve (StateType &state, const Control* control, const double duration)
            {
                ODESolver::ODEFunctor odefunc (ode_, control);

                if (error_.size () != state.size ())
                    error_.assign (state.size (), 0.0);

                solver_.adjust_size (state);

                double time = 0.0;
                while (time < duration)
                {
                    solver_.do_step (odefunc, state, time, intStep_, error_);
                    time += intStep_;
                }
            }

            /// \brief The numerical method used to compute a solution to the given ODE.
            Solver solver_;

            /// \brief The error values calculated during numerical integration
            ODESolver::StateType error_;
        };

        /// \brief Adaptive step size solver for ordinary differential equations of the type
        /// q' = f(q, u), where q is the current state of the system and u is a control applied
        /// to the system.  The maximum integration error is bounded in this approach.
        /// Solver is the numerical integration method used to solve the equations, and must implement
        /// the error stepper concept from boost::numeric::odeint.  The default
        /// is a fifth order Runge-Kutta Cash-Karp method with a fourth order error bound.
        template <class Solver = boost::numeric::odeint::runge_kutta_cash_karp54<ODESolver::StateType> >
        class ODEAdaptiveSolver : public ODESolver
        {
        public:
            /// \brief Parameterized constructor.  Takes a reference to the SpaceInformation,
            /// an ODE to solve, and an optional integration step size - default is 0.01
            ODEAdaptiveSolver (const SpaceInformationPtr &si, const ODESolver::ODE &ode, double intStep = 1e-2) : ODESolver(si, ode, intStep), maxError_(1e-6), maxEpsilonError_(1e-7)
            {
            }

            /// \brief Retrieve the total error allowed during numerical integration
            double getMaximumError (void) const
            {
                return maxError_;
            }

            /// \brief Set the total error allowed during numerical integration
            void setMaximumError (double error)
            {
                maxError_ = error;
            }

            /// \brief Retrieve the error tolerance during one step of numerical integration (local truncation error)
            double getMaximumEpsilonError (void) const
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
            virtual void solve (StateType &state, const Control* control, const double duration)
            {
                ODESolver::ODEFunctor odefunc (ode_, control);

                boost::numeric::odeint::controlled_runge_kutta< Solver > solver (boost::numeric::odeint::default_error_checker<double>(maxError_, maxEpsilonError_));
                boost::numeric::odeint::integrate_adaptive (solver, odefunc, state, 0.0, duration, intStep_);
            }

            /// \brief The maximum error allowed when performing numerical integration
            double maxError_;

            /// \brief The maximum error allowed during one step of numerical integration
            double maxEpsilonError_;
        };

    }
}

#endif
