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
            /// the current state, input control, current time, and output state.
            typedef boost::function4<void, const StateType &, const Control*, double, StateType &> ODE;

            /// \brief Definition of the optional user defined propagation function for the system defined by
            /// the ODE.  This method is tasked with converting the base::State values to the proper input for
            /// the solve method in the solver.
            typedef boost::function4<void, const base::State*, const Control *, const double, base::State *> PropagateFunction;

            /// \brief Parameterized constructor.  Takes a reference to the StateSpace,
            /// an ODE to solve, and the integration step size.
            ODESolver (const base::StateSpacePtr &space, const ODE &ode, double intStep) : space_(space), ode_(ode), intStep_(intStep), msg_("ODESolver")
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

            /// \brief Set the propagate function for the ODESolver.  This method
            /// replaces the default method propagateDefault.
            void setPropagateFunction (const PropagateFunction &function)
            {
                propFunc_ = function;
            }

            /// \brief Clear the propagate function for the ODESolver.  This reverts
            /// the propagate functionality back to propagateDefault.
            void clearPropagateFunction (void)
            {
                propFunc_ = NULL;
            }

            virtual void propagate (const base::State *state, const Control *control, const double duration, base::State *result)
            {
                if (propFunc_)
                    propFunc_ (state, control, duration, result);
                else
                    propagateDefault (state, control, duration, result);
            }

        protected:

            /// \brief Solve the ODE given the initial state, and a control to apply for some duration.
            virtual void solve (StateType &state, const Control* control, const double duration) = 0;

            /// \brief Propagate the system defined by the ODE starting at \e state
            /// given a \e control to apply for some positive \e duration.  The resulting
            /// state of the system is stored into \e result.
            virtual void propagateDefault (const base::State *state, const Control *control, const double duration, base::State *result)
            {
                assert (duration > 0.0);

                // Convert the state values to a portable data type
                StateType reals;
                getReals (reals, state);

                // Solve the ODE
                solve (reals, control, duration);

                // Set the resulting state values from the computed solution
                setReals (reals, result);
            }

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

            /// \brief The StateSpace that this ODESolver operates in.
            const base::StateSpacePtr  space_;

            /// \brief Definition of the ODE to find solutions for.
            ODE                        ode_;

            /// \brief The size of the numerical integration step.  Should be small to minimize error.
            double                     intStep_;

            /// \brief Interface used for reporting errors
            msg::Interface             msg_;

            /// \brief Optional user specified propagate function
            PropagateFunction          propFunc_;

            /// Functor used by the boost::numeric::odeint stepper object
            struct ODEFunctor
            {
                ODEFunctor (const ODE &o, const Control* ctrl) : ode(o), control(ctrl) {}

                // boost::numeric::odeint will callback to this method during integration to evaluate the system
                void operator () (const StateType &current, StateType &output, double time)
                {
                    ode (current, control, time, output);
                }

                ODE ode;
                const Control* control;
            };
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

            /// \brief Parameterized constructor.  Takes a reference to the StateSpace, 
            /// an ODE to solve, and an optional integration step size - default is 0.01
            ODEBasicSolver (const base::StateSpacePtr &space, const ODESolver::ODE &ode = NULL, double intStep = 1e-2) : ODESolver(space, ode, intStep)
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
            /// \brief Parameterized constructor.  Takes a reference to the StateSpace,
            /// an ODE to solve, and the integration step size - default is 0.01
            ODEErrorSolver (const base::StateSpacePtr &space, const ODESolver::ODE &ode = NULL, double intStep = 1e-2) : ODESolver(space, ode, intStep)
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
            /// \brief Parameterized constructor.  Takes a reference to the StateSpace,
            /// an ODE to solve, and an optional integration step size - default is 0.01
            ODEAdaptiveSolver (const base::StateSpacePtr &space, const ODESolver::ODE &ode = NULL, double intStep = 1e-2) : ODESolver(space, ode, intStep), maxError_(1e-6), maxEpsilonError_(1e-7)
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
