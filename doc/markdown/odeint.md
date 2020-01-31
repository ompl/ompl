# Using the ODESolver for Planning with Controls {#odeint}

[TOC]

OMPL provides a wrapper class for numerically solving differential equations using the [boost::numeric::odeint][odeint] package. A number of other software packages exist to perform numerical integration (e.g., GSL, ALGLIB, Scipy), but the odeint library is specifically chosen due to its feature-rich and easy-to-use implementation, as well as its lack of external dependencies.

The ODESolver class is particularly suited for motion planning problems. It provides the user with an object capable of solving equations of the form \f$\dot{q} = f(q,u)\f$, where \f$q\f$ is the current state of the system and \f$u\f$ is a control applied to the system at state \f$q\f$. Using the ODESolver removes the need for the user to implement numerical integration in their own code, and allows advanced users the ability to customize the method used for integration.

## A Bit of Theory

Assume that you are planning for a simplified car-like system where the velocity \f$v\f$ and steering angle \f$\phi\f$ can be directly controlled. Let the position of the car be described by (x,y,\f$\theta\f$). Then the dynamics of the system can be described by:

\f$
\left(
  \begin{array}{c}
    \dot{x}\\
    \dot{y}\\
    \dot{\theta}
  \end{array}
\right) =
\left(
  \begin{array}{c}
    v \cos(\theta)\\
    v \sin(\theta)\\
    v \tan(\phi)
  \end{array}
\right)
\f$

This shows how a given state of the car model (x,y,\f$\theta\f$) will evolve over time with a single control (\f$v, \phi\f$). Note that the equations above yield a delta (the differential) from the current state, i.e., they compute the change in the state values rather than the new state itself. Therefore, the equations must be integrated to find the new state of the system after applying the control for a given amount of time.

Computing the exact solution for the majority of non-linear differential equations is infeasible, if not impossible. However, it is easy to approximate solutions to these equations by discretizing time into small increments and reevaluating the system during each increment. This discretization, however, introduces error into the computation. Intuitively, a smaller time step generally results in a smaller error value, but takes longer to compute because there are more discrete steps. Without going into detail on the various numerical methods used to approximate solutions, suffice it to say that the "order" of a numerical method indicates the precision of the approach; the higher the order the better. For example, classical [Euler integration](https://mathworld.wolfram.com/EulerForwardMethod.html) is 1st order, indicating that the error during each time step is \f$O(t^2)\f$ and the global error is \f$O(t)\f$, where \f$t\f$ is the size of a single time step. For more information, [Wikipedia](https://en.wikipedia.org/wiki/ Numerical_ordinary_differential_equations) provides a thorough discussion on the theory of numerical integration for ordinary differential equations.

## Using OMPL's ODESolver

To use the ODESolver, a function describing the ODE of your system must be implemented.  This function must have the following signature:

~~~{.cpp}
void ODE(const oc::ODESolver::StateType& q, const oc::Control* u, oc::ODESolver::StateType& qdot)
~~~

This function takes a vector q (StateType is a std::vector) that describes the current state of the system, a control u that defines the inputs applied to the system at state q, and a vector qdot to store the output of the computation. ODESolver utilizes [boost::numeric::odeint][odeint] to perform the numerical integration, and it is necessary to translate the ompl::base::State values into an iterable container of real values.  Therefore, values in the vector q directly correspond to the real valued state parameters in the ompl::base::State.  This data is analogous to the result of calling ompl::base::ScopedState::reals.

### Define the ODE

Assume that you are planning for the simple car-like system described above. The state space of the car is [SE(2)](https://en.wikipedia.org/wiki/Euclidean_group) (x and y position with one angle for orientation). An implementation of this space already exists in OMPL (ompl::base::SE2StateSpace), so it is not necessary to define a new space for the car. The ompl::control::ControlSpace for this simple car model consists of the velocity and steering angle, both real valued. Given these definitions, the ODE defined for the ODESolver then has the following structure:

~~~{.cpp}
namespace oc = ompl::control;

void SimpleCarODE(const oc::ODESolver::StateType& q, const oc::Control* c, oc::ODESolver::StateType& qdot)
{
    // Retrieve control values.  Velocity is the first entry, steering angle is second.
    const double *u = c->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double velocity = u[0];
    const double steeringAngle = u[1];

    // Retrieve the current orientation of the car.  The memory for ompl::base::SE2StateSpace is mapped as:
    // 0: x
    // 1: y
    // 2: theta
    const double theta = q[2];

    // Ensure qdot is the same size as q.  Zero out all values.
    qdot.resize(q.size(), 0);

    qdot[0] = velocity * cos(theta);            // x-dot
    qdot[1] = velocity * sin(theta);            // y-dot
    qdot[2] = velocity * tan(steeringAngle);    // theta-dot
}
~~~

### A Basic Example

When planning with the ODESolver, the user must instantiate the derived solver. All of the solvers require a SpaceInformationPtr that the system operates in to be supplied in the constructor. This is used to extract the values of ompl::base::State into a container for integration via the StateSpace. The ODE itself must also be given to the solver. The simplest solver is the ompl::control::ODEBasicSolver, which uses fourth order [Runge-Kutta](https://mathworld.wolfram.com/Runge-KuttaMethod.html) integration. Given the car-like system and ODE described above, the solver can be instantiated with the following code snippet:

~~~{.cpp}
// SpaceInformationPtr is defined as the variable si.
ompl::control::ODESolverPtr odeSolver (new ompl::control::ODEBasicSolver<> (si, &SimpleCarODE));
~~~

Control based planners also require a ompl::control::StatePropagator object that defines how the system moves given a specific control. The ODESolver provides a wrapper for this functionality via the static getStatePropagator method:

~~~{.cpp}
si->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));
~~~

This is all that is needed for the planner to sample a control and propagate the system by solving the differential equations defined in the ode.  In some systems, a post-numerical integration callback event is useful. As an example, note that the ODE allows the state values to unboundedly change, and for some systems this is not acceptable. For the car-like system above, the orientation parameter \f$\theta\f$ is generally a value between 0 and \f$2\pi\f$, and integration could have the value exceed these bounds. In such a case, it is useful to define a post-integration event callback that is invoked after numerical integration is complete:

~~~{.cpp}
void postPropagate(const base::State* state, const Control* control, const double duration, base::State* result)
{
    ompl::base::SO2StateSpace SO2;

    // Ensure that the car's resulting orientation lies between 0 and 2*pi.
    ompl::base::SE2StateSpace::StateType& s = *result->as<ompl::base::SE2StateSpace::StateType>();
    SO2.enforceBounds(s[1]);
}
~~~

There is one final note about ODESolver and the postPropagate method.  If your system has other state-space components that are not changed in the ODE (i.e. the current gear of a car's transmission), these values must be explicitly copied into the resulting state.  ODESolver will only update the real-values of the state space.  For state spaces with components that are not real-valued, the post-integration event can be used to transfer this information from the initial state to the resulting state.

Once your postPropagate method is implemented, a StatePropagator can then be created which automatically invokes the postPropagate method using a function pointer after each state is propagated:

~~~{.cpp}
si->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, &postPropagate));
~~~

### Selecting an ODESolver

There are three derived classes from ODESolver that use [boost::numeric::odeint][odeint] to solve systems of ordinary differential equations. Each of the solvers is templated with a default argument corresponding to the numerical method used for integration.

- ompl::control::ODEBasicSolver
  A simple explicit ODE solver.  The default method is fourth order Runge-Kutta integration. This solver wraps around the basic stepper concept from boost::numeric::odeint.
- ompl::control::ODEErrorSolver
  An explicit solver where the error estimate can be retrieved.  The default method is a
  fifth-order Runge-Kutta Cash-Karp algorithm with a fourth order error bound.
  This solver wraps around the error stepper concept from boost::numeric::odeint.
- ompl::control::ODEAdaptiveSolver
  An explicit solver that utilizes adaptive time step sizes to ensure that the error estimate is below a given bound. The default method is a fifth-order Runge-Kutta Cash-Karp algorithm with a fourth order error bound used in conjunction with boost::numeric::odeint's controlled Runge-Kutta stepper. This solver must support boost::numeric::odeint's error stepper concept.

Selecting a method for solving your system is more of an art rather than a science, and depends on a number of factors including the complexity of the equations and the cost of evaluating them. Higher order methods tend to be more precise and numerically stable, but come at a higher computational cost.

Finally, the ompl::control::ODESolver base class can also be extended to new, user defined solvers. The ODESolver base itself does not depend on boost::numeric::odeint, and any user specified code or 3rd party library could be used to perform numerical integration.

[odeint]: https://www.boost.org/libs/numeric/odeint
