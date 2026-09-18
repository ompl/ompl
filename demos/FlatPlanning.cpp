// demos/FlatPlanning.cpp
//
// Plans a Panda through the sphere cage as a differentially flat system.
// The planner's states are joint angles stacked with joint velocities, edges between them are the
// polynomials that spend the least effort getting from one to the other, and the solution comes back as
// a trajectory a controller can run rather than a list of waypoints.
//
// The whole flat setup is the four lines under "flat setup" below.
// Everything else is the scenario and the reporting.

#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/FlatEffortObjective.h>
#include <ompl/base/spaces/FlatStateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/FlatTrajectory.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <ompl/vamp/VampStateSpace.h>
#include <ompl/vamp/VampStateValidityChecker.h>

#include <vamp/collision/factory.hh>
#include <vamp/robots/panda.hh>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using Robot = vamp::robots::Panda;
using Environment = vamp::collision::Environment<vamp::FloatVector<vamp::FloatVectorWidth>>;

namespace
{
    constexpr unsigned int ORDER = 2;
    constexpr double SPEED_LIMIT = 3.;
    constexpr double SOLVE_SECONDS = 10.;

    /// A derivative level of a flat state whose Euclidean norm is capped.
    class BallDerivativeSpace : public ob::RealVectorStateSpace
    {
    public:
        BallDerivativeSpace(unsigned int dimension, double radius)
          : ob::RealVectorStateSpace(dimension), radius_(radius)
        {
            setName("Ball" + getName());
            setBounds(-radius, radius);
        }

        double getRadius() const
        {
            return radius_;
        }

        bool satisfiesBounds(const ob::State *state) const override
        {
            return norm(state) <= radius_;
        }

        void enforceBounds(ob::State *state) const override
        {
            const double length = norm(state);
            if (length > radius_)
            {
                // Aiming at exactly the radius rounds the wrong way often enough to matter, leaving over
                // a quarter of enforced states above it at a radius of 3, so this aims a few of the last
                // bits inside and satisfiesBounds above can compare exactly.
                // Clamping to a box needs none of this, since assigning a bound lands on it exactly.
                const double target = radius_ * (1. - 4. * std::numeric_limits<double>::epsilon());
                double *values = state->as<StateType>()->values;
                for (unsigned int i = 0; i < getDimension(); ++i)
                    values[i] *= target / length;
            }
        }

        ob::StateSamplerPtr allocDefaultStateSampler() const override;

        double norm(const ob::State *state) const
        {
            const double *values = state->as<StateType>()->values;
            double total = 0.;
            for (unsigned int i = 0; i < getDimension(); ++i)
                total += values[i] * values[i];
            return std::sqrt(total);
        }

    private:
        double radius_;
    };

    /// Draws uniformly from inside the ball.
    ///
    /// Rejecting draws from the box around the ball would work in the plane and not here, since a ball
    /// takes up under four percent of the box around it in seven dimensions.
    class BallSampler : public ob::StateSampler
    {
    public:
        explicit BallSampler(const BallDerivativeSpace *space)
          : ob::StateSampler(space), ball_(space), values_(space->getDimension(), 0.)
        {
        }

        void sampleUniform(ob::State *state) override
        {
            rng_.uniformInBall(ball_->getRadius(), values_);
            double *target = state->as<ob::RealVectorStateSpace::StateType>()->values;
            for (std::size_t i = 0; i < values_.size(); ++i)
                target[i] = values_[i];
        }

        void sampleUniformNear(ob::State *state, const ob::State *near, double distance) override
        {
            rng_.uniformInBall(distance, values_);
            const double *center = near->as<ob::RealVectorStateSpace::StateType>()->values;
            double *target = state->as<ob::RealVectorStateSpace::StateType>()->values;
            for (std::size_t i = 0; i < values_.size(); ++i)
                target[i] = center[i] + values_[i];

            ball_->enforceBounds(state);
        }

        void sampleGaussian(ob::State *state, const ob::State *mean, double stdDev) override
        {
            const double *center = mean->as<ob::RealVectorStateSpace::StateType>()->values;
            double *target = state->as<ob::RealVectorStateSpace::StateType>()->values;
            for (std::size_t i = 0; i < values_.size(); ++i)
                target[i] = rng_.gaussian(center[i], stdDev);

            ball_->enforceBounds(state);
        }

    private:
        const BallDerivativeSpace *ball_;
        std::vector<double> values_;
    };

    ob::StateSamplerPtr BallDerivativeSpace::allocDefaultStateSampler() const
    {
        return std::make_shared<BallSampler>(this);
    }

    /// The cage of spheres the arm has to thread its way out of.
    Environment sphereCage()
    {
        vamp::collision::Environment<float> environment;

        const std::vector<std::array<float, 3>> centers = {
            {0.55, 0, 0.25},  {0.35, 0.35, 0.25},  {0, 0.55, 0.25},   {-0.55, 0, 0.25},   {-0.35, -0.35, 0.25},
            {0, -0.55, 0.25}, {0.35, -0.35, 0.25}, {0.35, 0.35, 0.8}, {0, 0.55, 0.8},     {-0.35, 0.35, 0.8},
            {-0.55, 0, 0.8},  {-0.35, -0.35, 0.8}, {0, -0.55, 0.8},   {0.35, -0.35, 0.8},
        };

        for (const auto &center : centers)
            environment.spheres.emplace_back(vamp::collision::factory::sphere::array(center, 0.2f));
        environment.sort();

        return Environment(environment);
    }

    /// Put the arm at \e joints and hold it still there.
    void atRest(const ob::FlatStateSpace *space, ob::State *state, const std::array<double, Robot::dimension> &joints)
    {
        Eigen::MatrixXd flat = Eigen::MatrixXd::Zero(ORDER, Robot::dimension);
        for (unsigned int i = 0; i < Robot::dimension; ++i)
            flat(0, i) = joints[i];
        space->fromFlatState(flat, state);
    }
}  // namespace

int main()
{
    const Environment environment = sphereCage();

    // The flat setup.
    auto output = std::make_shared<ompl::vamp::VampStateSpace<Robot>>();
    auto velocity = std::make_shared<BallDerivativeSpace>(Robot::dimension, SPEED_LIMIT);
    auto space = std::make_shared<ob::FlatStateSpace>(output, std::vector<ob::StateSpacePtr>{velocity});

    og::SimpleSetup setup(space);

    // The collision checker only knows about joint angles, so it gets handed the flat output and nothing
    // else.
    // The bounds go in the same check because the steering polynomial overshoots, and a velocity inside
    // its cap at both ends of an edge runs over it in the middle.
    auto outputInformation = std::make_shared<ob::SpaceInformation>(space->getOutputSpace());
    auto collision = std::make_shared<ompl::vamp::VampStateValidityChecker<Robot>>(outputInformation, environment);
    setup.setStateValidityChecker(
        [space, collision](const ob::State *state)
        {
            return space->satisfiesBounds(state) &&
                   collision->isValid(state->as<ob::FlatStateSpace::StateType>()->output());
        });

    setup.setOptimizationObjective(std::make_shared<ob::FlatEffortObjective>(setup.getSpaceInformation()));

    // The arm starts tucked and finishes reaching across itself, both at rest.
    ob::ScopedState<> start(space), goal(space);
    atRest(space.get(), start.get(), {0., -0.785, 0., -2.356, 0., 1.571, 0.785});
    atRest(space.get(), goal.get(), {2.35, 1., 0., -0.8, 0., 2.5, 0.785});
    setup.setStartAndGoalStates(start, goal);

    // RRTConnect checks each edge the way its path runs through it, so it holds up on a space whose edges
    // only run one way.
    setup.setPlanner(std::make_shared<og::RRTConnect>(setup.getSpaceInformation()));

    const auto began = std::chrono::steady_clock::now();
    const ob::PlannerStatus status = setup.solve(SOLVE_SECONDS);
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();

    if (status != ob::PlannerStatus::EXACT_SOLUTION)
    {
        std::cout << "No solution in " << SOLVE_SECONDS << " seconds\n";
        return 1;
    }

    og::PathGeometric &path = setup.getSolutionPath();
    std::cout << "Solved in " << elapsed << " seconds over " << path.getStateCount() << " states\n";
    std::cout << "The path passes its own validity check: " << std::boolalpha << path.check() << "\n";

    // A controller runs the trajectory.
    // Each segment is one polynomial the planner steered through and already checked, so nothing gets
    // re-fitted on the way out.
    const ob::FlatTrajectory trajectory(path);
    std::cout << "Trajectory of " << trajectory.size() << " segments lasting " << trajectory.duration() << " seconds\n";

    ob::Cost cost = setup.getOptimizationObjective()->identityCost();
    for (std::size_t i = 1; i < path.getStateCount(); ++i)
        cost = setup.getOptimizationObjective()->combineCosts(
            cost, setup.getOptimizationObjective()->motionCost(path.getState(i - 1u), path.getState(i)));
    std::cout << "Steering cost " << cost.value() << "\n";

    // Sampling the whole run confirms the velocity limit held everywhere, not only at the states the
    // planner steered between.
    Eigen::VectorXd rate(Robot::dimension);
    double fastest = 0.;
    double fastestJoint = 0.;
    constexpr unsigned int SAMPLES = 20000;
    for (unsigned int i = 0; i <= SAMPLES; ++i)
    {
        trajectory.evaluate(trajectory.duration() * i / SAMPLES, 1u, rate);
        fastest = std::max(fastest, rate.norm());
        fastestJoint = std::max(fastestJoint, rate.cwiseAbs().maxCoeff());
    }

    std::cout << "Fastest the arm moves over the run " << fastest << " against a cap of " << SPEED_LIMIT << "\n";
    std::cout << "Fastest any one joint moves " << fastestJoint << "\n";
    return fastest <= SPEED_LIMIT ? 0 : 1;
}
