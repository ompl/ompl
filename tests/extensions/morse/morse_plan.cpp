/* morse_plan.cpp */

#include "ompl/extensions/morse/MorseEnvironment.h"
#include "ompl/extensions/morse/MorseStateSpace.h"
#include "ompl/extensions/morse/MorseControlSpace.h"
#include "ompl/extensions/morse/MorseSimpleSetup.h"
#include "ompl/base/Goal.h"
#include "ompl/util/Console.h"
#include "ompl/util/ClassForward.h"

#include <vector>

using namespace ompl;

class MyEnvironment : public base::MorseEnvironment
{
public:
    MyEnvironment(const unsigned int rigidBodies, const unsigned int controlDim,
        const std::vector<double> &controlBounds, const std::vector<double> &positionBounds,
        const std::vector<double> &linvelBounds, const std::vector<double> &angvelBounds)
        : base::MorseEnvironment(rigidBodies, controlDim, controlBounds, positionBounds, linvelBounds, angvelBounds)
    {
    }
    void prepareStateRead(void)
    {
        // load fake state data
        for (unsigned int i = 0; i < 3*rigidBodies_; i++)
        {
            positions[i] = 1.0;
            linVelocities[i] = 1.0;
            angVelocities[i] = 1.0;
        }
        for (unsigned int i = 0; i < 4*rigidBodies_; i++)
        {
            quaternions[i] = 1.0;
            if (i%4)
                quaternions[i] = 0.0;
        }
    }
    void finalizeStateWrite(void)
    {
        // nothing to do
    }
    void applyControl(const std::vector<double> &control)
    {
        // nothing to do
    }
    void worldStep(const double dur)
    {
        // nothing to do
    }
};

class MyGoal : public base::Goal
{
public:
    MyGoal(base::SpaceInformationPtr si)
        : base::Goal(si)
    {
    }
    bool isSatisfied(const base::State *state) const
    {
        static int c = 0;
        if (++c == 10)
            return true;
        return false;
    }
};

int main()
{
    // Control Bounds: velocity <= 10 m/s, turning angle <= ~pi/3
    std::vector<double> cbounds(4);
    cbounds[0] = -10;
    cbounds[1] = 10;
    cbounds[2] = -1;
    cbounds[3] = 1;
    // Position Bounds: stay inside 200x200x200 m cube at origin
    std::vector<double> pbounds(6);
    pbounds[0] = -100;
    pbounds[1] = 100;
    pbounds[2] = -100;
    pbounds[3] = 100;
    pbounds[4] = -100;
    pbounds[5] = 100;
    // Linear Velocity Bounds: velocity in any axis <= 10 m/s
    std::vector<double> lbounds(6);
    lbounds[0] = -10;
    lbounds[1] = 10;
    lbounds[2] = -10;
    lbounds[3] = 10;
    lbounds[4] = -10;
    lbounds[5] = 10;
    // Angular Velocity Bounds: rotation <= ~1 rps on every axis
    std::vector<double> abounds(6);
    abounds[0] = -6;
    abounds[1] = 6;
    abounds[2] = -6;
    abounds[3] = 6;
    abounds[4] = -6;
    abounds[5] = 6;
    base::MorseEnvironmentPtr env(new MyEnvironment(2, 2, cbounds, pbounds, lbounds, abounds));
    
    control::SimpleSetupPtr ss(new control::MorseSimpleSetup(env));
    
    //base::StateSpacePtr space = ss->getStateSpace();
    
    // The right way; this works, but can't be done via the py-bindings because control::SpaceInformation is not exposed
    base::GoalPtr g(new MyGoal(ss->getSpaceInformation()));
    
    // The wrong way; this crashes when RRT starts and tries to create a new RRT::Motion,
    //  which entails the SpaceInformation calling controlSpace_->allocControl(), which
    //  can't work when we use what is merely a base::SpaceInformation instead of
    //  control::SpaceInformation that inherits from base::SpaceInformation.
    /*
    base::StateSpacePtr space = ss->getStateSpace();
    base::GoalPtr g(new MyGoal(base::StateInformation(space)));
    */
    
    ss->setGoal(g);
    
    bool solved = ss->solve(1.0);
    if (solved)
    {
        OMPL_INFORM("Solution found!");
    }
    else
    {
        OMPL_INFORM("No solution found.");
    }

    return 0;
}

