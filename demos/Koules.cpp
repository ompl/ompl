/* Author: Beck Chen */

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/DiscreteControlSpace.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/config.h>
#include <iostream>
#include <valarray>
#include <vector>
#include <limits>
#include <stdio.h>
#include <math.h>

#define NUM_KOULES 1
#define NUM_ATTEMPTS 3
#define ACC_SHIP 1
#define V_THETA 3.1415926
#define SIDELENGTH 1
#define CENTER_X 0.5
#define CENTER_Y 0.5
#define M_S 0.75
#define M_K 0.5
#define R_S 0.03
#define R_K 0.015
#define LAMBDA 4
#define H 0.05

#define min(x, y) ((x) < (y) ? (x) : (y))

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;


// Ship and Koules model object definition.
class KouleModel
{
public:
    
    KouleModel(const ob::StateSpace *space, int numKoules) : space_(space), num_(numKoules)
    {
    }

    void operator()(const ob::State *state, const oc::Control *control, std::valarray<double> &qdot) const
    {
        int u = control->as<oc::DiscreteControlSpace::ControlType>()->value;
        std::vector<double> q(5+4*num_);
        space_->copyToReals(q, state);
        qdot.resize(5+4*num_, 0);

        // ship: qdot[0,1] is xdot, qdot[2] is thetadot, qdot[3,4] is vdot
	    qdot[0] = q[3];
	    qdot[1] = q[4];
	    if (u == 1)
	        qdot[2] = V_THETA;
	    else if (u == 2)
	        qdot[2] = -V_THETA;
	    else if (u == 3) {
	        qdot[3] = ACC_SHIP * cos(q[2]);
	        qdot[4] = ACC_SHIP * sin(q[2]);
	    }
	    
	    // koules: qdot[5+4*i,6+4*i] is xdot, qdot[7+4*i,8+4*i] is vdot
        int i = 0;
        for (i; i < num_; i++) {
            qdot[5+4*i] = q[7+4*i];
	        qdot[6+4*i] = q[8+4*i];
	        qdot[7+4*i] = (CENTER_X - q[5+4*i]) * LAMBDA - q[7+4*i] * H;
	        qdot[8+4*i] = (CENTER_Y - q[6+4*i]) * LAMBDA - q[8+4*i] * H;
        }
        
	}

    void update(ob::State *state, const std::valarray<double> &qdot, double time) const
    {
        std::vector<double> q(5+4*num_);
        space_->copyToReals(q, state);

        // update collisions
        int i, j, k;
        int hasCollision[1+num_];
        double radiusA = R_S;
        double massA = M_S;
        double radiusB = R_K;
        double massB = M_K;
        for (i = 0; i <= num_; i++) {
            hasCollision[i] = 0;
        }
        for (i = 0; i < num_; i++) {
            for (j = i+1; j <= num_; j++) {
                // check collision
                if (i != 0) {
                    radiusA = R_K;
                    massA = M_K;
                }
                k = checkCollision(&q, i, radiusA, massA, j, radiusB, massB, time);
                hasCollision[i] = hasCollision[i] || k;
                hasCollision[j] = hasCollision[j] || k;
            }
        }
        
        // update objects with no collision according to qdot
        for (i = 0; i <= num_; i++) {
            if (!hasCollision[i]) {
                if (i == 0) {
                    for (j = 0; j < 5; j++)
                        q[j] += qdot[j] * time;
                } else {
                    for (j = 1; j <= 4; j++)
                        q[j+4*i] += qdot[j+4*i] * time;
                }
            }
        }
        space_->copyFromReals(state, q);
        
        // Normalize orientation between 0 and 2*pi
        ob::SO2StateSpace SO2;
        SO2.enforceBounds(state->as<ob::CompoundStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0));

        return;
    }

private:
    
    const ob::StateSpace *space_;
    
    int num_;

    // check collision among object A and B (specified by aIndex and bIndex)
    // algorithm come from http://www.kirupa.com/developer/as3/elastic_collisions_pg2.htm
    int checkCollision(std::vector<double> *q, int aIndex, double radiusA, double massA, int bIndex, double radiusB, double massB, double time) const
    {
        double a[4] = {0, 1, 3, 4};
        double b[4] = {1+4*bIndex, 2+4*bIndex, 3+4*bIndex, 4+4*bIndex};
        if (aIndex != 0) {
            a[0] = 1+4*aIndex;
            a[1] = 2+4*aIndex;
            a[2] = 3+4*aIndex;
            a[3] = 4+4*aIndex;
        }
        
        float delta = 0.001;
        float s [2];  // vector from b to a
        s[0] = (*q)[a[0]] - (*q)[b[0]];
        s[1] = (*q)[a[1]] - (*q)[b[1]];
        float dist = sqrt(s[0]*s[0]+s[1]*s[1]);
        float i = -(*q)[a[2]]*s[0] - (*q)[a[3]]*s[1] + (*q)[b[2]]*s[0] + (*q)[b[3]]*s[1];
        if (dist < radiusA+radiusB+delta && i > 0) { // close enough and moving closer; elastic collision happens
            // compute unit normal and tangent vectors
            float normal [2] = {s[0]/dist, s[1]/dist};
            float tangent [2] = {-normal[1], normal[0]};
            
            // compute scalar projections of velocities onto normal and tangent vectors
            float bNormal = normal[0] * (*q)[b[2]] + normal[1] * (*q)[b[3]];
            float bTangentPrime = tangent[0] * (*q)[b[2]] + tangent[1] * (*q)[b[3]];
            float aNormal = normal[0] * (*q)[a[2]] + normal[1] * (*q)[a[3]];
            float aTangentPrime = tangent[0] * (*q)[a[2]] + tangent[1] * (*q)[a[3]];
            
            // compute new velocities using one-dimensional elastic collision in the normal direction
            float bNormalPrime = (bNormal * (massB - massA) + 2 * massA * aNormal) / (massB + massA);
            float aNormalPrime = (aNormal * (massA - massB) + 2 * massB * bNormal) / (massB + massA);
            
            // compute new normal and tangential velocity vectors
            float bNewNormalVel [2] = {normal[0]*bNormalPrime, normal[1]*bNormalPrime};
            float bNewTangentVel [2] = {tangent[0]*bTangentPrime, tangent[1]*bTangentPrime};
            float aNewNormalVel [2] = {normal[0]*aNormalPrime, normal[1]*aNormalPrime};
            float aNewTangentVel [2] = {tangent[0]*aTangentPrime, tangent[1]*aTangentPrime};
            
            // compute new velocities
            float bNewVel [2] = {bNewNormalVel[0] + bNewTangentVel[0], bNewNormalVel[1] + bNewTangentVel[1]};
            float aNewVel [2] = {aNewNormalVel[0] + aNewTangentVel[0], aNewNormalVel[1] + aNewTangentVel[1]};
            
            // update state if collision happens
            (*q)[a[0]] = (*q)[a[0]] + aNewVel[0] * time;
            (*q)[a[1]] = (*q)[a[1]] + aNewVel[1] * time;
            (*q)[a[2]] = aNewVel[0];
            (*q)[a[3]] = aNewVel[1];
            (*q)[b[0]] = (*q)[b[0]] + bNewVel[0] * time;
            (*q)[b[1]] = (*q)[b[1]] + bNewVel[1] * time;
            (*q)[b[2]] = bNewVel[0];
            (*q)[b[3]] = bNewVel[1];
            
            return 1;
        } else {
            return 0;
        }
    }

};


// State propagator for KouleModel.
class KouleStatePropagator : public oc::StatePropagator
{
public:
    
    KouleStatePropagator(const oc::SpaceInformationPtr &si, int numKoules) : 
        oc::StatePropagator(si), model_(si->getStateSpace().get(), numKoules), timeStep_(0.005) 
    {
    }

    virtual void propagate(const ob::State *start, const oc::Control* control, const double duration, ob::State *result) const
    {
        double t = timeStep_;
        std::valarray<double> qdot;
        si_->getStateSpace().get()->copyState(result, start);
        while (t < duration + std::numeric_limits<double>::epsilon())
        {
            model_(result, control, qdot);
            model_.update(result, qdot, timeStep_);
            t += timeStep_;
        }
        if (t + std::numeric_limits<double>::epsilon() > duration)
        {
            model_(result, control, qdot);
            model_.update(result, qdot, t - duration);
        }
    }

    void setIntegTimeStep(double timeStep)
    {
        timeStep_ = timeStep;
    }

    double getIntegTimeStep()
    {
        return timeStep_;
    }

private:
    
    KouleModel model_;
    double timeStep_;

};


// A projection to 2D space.
class KouleProjection : public ob::ProjectionEvaluator
{
public:

KouleProjection(const ob::StateSpacePtr &space) : ob::ProjectionEvaluator(space)
{
}

virtual unsigned int getDimension(void) const
{
    return 2;
}

virtual void defaultCellSizes(void)
{
    cellSizes_.resize(2);
    cellSizes_[0] = 0.005;
    cellSizes_[1] = 0.005; // TODO: what cell size is the best?
}

virtual void project(const ob::State *state, ob::EuclideanProjection &projection) const
{
    const ob::SE2StateSpace::StateType *s = state->as<ob::CompoundStateSpace::StateType>()->
                       as<ob::SE2StateSpace::StateType>(0);
    projection[0] = s->getX();
    projection[1] = s->getY();
}
};


// Sampleable goal region for KoulesModel.
class KoulesGoal : public ob::GoalSampleableRegion
{
public:

    KoulesGoal(const ob::SpaceInformationPtr &si) : ob::GoalSampleableRegion(si)
    {
        srand(time(NULL));
        threshold_ = 0.1;
        num_ = (si->getStateDimension() - 5) / 4;
    }

    virtual double distanceGoal(const ob::State *st) const
    {
        // the shortest distance between a koule and an edge
        int i;
        double minDist = SIDELENGTH;
        double minX, minY;
        const ob::RealVectorStateSpace::StateType *s;
        for (i = 0; i < num_; i++) {
            s = st->as<ob::CompoundStateSpace::StateType>()
                    ->as<ob::RealVectorStateSpace::StateType>(2+2*i);
            minX = min(s->values[0], SIDELENGTH - s->values[0]);
            minY = min(s->values[1], SIDELENGTH - s->values[1]);
            minDist = min(minDist, min(minX, minY));
        }
        if (minDist < 0)
            minDist = 0;
        return minDist;
    }

    virtual unsigned int maxSampleCount(void) const
    {
        return 10; // TODO: not sure what this number should be
    }

    virtual void sampleGoal(ob::State *st) const
    {
        // randomly sample the position and velocity of the ship
        ob::SE2StateSpace::StateType *shipState = st->as<ob::CompoundStateSpace::StateType>()
                            ->as<ob::SE2StateSpace::StateType>(0);
        ob::RealVectorStateSpace::StateType *shipVel = st->as<ob::CompoundStateSpace::StateType>()
                            ->as<ob::RealVectorStateSpace::StateType>(1);
        shipState->setX((float)rand() / ((float)RAND_MAX / SIDELENGTH));
        shipState->setY((float)rand() / ((float)RAND_MAX / SIDELENGTH));
        shipState->setYaw(0.0);
        shipVel->values[0] = ((float)rand() / ((float)RAND_MAX / 2)) - 1;
        shipVel->values[1] = ((float)rand() / ((float)RAND_MAX / 2)) - 1;

        int j;
        float i;
        ob::RealVectorStateSpace::StateType *kouleState;
        ob::RealVectorStateSpace::StateType *kouleVel;
        for (j = 0; j < num_; j++) {
            // randomly pick an edge for each koule to collide
            kouleState = st->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(2+2*j);
            kouleVel = st->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(3+2*j);
            i = (float)rand() / (float)RAND_MAX;
            if (i < 0.25) {
                kouleState->values[0] = 0;
                kouleState->values[1] = (float)rand() / ((float)RAND_MAX / SIDELENGTH);
            } else if (i < 0.5) {
                kouleState->values[0] = SIDELENGTH;
                kouleState->values[1] = (float)rand() / ((float)RAND_MAX / SIDELENGTH);
            } else if (i < 0.75) {
                kouleState->values[1] = 0;
                kouleState->values[0] = (float)rand() / ((float)RAND_MAX / SIDELENGTH);
            } else {
                kouleState->values[1] = SIDELENGTH;
                kouleState->values[0] = (float)rand() / ((float)RAND_MAX / SIDELENGTH);
            }
            kouleVel->values[0] = 0;
            kouleVel->values[1] = 0;
        } // TODO: maybe just set the last koule to be out of bound?
        
    }
    
private:

    int num_;
};



bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    return si->satisfiesBounds(state);
}



void planWithSimpleSetup(int isBenchmark)
{
    int remainingKoules = NUM_KOULES;
    int remainingTrials = NUM_ATTEMPTS;
    std::vector< std::vector<double> > startStates;
    for (int i = 1; i <= NUM_KOULES; i++) {
        if (i == NUM_KOULES) {
            double arr[5+4*i];
            for (int k = 0; k < 5+4*i; k++)
                arr[k] = 0.0;
            // always place ship at the center
            arr[0] = 0.5;
            arr[1] = 0.5;
            // randomize start position of koules
            int j = 1;
            int k;
            double x, y, xdiff, ydiff;
            bool collision;
            while (j <= NUM_KOULES) {
                // randomize a start position in a center square
                x = (double)(rand() % 7) / 10 + 0.2;
                y = (double)(rand() % 7) / 10 + 0.2;
                collision = false;
                // check collision with ship
                xdiff = x - arr[0];
                ydiff = y - arr[1];
                if (xdiff*xdiff + ydiff*ydiff <= (R_S+R_K)*(R_S+R_K)) {
                    collision = true;
                } else {
                    for (k = 1; k < j; k++) {
                        xdiff = x - arr[1+4*k];
                        ydiff = y - arr[2+4*k];
                        if (xdiff*xdiff + ydiff*ydiff <= 4*R_K*R_K) {
                            collision = true;
                            break;
                        }
                    }
                }
                if (!collision) {
                    arr[1+4*j] = x;
                    arr[2+4*j] = y;
                    j++;
                }
            }
            std::vector<double> s(arr, arr + sizeof(arr) / sizeof(arr[0]));
            startStates.push_back(s);
        } else {
            std::vector<double> s(5+4*i);
            startStates.push_back(s);
        }
    }

    while (remainingKoules > 0) {
    
        if (remainingTrials == 0) {
            if (remainingKoules == NUM_KOULES) {
                std::cout << "Failed to find a solution." << std::endl;
                return;
            } else {
                remainingKoules++;
                remainingTrials = NUM_ATTEMPTS;
            }
        }
        
        remainingTrials--;
        std::cout << "# Remaining Koules: " << remainingKoules << std::endl;
        std::cout << "Trial " << NUM_ATTEMPTS-remainingTrials << std::endl;
        
        // construct state space
        ob::StateSpacePtr space(new ob::CompoundStateSpace());

        // add subspaces for ship and koules
        int i, j;
        space->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::SE2StateSpace()), 2);
        space->as<ob::CompoundStateSpace>()->addSubspace(
                        ob::StateSpacePtr(new ob::RealVectorStateSpace(2)), 1);
        for (i = 0; i < remainingKoules; i++) {
            space->as<ob::CompoundStateSpace>()->addSubspace(
                            ob::StateSpacePtr(new ob::RealVectorStateSpace(2)), 1);
            space->as<ob::CompoundStateSpace>()->addSubspace(
                            ob::StateSpacePtr(new ob::RealVectorStateSpace(2)), 0);
        }

        // set the bounds for ship
        ob::RealVectorBounds shipBounds(2);
        shipBounds.setLow(0);
        shipBounds.setHigh(SIDELENGTH);

        space->as<ob::CompoundStateSpace>()->as<ob::SE2StateSpace>(0)->setBounds(shipBounds);

        // set the bounds for koule
        ob::RealVectorBounds kouleBounds(2);
        kouleBounds.setLow(-0.1);
        kouleBounds.setHigh(SIDELENGTH+0.1);

        for (i = 0; i < remainingKoules; i++) {
            space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(2+2*i)->setBounds(kouleBounds);
        }

        // set the bounds for velocities
        ob::RealVectorBounds velBounds(2);
        velBounds.setLow(-1);
        velBounds.setHigh(1);

        space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1)->setBounds(velBounds);
        for (i = 0; i < remainingKoules; i++) {
            space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(3+2*i)->setBounds(velBounds);
        }

        // construct control space
        oc::ControlSpacePtr cspace(new oc::DiscreteControlSpace(space, 0, 3));

        // define a simple setup class
        oc::SimpleSetup ss(cspace);

        if (!isBenchmark) {
            // set planner
            const ob::PlannerPtr planner(new oc::RRT(ss.getSpaceInformation()));
            ss.setPlanner(planner);
        }
        
        // set validity checker
        ss.setStateValidityChecker(boost::bind(&isStateValid, ss.getSpaceInformation().get(), _1));

        // set state propagator
        ss.setStatePropagator(oc::StatePropagatorPtr(new KouleStatePropagator(ss.getSpaceInformation(), remainingKoules)));
        
        // setup start state
        ob::ScopedState<ob::CompoundStateSpace> start(space);
        start->as<ob::SE2StateSpace::StateType>(0)->setX(startStates[remainingKoules-1][0]);
        start->as<ob::SE2StateSpace::StateType>(0)->setY(startStates[remainingKoules-1][1]);
        start->as<ob::SE2StateSpace::StateType>(0)->setYaw(startStates[remainingKoules-1][2]);
        start->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = startStates[remainingKoules-1][3];
        start->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = startStates[remainingKoules-1][4];
        for (i = 0; i < remainingKoules; i++) {
            start->as<ob::RealVectorStateSpace::StateType>(2+2*i)->values[0] = startStates[remainingKoules-1][5+4*i];
            start->as<ob::RealVectorStateSpace::StateType>(2+2*i)->values[1] = startStates[remainingKoules-1][6+4*i];
            start->as<ob::RealVectorStateSpace::StateType>(3+2*i)->values[0] = startStates[remainingKoules-1][7+4*i];
            start->as<ob::RealVectorStateSpace::StateType>(3+2*i)->values[1] = startStates[remainingKoules-1][8+4*i];
        }
        ss.setStartState(start);

        // create goal
        KoulesGoal * goal = new KoulesGoal(ss.getSpaceInformation());
        goal->setThreshold(0.01);
        ss.setGoal(ob::GoalPtr(goal));
        
        // set propagation step size
        ss.getSpaceInformation()->setPropagationStepSize(0.01); // TODO: What is the proper step size?

        if (!isBenchmark) {
        
            // attempt to solve the problem within 600 seconds
            ob::PlannerStatus solved = ss.solve(600.0);

            if (solved && ss.haveExactSolutionPath())
            {
                std::cout << "Found solution: ";
                
                // print the path to screen
                og::PathGeometric path = ss.getSolutionPath().asGeometric();
                std::vector<ob::State *> pathStates = path.getStates();
                std::cout << "Path with " << pathStates.size() << " states." << std::endl;
                const ob::SE2StateSpace::StateType *shipState;
                const ob::RealVectorStateSpace::StateType *shipVel;
                const ob::RealVectorStateSpace::StateType *kouleState;
                const ob::RealVectorStateSpace::StateType *kouleVel;
                for (i = 0; i < pathStates.size(); i++) {
                    shipState = pathStates[i]->as<ob::CompoundStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0);
                    shipVel = pathStates[i]->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1);
                    std::cout << shipState->getX() << " " << shipState->getY() << " " << shipState->getYaw() << " " << shipVel->values[0] << " " << shipVel->values[1];
                    for (j = 0; j < remainingKoules; j++) {
                        kouleState = pathStates[i]->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(2+2*j);
                        kouleVel = pathStates[i]->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(3+2*j);
                        std::cout << " " << kouleState->values[0] << " " << kouleState->values[1] << " " << kouleVel->values[0] << " " << kouleVel->values[1];
                    }
                    std::cout << std::endl;
                }
                
                // update counters
                remainingKoules--;
                remainingTrials = NUM_ATTEMPTS;
                if (remainingKoules == 0)
                    return;
                    
                // save final state as the start state of next time
                ob::State *finalState = pathStates[pathStates.size()-1];
                shipState = finalState->as<ob::CompoundStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0);
                startStates[remainingKoules-1][0] = shipState->getX();
                startStates[remainingKoules-1][1] = shipState->getY();
                startStates[remainingKoules-1][2] = shipState->getYaw();
                shipVel = finalState->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1);
                startStates[remainingKoules-1][3] = shipVel->values[0];
                startStates[remainingKoules-1][4] = shipVel->values[1];
                j = 5;
                for (i = 0; i <= remainingKoules; i++) {
                    kouleState = finalState->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(2+2*i);
                    kouleVel = finalState->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(3+2*i);
                    if (kouleState->values[0] <= 0.01 || kouleState->values[0] >= SIDELENGTH-0.01 ||
                        kouleState->values[1] <= 0.01 || kouleState->values[1] >= SIDELENGTH-0.01)
                        continue;
                    startStates[remainingKoules-1][j++] = kouleState->values[0];
                    startStates[remainingKoules-1][j++] = kouleState->values[1];
                    startStates[remainingKoules-1][j++] = kouleVel->values[0];
                    startStates[remainingKoules-1][j++] = kouleVel->values[1];
                }
                ss.clear();
            }  else  {
                std::cout << "No exact solution found." << std::endl;
            }
        
        } else {
        
            // Create a benchmark class
            ompl::tools::Benchmark b(ss, "Koule experiment");
            
            // Add the planners to evaluate
            b.addPlanner(ob::PlannerPtr(new oc::RRT(ss.getSpaceInformation())));

            ob::ProjectionEvaluatorPtr proj = ob::ProjectionEvaluatorPtr(new KouleProjection(space));
            proj->setup();
            space->registerDefaultProjection(proj);

            b.addPlanner(ob::PlannerPtr(new oc::KPIECE1(ss.getSpaceInformation())));
            b.addPlanner(ob::PlannerPtr(new oc::EST(ss.getSpaceInformation())));

            // Start benchmark
            ompl::tools::Benchmark::Request req;
            req.maxTime = 600.0;
            req.maxMem = 10000.0;
            req.runCount = 10;
            req.displayProgress = true;
            b.benchmark(req);
            
            // This will generate a file of the form ompl_host_time.log
            b.saveResultsToFile();
            
            return;
        }
    }
}

int main(int argc, char **argv)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    int isBenchmark = atoi(argv[1]);
    planWithSimpleSetup(isBenchmark);

    return 0;
}

