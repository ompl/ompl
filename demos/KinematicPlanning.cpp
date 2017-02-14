/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

#include <fstream>

#include <boost/bind.hpp>
#include <boost/math/constants/constants.hpp>

#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/geometric/ConstrainedSimpleSetup.h>

#include <ompl/base/goals/GoalLazySamples.h>

#include <ompl/base/ConstraintInformation.h>
#include <ompl/geometric/constraints/EndEffectorConstraint.h>
#include <ompl/geometric/planners/rrt/CBiRRT2.h>
#include <ompl/geometric/planners/rrt/ConstrainedRRT.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isValid(const ob::State *state)
{
    return true;
}

// Forward kinematics for n-link planar manipulator with uniformly spaced joints
// Returns global reference frame for each link AND the end effector
void forwardKinematics(const ob::State *state, std::vector<Eigen::Affine3d> &frames, unsigned int numLinks,
                       unsigned int linkLength, std::pair<double, double> origin)
{
    // Getting all joint angles
    const ob::CompoundStateSpace::StateType *cstate = state->as<ob::CompoundStateSpace::StateType>();
    std::vector<double> angles;
    for (unsigned int i = 0; i < numLinks; ++i)
        angles.push_back(cstate->as<ob::SO2StateSpace::StateType>(i)->value);

    // Transform to take us to the next link wrt previous link frame
    Eigen::Affine3d linkOffset(Eigen::Translation3d(linkLength, 0, 0));

    // The base frame rooted at the origin.  No rotation
    Eigen::Affine3d frame(Eigen::Translation3d(origin.first, origin.second, 0.0));
    frames.push_back(frame);

    for (unsigned int i = 0; i < numLinks; ++i)
    {
        // Apply the joint angle rotation to the previous frame
        Eigen::Affine3d rotation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                                 Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(angles[i], Eigen::Vector3d::UnitZ()));

        frame = frame * rotation * linkOffset;
        frames.push_back(frame);
    }
}

// Return angle in range (-pi, pi]
static double normalizeAngle(double angle)
{
    double v = fmod(angle, 2.0 * boost::math::constants::pi<double>());
    if (v <= -boost::math::constants::pi<double>())
        v += 2.0 * boost::math::constants::pi<double>();
    else if (v > boost::math::constants::pi<double>())
        v -= 2.0 * boost::math::constants::pi<double>();

    return v;
}

// Set the compound SO2 state given a list of joint angles
static void setState(ob::State *state, const std::vector<double> angles)
{
    ob::CompoundState *cstate = state->as<ob::CompoundStateSpace::StateType>();
    for (size_t i = 0; i < angles.size(); ++i)
    {
        cstate->as<ob::SO2StateSpace::StateType>(i)->value = normalizeAngle(angles[i]);
    }
}

// Inverse kinematics for 3-link planar manipulator with uniformly spaced joints
bool inverseKinematics(ob::State *state, const Eigen::Affine3d &pose, unsigned int numLinks, double linkLength)
{
    if (numLinks != 3)
        throw ompl::Exception("IK solution only works for 3 link manipulator");

    // This is the orientation for the end effector
    double angle = acos(pose.matrix()(0, 0));
    // Due to numerical instability, sometimes acos will return nan if
    // the value is slightly larger than one.  Check for this and try the
    // asin of the next value
    if (angle != angle)  // a nan is never equal to itself
        angle = asin(pose.matrix()(0, 1));

    // If still nan, return false
    if (angle != angle)
        return false;

    // This is the desired position of the end effector
    Eigen::Affine3d translation(Eigen::Translation3d(pose.translation()));
    Eigen::Affine3d rotation(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));

    Eigen::Affine3d eeFrame = translation * rotation;
    Eigen::Affine3d eeTransform(Eigen::Translation3d(1, 0, 0));
    Eigen::Affine3d frameOrigin = eeFrame * (eeTransform.inverse());

    // W is the (x,y) coord for the parent link of the end effector
    double Wx = frameOrigin.translation()[0];
    double Wy = frameOrigin.translation()[1];

    std::vector<double> angles(numLinks, 0.0);  // where we store the IK solution

    double l2 = linkLength * linkLength;

    double c2 = (Wx * Wx + Wy * Wy - l2 - l2) / (2.0 * l2);
    double s2 = sqrt(1.0 - c2 * c2);  // there is a +/- to play with here

    angles[1] = atan2(s2, c2);

    // Check for singularity/no solution.  atan2 will return nan
    if (angles[1] != angles[1])
        return false;

    double k1 = linkLength + linkLength * c2;
    double k2 = linkLength * s2;
    angles[0] = atan2(Wy, Wx) - atan2(k2, k1);
    angles[2] = angle - angles[0] - angles[1];

    // Setting the solution into state
    setState(state, angles);
    return true;
}

// A specification of a goal for a kinematic manipulator
// Uses EndEffectorConstraint class to define a pose for the
// manipulator that defines the goal.  Goal can also
// be a region.
class EndEffectorGoal : public ob::GoalLazySamples
{
public:
    EndEffectorGoal(const ob::SpaceInformationPtr &si, const ob::GoalSamplingFn &sampler,
                    og::EndEffectorConstraintPtr ee)
      : ob::GoalLazySamples(si, sampler), ee_(std::move(ee))
    {
    }

    ~EndEffectorGoal() override
    {
        // Very important.  Goal sampling thread may be running when
        // this object gets destroyed, causing a seg fault when accessing
        // the ee_ object.
        ee_.reset();
    }

    bool isSatisfied(const ob::State *state) const override
    {
        return ee_->isSatisfied(state);
    }

    double distanceGoal(const ob::State *state) const override
    {
        OMPL_WARN("%s: NOT IMPLEMENTED", __FUNCTION__);
        return std::numeric_limits<double>::max();
    }

    static bool goalStateSampler(const ob::GoalLazySamples *gls, ob::State *state)
    {
        const EndEffectorGoal *thisObj = static_cast<const EndEffectorGoal *>(gls);
        while (thisObj->ee_ && !thisObj->ee_->sample(state))
            ;
        return true;
    }

protected:
    og::EndEffectorConstraintPtr ee_;
};

int main(int argc, char **argv)
{
    // Planar kinematic chain with 3 links.  Equidistant joints.
    unsigned int numLinks = 3;
    double linkLength = 1.0;
    std::pair<double, double> origin = std::make_pair(0.0, 0.0);

    ob::StateSpacePtr space(new ob::CompoundStateSpace());
    for (unsigned int i = 0; i < numLinks; ++i)
    {
        ob::StateSpacePtr subspace(new ob::SO2StateSpace());
        space->as<ob::CompoundStateSpace>()->addSubspace(subspace, 1.0);
    }

    og::ConstrainedSimpleSetup ss(space);

    // Creating end effector constraint for the goal
    ob::StateSpace::SubstateLocation loc;
    loc.chain.clear();
    loc.space = space.get();
    og::EndEffectorConstraintPtr eeGoalConstraint(
        new og::EndEffectorConstraint(space, loc, boost::bind(forwardKinematics, _1, _2, numLinks, linkLength, origin),
                                      boost::bind(inverseKinematics, _1, _2, numLinks, linkLength)));

    // Goal is to bring end effector +/- 0.01 from the origin
    eeGoalConstraint->setPosition(origin.first, origin.second, 0.0);
    eeGoalConstraint->setPositionTolerance(0.01, 0.01, 0.0);
    // We don't care about orientation at the goal
    eeGoalConstraint->setOrientation(0, 0, 0);
    eeGoalConstraint->setOrientationTolerance(boost::math::constants::two_pi<double>(),
                                              boost::math::constants::two_pi<double>(),
                                              boost::math::constants::two_pi<double>());

    ob::GoalPtr goal(
        new EndEffectorGoal(ss.getSpaceInformation(), EndEffectorGoal::goalStateSampler, eeGoalConstraint));
    ss.setGoal(goal);

    // Constrain end effector +/- 1.0 from the origin for the entire trajectory
    og::EndEffectorConstraintPtr eeConstraint(
        new og::EndEffectorConstraint(space, loc, boost::bind(forwardKinematics, _1, _2, numLinks, linkLength, origin),
                                      boost::bind(inverseKinematics, _1, _2, numLinks, linkLength)));

    eeConstraint->setPosition(origin.first, origin.second, 0.0);
    eeConstraint->setPositionTolerance(1.0, 1.0, 0.0);
    // We don't care about orientation
    eeConstraint->setOrientation(0, 0, 0);
    eeConstraint->setOrientationTolerance(boost::math::constants::two_pi<double>(),
                                          boost::math::constants::two_pi<double>(),
                                          boost::math::constants::two_pi<double>());

    ob::ConstraintInformationPtr ci(new ob::ConstraintInformation());
    ci->addConstraint(eeConstraint);

    ss.getConstrainedSpaceInformation()->setConstraintInformation(ci);

    // StateValidityChecker evaluates constraint information
    ss.setStateValidityChecker(isValid);

    // Sample a start state that satisfies kinematic global kinematic
    // constraints, but NOT the goal.
    ob::ScopedState<> start(space);
    bool gotStart = false;
    do
    {
        if (ci->sample(start.get()))
            gotStart = !goal->isSatisfied(start.get());
    } while (!gotStart);
    ss.setStartState(start);

    // ob::PlannerPtr planner(new og::ConstrainedRRT(ss.getSpaceInformation()));
    ob::PlannerPtr planner(new og::CBiRRT2(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    ss.setup();
    // ss.print();

    // Finding a path
    if (ss.solve(1))
    {
        // Prohibit a race condition between goal sampling thread and
        // goal destructor by explicitly stopping goal sampling here
        goal->as<EndEffectorGoal>()->stopSampling();

        og::PathGeometric &pgeo = ss.getSolutionPath();
        bool good = true;
        std::cout << "Path has " << pgeo.getStateCount() << " states in it.  Verifying..." << std::endl;

        // for(size_t i = 0; i < pgeo.getStateCount(); ++i)
        //{
        //    std::vector<Eigen::Affine3d> frames;
        //    forwardKinematics(pgeo.getState(i), frames, numLinks, linkLength, origin);
        //    std::cout << "Positions at state " << i << std::endl;
        //    for(size_t j = 0; j < frames.size(); ++j)
        //        std::cout << "(" << frames[j].translation()[0] << " "
        //                         << frames[j].translation()[1] << " "
        //                         << frames[j].translation()[2] << ")" << std::endl;
        //    std::cout << std::endl;
        //}

        for (size_t i = 0; i < pgeo.getStateCount() && good; ++i)
        {
            good = ci->isSatisfied(pgeo.getState(i));
            if (!good)
                std::cerr << "[ERROR] State " << i << " does not satisfy constraints" << std::endl;
        }

        if (good && !eeGoalConstraint->isSatisfied(pgeo.getState(pgeo.getStateCount() - 1)))
        {
            good = false;
            std::cerr << "[ERROR] Final state does not satisfy goal constraint!" << std::endl;
        }

        if (good)
        {
            std::cout << "Path satisfies all kinematic constraints!" << std::endl;
            // std::ofstream fout;
            // fout.open("path.txt");
            // pgeo.printAsMatrix(fout);
            // fout.close();
        }
    }
    else
        std::cout << "NO SOLUTION FOUND" << std::endl;

    return 0;
}
