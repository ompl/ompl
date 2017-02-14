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

#define BOOST_TEST_MODULE "KinematicConstraints"
#include <boost/bind.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/util/RandomNumbers.h>

#include <ompl/geometric/constraints/KinematicConstraint.h>

#include <eigen3/Eigen/Dense>

namespace ob = ompl::base;
namespace og = ompl::geometric;

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

static void setState(ob::State *state, const std::vector<double> angles)
{
    ob::CompoundState *cstate = state->as<ob::CompoundStateSpace::StateType>();
    for (size_t i = 0; i < angles.size(); ++i)
    {
        cstate->as<ob::SO2StateSpace::StateType>(i)->value = normalizeAngle(angles[i]);
    }
}

// Inverse kinematics for 3-link planar manipulator with uniformly spaced joints
bool inverseKinematics(ob::State *state, const std::map<unsigned int, Eigen::Affine3d> &poses, unsigned int numLinks,
                       double linkLength)
{
    if (numLinks != 3)
        throw ompl::Exception("IK solution only works for 3 link manipulator");

    // Only computing IK for end effector
    auto pose_it = poses.find(numLinks);
    if (pose_it == poses.end())
        throw ompl::Exception("No pose for end effector specified");

    const Eigen::Affine3d &pose = pose_it->second;

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
    Eigen::Affine3d eeTransform(Eigen::Translation3d(linkLength, 0, 0));
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

BOOST_AUTO_TEST_CASE(ForwardKinematics)
{
    unsigned int numLinks = 3;
    double linkLength = 1.0;
    std::pair<double, double> origin = std::make_pair(0.0, 0.0);

    ob::StateSpacePtr space(new ob::CompoundStateSpace());
    for (unsigned int i = 0; i < numLinks; ++i)
    {
        ob::StateSpacePtr subspace(new ob::SO2StateSpace());
        space->as<ob::CompoundStateSpace>()->addSubspace(subspace, 1.0);
    }
    space->setup();
    ob::State *state = space->allocState();
    {
        // Zero for all joint angles
        std::vector<double> angles(numLinks, 0.0);
        setState(state, angles);

        std::vector<Eigen::Affine3d> frames;

        forwardKinematics(state, frames, numLinks, linkLength, origin);
        BOOST_REQUIRE(frames.size() == numLinks + 1);

        // CHECKING POSITIONS
        // Expecting (1,0,0) - first link
        Eigen::Vector3d trans1(frames[1].translation());
        BOOST_CHECK(fabs(1.0 - trans1[0]) < 1e-6);
        BOOST_CHECK(fabs(trans1[1]) < 1e-6);
        BOOST_CHECK(fabs(trans1[2]) < 1e-6);

        // Expecting (2,0,0) - second link
        Eigen::Vector3d trans2(frames[2].translation());
        BOOST_CHECK(fabs(2.0 - trans2[0]) < 1e-6);
        BOOST_CHECK(fabs(trans2[1]) < 1e-6);
        BOOST_CHECK(fabs(trans2[2]) < 1e-6);

        // Expecting (3,0,0) - end effector
        Eigen::Vector3d trans3(frames[3].translation());
        BOOST_CHECK(fabs(3.0 - trans3[0]) < 1e-6);
        BOOST_CHECK(fabs(trans3[1]) < 1e-6);
        BOOST_CHECK(fabs(trans3[2]) < 1e-6);

        // Checking Orientations
        // All should be zero
        for (auto & frame : frames)
        {
            Eigen::Vector3d rpy = frame.rotation().eulerAngles(0, 1, 2);  // rpy, in that order
            for (unsigned int j = 0; j < 3; ++j)
                BOOST_CHECK(fabs(rpy(j)) < 1e-6);
        }
    }
    {
        // Pi/2 for first angle, zero for rest
        std::vector<double> angles(numLinks, 0.0);
        angles[0] = boost::math::constants::pi<double>() / 2.0;  // Setting first link to pi/2
        setState(state, angles);

        std::vector<Eigen::Affine3d> frames;

        forwardKinematics(state, frames, numLinks, linkLength, origin);
        BOOST_REQUIRE(frames.size() == numLinks + 1);

        // CHECKING POSITIONS
        // Expecting (0,1,0) - first link
        Eigen::Vector3d trans1(frames[1].translation());
        BOOST_CHECK(fabs(trans1[0]) < 1e-6);
        BOOST_CHECK(fabs(1.0 - trans1[1]) < 1e-6);
        BOOST_CHECK(fabs(trans1[2]) < 1e-6);

        // Expecting (0,2,0) - second link
        Eigen::Vector3d trans2(frames[2].translation());
        BOOST_CHECK(fabs(trans2[0]) < 1e-6);
        BOOST_CHECK(fabs(2.0 - trans2[1]) < 1e-6);
        BOOST_CHECK(fabs(trans2[2]) < 1e-6);

        // Expecting (0,3,0) - end effector
        Eigen::Vector3d trans3(frames[3].translation());
        BOOST_CHECK(fabs(trans3[0]) < 1e-6);
        BOOST_CHECK(fabs(3.0 - trans3[1]) < 1e-6);
        BOOST_CHECK(fabs(trans3[2]) < 1e-6);

        // Checking Orientations
        // Yaw should be 90, others should be zero
        for (size_t i = 1; i < frames.size(); ++i)
        {
            Eigen::Vector3d rpy = frames[i].rotation().eulerAngles(0, 1, 2);  // rpy, in that order
            for (unsigned int j = 0; j < 2; ++j)
                BOOST_CHECK(fabs(rpy(j)) < 1e-6);
            BOOST_CHECK(fabs((boost::math::constants::pi<double>() / 2.0) - fabs(rpy(2))) < 1e-6);
        }
    }
    {
        // Pi/2 for first and second angle, zero for third
        std::vector<double> angles(numLinks, 0.0);
        angles[0] = boost::math::constants::pi<double>() / 2.0;  // Setting first link to pi/2
        angles[1] = boost::math::constants::pi<double>() / 2.0;  // Setting second link to pi/2
        setState(state, angles);

        std::vector<Eigen::Affine3d> frames;

        forwardKinematics(state, frames, numLinks, linkLength, origin);
        BOOST_REQUIRE(frames.size() == numLinks + 1);

        // CHECKING POSITIONS
        // Expecting (0,1,0) - first link
        Eigen::Vector3d trans1(frames[1].translation());
        BOOST_CHECK(fabs(trans1[0]) < 1e-6);
        BOOST_CHECK(fabs(1.0 - trans1[1]) < 1e-6);
        BOOST_CHECK(fabs(trans1[2]) < 1e-6);

        // Expecting (-1,1,0) - second link
        Eigen::Vector3d trans2(frames[2].translation());
        BOOST_CHECK(fabs(-1.0 - trans2[0]) < 1e-6);
        BOOST_CHECK(fabs(1.0 - trans2[1]) < 1e-6);
        BOOST_CHECK(fabs(trans2[2]) < 1e-6);

        // Expecting (-2,1,0) - end effector
        Eigen::Vector3d trans3(frames[3].translation());
        BOOST_CHECK(fabs(-2.0 - trans3[0]) < 1e-6);
        BOOST_CHECK(fabs(1.0 - trans3[1]) < 1e-6);
        BOOST_CHECK(fabs(trans3[2]) < 1e-6);

        // Checking Orientations
        // Everything but yaw should be zero
        for (size_t i = 1; i < frames.size(); ++i)
        {
            Eigen::Vector3d rpy = frames[i].rotation().eulerAngles(0, 1, 2);  // rpy, in that order
            for (unsigned int j = 0; j < 2; ++j)
                BOOST_CHECK(fabs(rpy(j)) < 1e-6);

            // First link is at pi/2
            if (i == 1)
                BOOST_CHECK(fabs((boost::math::constants::pi<double>() / 2.0) - fabs(rpy(2))) < 1e-6);
            // Other links should be at pi
            else
                BOOST_CHECK(fabs((boost::math::constants::pi<double>()) - fabs(rpy(2))) < 1e-6);
        }
    }
    space->freeState(state);
}

BOOST_AUTO_TEST_CASE(InverseKinematics)
{
    unsigned int numLinks = 3;
    double linkLength = 1.0;
    std::pair<double, double> origin = std::make_pair(0.0, 0.0);

    ob::StateSpacePtr space(new ob::CompoundStateSpace());
    for (unsigned int i = 0; i < numLinks; ++i)
    {
        ob::StateSpacePtr subspace(new ob::SO2StateSpace());
        space->as<ob::CompoundStateSpace>()->addSubspace(subspace, 1.0);
    }
    space->setup();

    ob::State *state = space->allocState();
    {
        std::map<unsigned int, Eigen::Affine3d> poses;

        // Find angles to put end effector at (0,1) with no rotation
        Eigen::Affine3d eePose(Eigen::Translation3d(0, 1, 0));
        poses[numLinks] = eePose;

        BOOST_REQUIRE(inverseKinematics(state, poses, numLinks, linkLength));

        // Verify IK with FK
        std::vector<Eigen::Affine3d> frames;
        forwardKinematics(state, frames, numLinks, linkLength, origin);

        Eigen::Vector3d trans(frames.back().translation());
        Eigen::Vector3d rpy = frames.back().rotation().eulerAngles(0, 1, 2);  // rpy, in that order

        BOOST_CHECK(fabs(trans[0]) < 1e-6);
        BOOST_CHECK(fabs(1.0 - trans[1]) < 1e-6);
        BOOST_CHECK(fabs(trans[2]) < 1e-6);

        for (size_t i = 0; i < 3; ++i)
            BOOST_CHECK(fabs(rpy[0]) < 1e-6);
    }
    {
        std::map<unsigned int, Eigen::Affine3d> poses;

        // Find angles to put end effector at (0,-1) with no rotation
        Eigen::Affine3d eePose(Eigen::Translation3d(0, -1, 0));
        poses[numLinks] = eePose;

        BOOST_REQUIRE(inverseKinematics(state, poses, numLinks, linkLength));

        // Verify IK with FK
        std::vector<Eigen::Affine3d> frames;
        forwardKinematics(state, frames, numLinks, linkLength, origin);

        Eigen::Vector3d trans(frames.back().translation());
        Eigen::Vector3d rpy = frames.back().rotation().eulerAngles(0, 1, 2);  // rpy, in that order

        BOOST_CHECK(fabs(trans[0]) < 1e-6);
        BOOST_CHECK(fabs(-1.0 - trans[1]) < 1e-6);
        BOOST_CHECK(fabs(trans[2]) < 1e-6);

        for (size_t i = 0; i < 3; ++i)
            BOOST_CHECK(fabs(rpy[0]) < 1e-6);
    }

    space->freeState(state);
}

BOOST_AUTO_TEST_CASE(KinematicConstraint)
{
    // Creating a state space for a numLink kinematic chain
    // with unit spaced revolute joints
    unsigned int numLinks = 3;
    double linkLength = 1.0;
    std::pair<double, double> origin = std::make_pair(0.0, 0.0);

    ob::StateSpacePtr space(new ob::CompoundStateSpace());
    for (unsigned int i = 0; i < numLinks; ++i)
    {
        ob::StateSpacePtr subspace(new ob::SO2StateSpace());
        space->as<ob::CompoundStateSpace>()->addSubspace(subspace, 1.0);
    }
    space->setup();

    ob::StateSpace::SubstateLocation loc;
    loc.chain.clear();
    loc.space = space.get();

    // Creating kinematic constraint
    og::KinematicConstraint constraint(space, loc, numLinks,
                                       boost::bind(forwardKinematics, _1, _2, numLinks, linkLength, origin),
                                       boost::bind(inverseKinematics, _1, _2, numLinks, linkLength));

    ompl::RNG rng;
    ob::State *state = space->allocState();
    ob::StateSamplerPtr stateSampler = space->allocStateSampler();

    unsigned int samples = 1000;
    for (unsigned int i = 0; i < samples; ++i)
    {
        double t = rng.uniform01() * boost::math::constants::two_pi<double>();
        double u = rng.uniform01() + rng.uniform01();
        double r = (u > 1.0 ? 2.0 - u : u);

        // Sampling an end effector position uniformly in the reachable area of the chain
        double x = r * cos(t);  // * numLinks;
        double y = r * sin(t);  // * numLinks;

        // frame #, x, y, z, roll, pitch, yaw
        constraint.setPoseConstraint(numLinks, x, y, 0, 0, 0, 0);

        // Sample a random state
        stateSampler->sampleUniform(state);

        // Random state satisfies constraints?  Not very likely
        BOOST_CHECK(!constraint.isSatisfied(state));

        // Project the state onto constraints.  Better work now
        while (!constraint.project(state))
            ;
        BOOST_CHECK(constraint.isSatisfied(state));
    }

    space->freeState(state);
}
