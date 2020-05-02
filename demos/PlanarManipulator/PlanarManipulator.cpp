/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Rice University
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
 *   * Neither the name of Rice University nor the names of its
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

#include <iostream>
#include "PlanarManipulator.h"
#include <boost/math/constants/constants.hpp>
#include <stdexcept>

#define PI boost::math::constants::pi<double>()
#define TWOPI boost::math::constants::two_pi<double>()

PlanarManipulator::PlanarManipulator(unsigned int numLinks, double linkLength, const std::pair<double, double> &origin)
  : numLinks_(numLinks), linkLengths_(numLinks_, linkLength)
{
    baseFrame_ = Eigen::Affine2d::Identity();
    baseFrame_.translation()[0] = origin.first;
    baseFrame_.translation()[1] = origin.second;

    hasBounds_.assign(numLinks_, false);
    lowerBounds_.assign(numLinks_, std::numeric_limits<double>::min());
    upperBounds_.assign(numLinks_, std::numeric_limits<double>::max());
}

PlanarManipulator::PlanarManipulator(unsigned int numLinks, const std::vector<double> &linkLengths,
                                     const std::pair<double, double> &origin)
  : numLinks_(numLinks), linkLengths_(linkLengths)
{
    if (linkLengths_.size() != numLinks)
    {
        std::cerr << "Length of linkLengths (" << linkLengths.size() << ") is not equal to the number of links ("
                  << numLinks << ")" << std::endl;
        throw;
    }

    baseFrame_ = Eigen::Affine2d::Identity();
    baseFrame_.translation()[0] = origin.first;
    baseFrame_.translation()[1] = origin.second;

    hasBounds_.assign(numLinks_, false);
    lowerBounds_.assign(numLinks_, std::numeric_limits<double>::min());
    upperBounds_.assign(numLinks_, std::numeric_limits<double>::max());
}

PlanarManipulator::~PlanarManipulator()
{
}

// Return the number of links
unsigned int PlanarManipulator::getNumLinks() const
{
    return numLinks_;
}

const Eigen::Affine2d &PlanarManipulator::getBaseFrame() const
{
    return baseFrame_;
}

void PlanarManipulator::setBaseFrame(const Eigen::Affine2d &frame)
{
    baseFrame_ = frame;
}

// Return the length of each link
const std::vector<double> &PlanarManipulator::getLinkLengths() const
{
    return linkLengths_;
}

// Set the bounds for link # link
void PlanarManipulator::setBounds(unsigned int link, double low, double high)
{
    assert(link < hasBounds_.size());

    hasBounds_[link] = true;
    lowerBounds_[link] = low;
    upperBounds_[link] = high;
}

// Set the bounds for ALL links
void PlanarManipulator::setBounds(const std::vector<double> &low, const std::vector<double> &high)
{
    assert(low.size() == high.size());
    assert(low.size() == numLinks_);

    hasBounds_.assign(numLinks_, true);
    lowerBounds_ = low;
    upperBounds_ = high;
}

bool PlanarManipulator::hasBounds(unsigned int link) const
{
    return hasBounds_[link];
}

void PlanarManipulator::getBounds(unsigned int link, double &low, double &high) const
{
    low = lowerBounds_[link];
    high = upperBounds_[link];
}
const std::vector<double> &PlanarManipulator::lowerBounds() const
{
    return lowerBounds_;
}

const std::vector<double> &PlanarManipulator::upperBounds() const
{
    return upperBounds_;
}

// Forward kinematics for the given joint configuration.  The frames for links
// 1 through the end-effector are returned.
void PlanarManipulator::FK(const std::vector<double> &joints, std::vector<Eigen::Affine2d> &frames) const
{
    FK(&joints[0], frames);
}

void PlanarManipulator::FK(const Eigen::VectorXd &joints, std::vector<Eigen::Affine2d> &frames) const
{
    FK(&joints(0), frames);
}

void PlanarManipulator::FK(const double *joints, std::vector<Eigen::Affine2d> &frames) const
{
    frames.clear();
    Eigen::Affine2d frame(baseFrame_);

    for (unsigned int i = 0; i < numLinks_; ++i)
    {
        // Rotate, then translate.  Just like the old gypsy woman said.
        Eigen::Affine2d offset(Eigen::Rotation2Dd(joints[i]) * Eigen::Translation2d(linkLengths_[i], 0));
        frame = frame * offset;
        frames.push_back(frame);
    }
}

void PlanarManipulator::FK(const std::vector<double> &joints, Eigen::Affine2d &eeFrame) const
{
    FK(&joints[0], eeFrame);
}

void PlanarManipulator::FK(const Eigen::VectorXd &joints, Eigen::Affine2d &eeFrame) const
{
    FK(&joints(0), eeFrame);
}

void PlanarManipulator::FK(const double *joints, Eigen::Affine2d &eeFrame) const
{
    eeFrame = baseFrame_;

    for (unsigned int i = 0; i < numLinks_; ++i)
    {
        // Rotate, then translate.  Just like the old gypsy woman said.
        Eigen::Affine2d offset(Eigen::Rotation2Dd(joints[i]) * Eigen::Translation2d(linkLengths_[i], 0));
        eeFrame = eeFrame * offset;
    }
}

// Inverse kinematics for the given end effector frame.  Only one solution is returned.
// Returns false if no solution exists to the given pose.
bool PlanarManipulator::IK(std::vector<double> &solution, const Eigen::Affine2d &eeFrame) const
{
    std::vector<double> seed(numLinks_, M_PI / 2.0);
    return IK(solution, seed, eeFrame);
}

bool PlanarManipulator::IK(std::vector<double> &solution, const std::vector<double> &seed,
                           const Eigen::Affine2d &desiredFrame) const
{
    // desired frame is clearly impossible to achieve
    if (infeasible(desiredFrame))
        return false;

    // This is the orientation for the end effector
    double angle = acos(desiredFrame.rotation()(0, 0));
    // Due to numerical instability, sometimes acos will return nan if
    // the value is slightly larger than one.  Check for this and try the
    // asin of the next value
    if (angle != angle)  // a nan is never equal to itself
        angle = asin(desiredFrame.rotation()(0, 1));

    // If still nan, return false
    if (angle != angle)
        return false;

    // Get the current pose
    Eigen::Affine2d frame;
    FK(seed, frame);
    Eigen::VectorXd current;
    frameToPose(frame, current);

    // Get the desired pose
    Eigen::VectorXd desired;
    frameToPose(desiredFrame, desired);

    // Compute the error
    Eigen::VectorXd e(desired - current);

    Eigen::VectorXd joints(seed.size());
    for (size_t i = 0; i < seed.size(); ++i)
        joints(i) = seed[i];

    double alpha = 0.1;  // step size

    unsigned int iter = 1;
    double eps = 1e-6;

    Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(3, numLinks_);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(3, numLinks_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(numLinks_, 3);

    // Iterate the jacobian
    while (e.norm() > eps)
    {
        // Get jacobian for this joint configuration
        Jacobian(joints, jac);

        // Compute inverse of the jacobian
        // Really unstable
        // Eigen::MatrixXd jac_inv = ((jac.transpose() * jac).inverse()) * jac.transpose();

        // Moore-Penrose Pseudoinverse
        svd.compute(jac);
        const Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType &d = svd.singularValues();

        // "Invert" the singular value matrix
        for (int i = 0; i < d.size(); ++i)
            if (d(i) > eps)
                D(i, i) = 1.0 / d(i);
            else
                D(i, i) = 0.0;

        // Inverse is V*D^-1*U^t
        Eigen::MatrixXd jac_inv = svd.matrixV() * D * svd.matrixU().transpose();

        // Get the joint difference
        Eigen::VectorXd delta_theta = jac_inv * e;

        // Check for failure
        if (delta_theta(0) != delta_theta(0))  // nan
        {
            // std::cout << jac_inv.matrix() << std::endl;
            return false;
        }

        // Increment the current joints by a (small) multiple of the difference
        joints = joints + (alpha * delta_theta);

        // Figure out the current EE pose and update e.
        FK(joints, frame);
        frameToPose(frame, current);
        // double n = e.norm();
        e = desired - current;
        iter++;

        if (iter > 5000)
            return false;  // diverge
    }

    // Store the solution.  Make sure angles are in range [-pi, pi]
    solution.resize(numLinks_);
    for (size_t i = 0; i < solution.size(); ++i)
    {
        double angle = joints(i);
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        solution[i] = angle;
    }

    return true;
}

bool PlanarManipulator::FABRIK(std::vector<double> &solution, const Eigen::Affine2d &eeFrame, double xyTol,
                               double thetaTol) const
{
    std::vector<double> seed(numLinks_, M_PI / 2.0);
    return FABRIK(solution, seed, eeFrame, xyTol, thetaTol);
}

bool PlanarManipulator::FABRIK(std::vector<double> &solution, const std::vector<double> &seed,
                               const Eigen::Affine2d &desiredFrame, double xyTol, double thetaTol) const
{
    unsigned int numLinks = getNumLinks();  // have to use this local variable.  Compiler error (bug?) in
                                            // Eigen::Translation2d when I use numLinks_

    if (seed.size() != numLinks)
    {
        std::cerr << "Seed has length " << seed.size() << " but there are " << numLinks << " links" << std::endl;
        return false;
    }

    // desired frame is clearly impossible to achieve
    if (infeasible(desiredFrame))
        return false;

    // copy the seed into the solution
    solution.assign(seed.begin(), seed.end());

    // Compute the location of the origin of the last link with the correct rotation
    // This location will not move
    Eigen::Vector2d loc(-linkLengths_.back(), 0);
    loc = desiredFrame * loc;

    // The desired orientation of the end effector
    double v = desiredFrame.matrix()(0, 0);
    // Go Go Gadget Numerical Stabilizer!
    if (v < -1.0)
        v = -1.0;
    if (v > 1.0)
        v = 1.0;
    double eeTheta = acos(v);
    if (desiredFrame.matrix()(1, 0) < 0)  // sin is negative, so the angle is negative
        eeTheta = -eeTheta;

    // The location of each joint, starting with the origin and ending with the end effector
    // For an n-joint chain, jointPositions with have n+1 entries.
    std::vector<Eigen::Vector2d> jointPositions;
    jointPositions.push_back(baseFrame_.translation());  // root position

    // Compute the initial locations of each joint
    Eigen::Affine2d frame(baseFrame_);
    for (unsigned int i = 0; i < seed.size(); ++i)
    {
        Eigen::Affine2d offset(Eigen::Rotation2Dd(seed[i]) * Eigen::Translation2d(linkLengths_[i], 0));
        frame = frame * offset;
        jointPositions.push_back(frame.translation());
    }

    // Store the errors here
    double xyError = std::numeric_limits<double>::max();
    double thetaError = std::numeric_limits<double>::max();

    int numIterations = 0;
    int maxIter = 250;  // after this many iterations, declare failure.  This is probably WAY too many - tends to
                        // converge very fast, after 1-3 iterations
    while ((xyError > xyTol || thetaError > thetaTol) && numIterations++ < maxIter)
    {
        xyError = thetaError = 0.0;

        // Placing last link where it must go
        jointPositions[numLinks] = desiredFrame.translation();  // goal position, nailed it.
        jointPositions[numLinks - 1] = loc;                     // goal orientation, nailed it.

        // A reverse coordinate frame, working backward along the chain.  Used to resolve joint limits in backward phase
        // VERY IMPORTANT to translate first, then rotate.  This NOT like the old gypsy woman said.  I'm getting my
        // money back.
        Eigen::Affine2d backwardFrame(Eigen::Translation2d(jointPositions[numLinks - 1]) *
                                      Eigen::Rotation2Dd(PI + eeTheta));  // end of chain, looking down the chain

        // Move backward from the end effector toward the base
        for (int i = jointPositions.size() - 3; i >= 0;
             --i)  // -1 is end effector position.  Skip -2 to preserve the end effector orientation.
        {
            // draw a line between position i+1 and i, and place pt i at the kinematically feasible place along the line
            double t = linkLengths_[i] / (jointPositions[i + 1] - jointPositions[i]).norm();
            jointPositions[i] = (1 - t) * jointPositions[i + 1] + t * jointPositions[i];

            // Ensure the angle for joint i is within bounds.  If not, move the joint position
            Eigen::Vector2d vec = backwardFrame.inverse() * jointPositions[i];
            double angle = -atan2(vec(1), vec(0));  // The angle computed is the negative of the real angle (because we
                                                    // worked backward).

            // If not in bounds, the angle will be one of the joint limits
            if (hasBounds_[i] && (angle < lowerBounds_[i] || angle > upperBounds_[i]))
            {
                double dlow = fabs(angle - lowerBounds_[i]);
                double dhigh = fabs(angle - upperBounds_[i]);
                angle = (dlow < dhigh ? lowerBounds_[i] : upperBounds_[i]);
            }

            Eigen::Affine2d offset(Eigen::Rotation2Dd(-angle) * Eigen::Translation2d(linkLengths_[i], 0));
            backwardFrame = backwardFrame * offset;
            jointPositions[i] = backwardFrame.translation();
        }

        jointPositions[0] = baseFrame_.translation();  // move base back to where it is supposed to be
        Eigen::Affine2d forwardFrame(baseFrame_);

        // Move forward toward the end effector
        for (size_t i = 0; i < jointPositions.size() - 1;
             ++i)  // This pass moves everything except end effector.  Orientation at the end may be violated, but we
                   // check for this
        {
            // draw a line between position i+1 and i and place i+1 at the kinematically feasible place along the line
            double t = linkLengths_[i] / (jointPositions[i + 1] - jointPositions[i]).norm();
            jointPositions[i + 1] = (1 - t) * jointPositions[i] + t * jointPositions[i + 1];

            // Figure out the joint angle required for this
            Eigen::Vector2d vec = forwardFrame.inverse() * jointPositions[i + 1];
            double angle = atan2(vec(1), vec(0));

            // If not in bounds, the angle will be one of the joint limits
            if (hasBounds_[i] && (angle < lowerBounds_[i] || angle > upperBounds_[i]))
            {
                double dlow = fabs(angle - lowerBounds_[i]);
                double dhigh = fabs(angle - upperBounds_[i]);
                angle = (dlow < dhigh ? lowerBounds_[i] : upperBounds_[i]);
            }
            solution[i] = angle;

            // Update frame
            Eigen::Affine2d offset(Eigen::Rotation2Dd(angle) * Eigen::Translation2d(linkLengths_[i], 0));
            forwardFrame = forwardFrame * offset;
            jointPositions[i + 1] = forwardFrame.translation();  // joint location is the translation of the frame.  Do
                                                                 // this here in case joint limits changed the angle
        }

        // Compute translation and orientation error
        xyError = (jointPositions.back() - desiredFrame.translation()).norm();

        v = forwardFrame.matrix()(0, 0);
        // Go Go Gadget Numerical Stabilizer!
        if (v > 1.0)
            v = 1.0;
        if (v < -1.0)
            v = -1.0;

        // The real angle
        double thetaActual = acos(v);
        if (forwardFrame.matrix()(1, 0) < 0)  // sin is negative, so the angle is negative
            thetaActual = -thetaActual;
        thetaError = fabs(eeTheta - thetaActual);
    }

    // Winning.  Extract joint angles.  Oh wait.  Already done BOOM.
    if (xyError < xyTol && thetaError < thetaTol)
        return true;

    return false;
}

void PlanarManipulator::Jacobian(const double *joints, Eigen::MatrixXd &jac) const
{
    if (jac.rows() != 3 || jac.cols() != numLinks_)
        jac = Eigen::MatrixXd::Zero(3, numLinks_);

    std::vector<double> sins(numLinks_);
    std::vector<double> coss(numLinks_);
    double theta = 0.0;
    for (size_t i = 0; i < numLinks_; ++i)
    {
        theta += joints[i];
        sins[i] = sin(theta);
        coss[i] = cos(theta);
    }

    for (size_t i = 0; i < numLinks_; ++i)
    {
        double entry1 = 0.0;
        double entry2 = 0.0;
        for (size_t j = numLinks_; j > i; --j)
        {
            entry1 += -(linkLengths_[j - 1] * sins[j - 1]);
            entry2 += linkLengths_[j - 1] * coss[j - 1];
        }
        jac(0, i) = entry1;
        jac(1, i) = entry2;
        jac(2, i) = 1.0;
    }
}

// Return the Jacobian for the manipulator at the given joint state.
void PlanarManipulator::Jacobian(const std::vector<double> &joints, Eigen::MatrixXd &jac) const
{
    Jacobian(&joints[0], jac);
}

void PlanarManipulator::Jacobian(const Eigen::VectorXd &joints, Eigen::MatrixXd &jac) const
{
    Jacobian(&joints(0), jac);  // TODO: This is untested.  I dunno if an eigen::vector is contiguous.  It probably is.
}

void PlanarManipulator::frameToPose(const Eigen::Affine2d &frame, Eigen::VectorXd &pose)
{
    pose = Eigen::VectorXd(3);
    pose(0) = frame.translation()(0);
    pose(1) = frame.translation()(1);
    pose(2) = acos(frame.matrix()(0, 0));
}

bool PlanarManipulator::infeasible(const Eigen::Affine2d &frame) const
{
    // Check for impossible query
    // Compute the location of the origin of the last link with the correct rotation
    Eigen::Vector2d loc(-linkLengths_[numLinks_ - 1], 0);
    loc = frame * loc;

    // Length of the chain, except for the last link
    double len = 0.0;
    for (size_t i = 0; i < numLinks_ - 1; ++i)
        len += linkLengths_[i];

    // Make sure the chain (without the last link) is long enough to reach the
    // origin of the last link
    Eigen::Vector2d org(baseFrame_.translation());
    if ((org - loc).norm() > len)  // infeasible IK request
        return true;

    return false;
}
