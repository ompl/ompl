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

#ifndef PLANAR_MANIPULATOR_H_
#define PLANAR_MANIPULATOR_H_

#include <vector>
#include <eigen3/Eigen/Dense>

class PlanarManipulator
{
public:
    PlanarManipulator(unsigned int numLinks, double linkLength,
                      const std::pair<double, double> &origin = std::make_pair(0, 0));

    PlanarManipulator(unsigned int numLinks, const std::vector<double> &linkLengths,
                      const std::pair<double, double> &origin = std::make_pair(0, 0));

    virtual ~PlanarManipulator();

    // Return the number of links
    unsigned int getNumLinks() const;

    // Return the base frame of the manipulator
    const Eigen::Affine2d &getBaseFrame() const;

    // Set the base frame of the manipulator
    void setBaseFrame(const Eigen::Affine2d &frame);

    // Return the length of each link
    const std::vector<double> &getLinkLengths() const;

    // Set the bounds for link # link
    void setBounds(unsigned int link, double low, double high);
    // Set the bounds for ALL links
    void setBounds(const std::vector<double> &low, const std::vector<double> &high);

    // Return true if the given link has bounds set
    bool hasBounds(unsigned int link) const;
    // Get the lower and upper joint angle bounds for this link
    void getBounds(unsigned int link, double &low, double &high) const;
    // Get all lower bound joint limits
    const std::vector<double> &lowerBounds() const;
    // Get all upper bound joint limits
    const std::vector<double> &upperBounds() const;

    // Forward kinematics for the given joint configuration.  The frames for links
    // 1 through the end-effector are returned.
    void FK(const double *joints, std::vector<Eigen::Affine2d> &frames) const;
    void FK(const std::vector<double> &joints, std::vector<Eigen::Affine2d> &frames) const;
    void FK(const Eigen::VectorXd &joints, std::vector<Eigen::Affine2d> &frames) const;
    void FK(const double *joints, Eigen::Affine2d &eeFrame) const;
    void FK(const std::vector<double> &joints, Eigen::Affine2d &eeFrame) const;
    void FK(const Eigen::VectorXd &joints, Eigen::Affine2d &eeFrame) const;

    // Inverse kinematics for the given end effector frame.  Jacobian pseudo-inverse method
    // Returns false if no solution is found to the given pose.
    // NOTE: Joint limits are not respected in this IK solver.
    bool IK(std::vector<double> &solution, const Eigen::Affine2d &eeFrame) const;
    bool IK(std::vector<double> &solution, const std::vector<double> &seed, const Eigen::Affine2d &desiredFrame) const;

    // Forward and backward reaching inverse kinematics (FABRIK)
    // DOES respect joint limits
    bool FABRIK(std::vector<double> &solution, const Eigen::Affine2d &eeFrame, double xyTol = 1e-5,
                double thetaTol = 1e-3) const;
    bool FABRIK(std::vector<double> &solution, const std::vector<double> &seed, const Eigen::Affine2d &desiredFrame,
                double xyTol = 1e-5, double thetaTol = 1e-3) const;

    // Return the Jacobian for the manipulator at the given joint state.
    void Jacobian(const std::vector<double> &joints, Eigen::MatrixXd &jac) const;
    void Jacobian(const Eigen::VectorXd &joints, Eigen::MatrixXd &jac) const;
    void Jacobian(const double *joints, Eigen::MatrixXd &jac) const;

    // Convert the given frame to x,y,theta vector
    static void frameToPose(const Eigen::Affine2d &frame, Eigen::VectorXd &pose);

    // Interpolate this manipulator using Cartesian space minimization technique
    void cartesianInterpolate(const std::vector<double> &start, const std::vector<double> &end, double t,
                              std::vector<double> &result) const;
    void cartesianInterpolate(const double *const start, const double *const end, double t, double *result) const;

    // Interpolate this manipulator using a hybrid Cartesian-Angular technique
    // Stride is the modulo value for the links included in the cartesian scheme
    // A stride=2 indicates every other link, stride=3 indicates every third link, etc.
    // NOTE: Currently assumes Euclidean state space

    //#warning "Stride default value is 3"
    void hybridCartesianInterpolate(const std::vector<double> &start, const std::vector<double> &end, double t,
                                    std::vector<double> &result, int stride = 2) const;
    void hybridCartesianInterpolate(const double *const start, const double *const end, double t, double *result,
                                    unsigned int stride = 2) const;

    // Compute the angles for the chain if the parameterization is reversed, from end to beginning.
    // The root of the reversed angles is assumed to be the frame of the end effector
    void reverseChainAngles(const std::vector<double> &angles, std::vector<double> &reversed) const;

protected:
    bool infeasible(const Eigen::Affine2d &frame) const;

    // The number of links in the chain
    unsigned int numLinks_;
    // The length of each link
    std::vector<double> linkLengths_;
    // Optional bounds on the joint angles.  MUST be a subset of [-pi,pi]
    std::vector<bool> hasBounds_;
    std::vector<double> lowerBounds_;
    std::vector<double> upperBounds_;

    // The base frame of the kinematic chain
    Eigen::Affine2d baseFrame_;
};

#endif
