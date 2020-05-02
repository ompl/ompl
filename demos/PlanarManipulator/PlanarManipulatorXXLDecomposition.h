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

#ifndef PLANAR_MANIPULATOR_XXL_DECOMPOSITION_
#define PLANAR_MANIPULATOR_XXL_DECOMPOSITION_

#include <eigen3/Eigen/Dense>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/xxl/XXLPlanarDecomposition.h>
#include "PlanarManipulator.h"
#include "PlanarManipulatorStateSpace.h"

// An XXL decomposition for a planar manipulator
class PMXXLDecomposition : public ompl::geometric::XXLPlanarDecomposition
{
public:
    /// \brief Constructor.  The end of the links in projectedJoints constitute the different
    /// layers to this decomposition.
    PMXXLDecomposition(ompl::base::SpaceInformationPtr si, const PlanarManipulator *manip,
                       const ompl::base::RealVectorBounds &xyBounds, const std::vector<int> &xySlices,
                       const int thetaSlices, std::vector<int> &projectedJoints, bool diagonalEdges)
      : ompl::geometric::XXLPlanarDecomposition(xyBounds, xySlices, thetaSlices, diagonalEdges)
      , si_(si)
      , manip_(manip)
      , projectedJoints_(projectedJoints)
    {
        if (projectedJoints_.size() == 0)
        {
            OMPL_WARN("No projected joints.  Assuming end effector projection");
            projectedJoints_.push_back(manip_->getNumLinks() - 1);
        }
        // Construct partial manipulators for projections
        int startLink = 0;
        const std::vector<double> &linkLengths = manip_->getLinkLengths();
        for (size_t i = 0; i < projectedJoints_.size(); ++i)
        {
            if (projectedJoints_[i] >= (int)manip_->getNumLinks())
                throw ompl::Exception("Projected joint is out of range");
            if (projectedJoints_[i] < startLink)
                throw ompl::Exception("Projected joints must be unique and sorted in ascending order");

            int numLinks = projectedJoints_[i] - startLink + 1;
            OMPL_DEBUG("PMXXLDecomposition: Constructing partial manipulator %lu with %d links [from links %d to %d]",
                       i, numLinks, startLink, startLink + numLinks);
            std::vector<double> lengths(numLinks);
            for (int j = startLink; j < startLink + numLinks; ++j)
                lengths[j - startLink] = linkLengths[j];
            partialManips_.push_back(PlanarManipulator(numLinks, lengths));
            startLink = projectedJoints_[i] + 1;
        }

        partialManips_[0].setBaseFrame(manip_->getBaseFrame());
    }

    virtual ~PMXXLDecomposition()
    {
    }

    /** \brief Return the number of layers possible in this decomposition.  Must be at least 1 */
    virtual int numLayers() const
    {
        return (int)projectedJoints_.size();
    }

    static void frameToPose(const Eigen::Affine2d &frame, Eigen::VectorXd &pose)
    {
        pose = Eigen::VectorXd(3);
        pose(0) = frame.translation()(0);
        pose(1) = frame.translation()(1);
        pose(2) = acos(frame.matrix()(0, 0));
    }

    virtual bool steerToRegion(int r, int layer, const ompl::base::State *start,
                               std::vector<ompl::base::State *> &states) const
    {
        if (layer >= (int)partialManips_.size())
            throw ompl::Exception("Layer is out of range");

        if (!start)
            throw ompl::Exception("Start state must be non-nullptr");

        // Get a handle to the manipulator
        PlanarManipulator &partialManip = partialManips_[layer];

        if (layer > 0)
        {
            std::vector<Eigen::Affine2d> frames;
            manip_->FK(start->as<PlanarManipulatorStateSpace::StateType>()->values, frames);

            // Set the base frame of the manipulator
            partialManip.setBaseFrame(frames[projectedJoints_[layer - 1]]);
        }

        // Current joint angles
        Eigen::VectorXd angles(partialManip.getNumLinks());

        // Extracting current joint angles.
        // For sublayers, the joint seed is a suffix of the seed
        if (layer > 0)
            memcpy(&angles[0],
                   &start->as<PlanarManipulatorStateSpace::StateType>()->values[projectedJoints_[layer - 1] + 1],
                   partialManip.getNumLinks() * sizeof(double));
        else
            memcpy(&angles[0], start->as<PlanarManipulatorStateSpace::StateType>()->values,
                   partialManip.getNumLinks() * sizeof(double));

        // Sample a pose in the desired region
        Eigen::VectorXd desired(3);
        sampleCoordinateFromRegion(r, &desired(0));

        // Create the end effector pose for partialManip.  This is the coordinate we just sampled
        Eigen::Affine2d link_pose = Eigen::Affine2d::Identity();
        link_pose.rotate(desired[2]);
        link_pose.translation()(0) = desired[0];
        link_pose.translation()(1) = desired[1];

        Eigen::Affine2d current_frame;
        partialManip.FK(angles, current_frame);
        Eigen::VectorXd current;
        frameToPose(current_frame, current);

        std::vector<int> projection;
        project(start, projection);

        // Trivially done.  Start is in the correct region
        if (projection[layer] == r)
        {
            return true;
        }

        // Nudge factor for the joint angles
        double alpha = 0.5;  // big-ish

        Eigen::VectorXd e(desired - current);

        unsigned int iter = 0;
        unsigned int maxIter = 20;

        Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(3, partialManip.getNumLinks());
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(3, partialManip.getNumLinks(), Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::MatrixXd D = Eigen::MatrixXd::Zero(partialManip.getNumLinks(), 3);

        unsigned int precedingVals = 0;
        if (layer > 0)
            precedingVals = projectedJoints_[layer - 1] + 1;

        bool valid = true;
        while (projection[layer] != r && iter++ < maxIter)
        {
            partialManip.Jacobian(angles, jac);

            // Invert the jacobian
            // Moore-Penrose Pseudoinverse
            svd.compute(jac);
            const Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType &d = svd.singularValues();

            // "Invert" the singular value matrix
            for (int i = 0; i < d.size(); ++i)
                if (d(i) > 1e-6)
                    D(i, i) = 1.0 / d(i);
                else
                    D(i, i) = 0.0;

            // Inverse is V*D^-1*U^t
            Eigen::MatrixXd jac_inv = svd.matrixV() * D * svd.matrixU().transpose();
            Eigen::VectorXd delta_theta = jac_inv * e;

            if (delta_theta(0) != delta_theta(0))  // nan
            {
                OMPL_ERROR("EPIC FAIL IN STEER");
                return false;
            }

            // New joint angles
            angles = angles + (alpha * delta_theta);
            partialManip.FK(angles, current_frame);
            frameToPose(current_frame, current);

            // New error
            e = desired - current;

            // Constructing state
            ompl::base::State *newState = si_->allocState();
            double *values = newState->as<PlanarManipulatorStateSpace::StateType>()->values;

            // Upper layer joints will not change
            if (precedingVals > 0)
                memcpy(values, start->as<PlanarManipulatorStateSpace::StateType>()->values,
                       precedingVals * sizeof(double));

            // Copying solution for this layer
            memcpy(&values[precedingVals], &angles[0], partialManip.getNumLinks() * sizeof(double));

            // Keep any remaining links the same.  Perturb them if they are in collision.
            valid = true;
            if (precedingVals + partialManip.getNumLinks() < manip_->getNumLinks())
            {
                valid = false;
                unsigned int index = precedingVals + partialManip.getNumLinks();
                unsigned int attempts = 5;
                const double *seed = start->as<PlanarManipulatorStateSpace::StateType>()->values;

                double variance = 0.025;
                for (unsigned int att = 0; att < attempts && !valid; ++att)
                {
                    // for(unsigned int i = index; i < manip_->getNumLinks(); ++i)
                    //    values[i] = rng_.uniformReal(-PI, PI);
                    for (unsigned int i = index; i < manip_->getNumLinks(); ++i)
                        values[i] = rng_.gaussian(seed[i], variance);

                    variance += variance;

                    // Make sure all values are in the requested range
                    si_->getStateSpace()->enforceBounds(newState);

                    // Check validity
                    valid = si_->isValid(newState);
                }
            }

            // Next state is valid.  Now see if the motion is valid
            const ompl::base::State *previous = (states.size() == 0 ? start : states.back());
            if (valid && si_->checkMotion(previous, newState))
            {
                project(newState, projection);
                states.push_back(newState);
            }
            else
            {
                // free allocated memory
                for (size_t i = 0; i < states.size(); ++i)
                    si_->freeState(states[i]);
                states.clear();

                return false;
            }
        }

        // Return true if state projects to the correct region
        if (projection[layer] == r)
        {
            return true;
        }
        return false;
    }

    /** \brief Sample a valid state \e s from region r in layer 0. */
    // todo: remove this
    virtual bool sampleFromRegion(int r, ompl::base::State *s, const ompl::base::State *seed = nullptr) const
    {
        return sampleFromRegion(r, s, seed, 0);
    }

    void initializePartialManipulator(int layer, const double *seedAngles) const
    {
        if (layer > 0)  // must set the base frame of the manipulator based on seed angles
        {
            PlanarManipulator &partialManip = partialManips_[layer];

            std::vector<Eigen::Affine2d> frames;
            manip_->FK(seedAngles, frames);
            partialManip.setBaseFrame(frames[projectedJoints_[layer - 1]]);
        }
    }

    void sampleEndEffectorPose(int region, Eigen::Affine2d &pose) const
    {
        // Sample a pose in the desired region
        std::vector<double> coord;
        sampleCoordinateFromRegion(region, coord);

        // Create the end effector pose for partialManip.  This is the coordinate we just sampled
        pose = Eigen::Affine2d::Identity();
        pose.rotate(coord[2]);
        pose.translation()(0) = coord[0];
        pose.translation()(1) = coord[1];
    }

    void initializePartialSeed(int layer, const double *seedAngles, std::vector<double> &partialSeed) const
    {
        const PlanarManipulator &partialManip = partialManips_[layer];

        partialSeed.resize(partialManip.getNumLinks());
        if (seedAngles)  // use seed angles through the partial manipulator
        {
            memcpy(&partialSeed[0], seedAngles, partialManip.getNumLinks() * sizeof(double));
        }
        else  // Set all joints randomly
        {
            for (size_t i = 0; i < partialSeed.size(); ++i)
                partialSeed[i] = rng_.uniformReal(-PI, PI);
        }
    }

    bool sampleRemainingJoints(int layer, ompl::base::State *s, const double *const seedVals,
                               const std::vector<double> &partialSln) const
    {
        double *values = s->as<PlanarManipulatorStateSpace::StateType>()->values;

        unsigned int precedingVals =
            (layer > 0 ? projectedJoints_[layer - 1] + 1 : 0);  // number of joints in the partial solution?

        // Copying seed values for immutable joints
        if (precedingVals > 0)
            memcpy(values, seedVals, precedingVals * sizeof(double));

        // Copying solution for this layer
        PlanarManipulator &partialManip = partialManips_[layer];
        memcpy(&values[precedingVals], &partialSln[0], partialManip.getNumLinks() * sizeof(double));

        precedingVals += partialManip.getNumLinks();

        // deal with the rest of the links after partialManip
        if (precedingVals < manip_->getNumLinks())
        {
            int maxAttempts = 10;  // try this many joint configurations
            double variance = 0.05;
            for (int att = 0; att < maxAttempts; ++att)
            {
                bool random = rng_.uniform01() < 0.50;
                for (unsigned int i = precedingVals; i < manip_->getNumLinks(); ++i)
                {
                    if (!seedVals || random)
                        values[i] = rng_.uniformReal(-PI, PI);
                    else
                        values[i] = rng_.gaussian(seedVals[i], variance);
                }

                variance += variance;  // double the variance after every attempt

                if (si_->isValid(s))
                    return true;
            }
        }
        else  // nothing to sample.  Just check validity
        {
            return si_->isValid(s);
        }

        return false;  // failed to find valid state
    }

    virtual bool sampleFromRegion(int r, ompl::base::State *s, const ompl::base::State *seed, int layer) const
    {
        if (layer >= (int)partialManips_.size())
            throw ompl::Exception("Layer is out of range");
        if (!seed && layer > 0)
            throw ompl::Exception("You must set the seed value to sample from a layer > 0");

        const double *seedVals = seed ? seed->as<PlanarManipulatorStateSpace::StateType>()->values : nullptr;
        initializePartialManipulator(layer, seedVals);
        PlanarManipulator &partialManip = partialManips_[layer];

        int maxPoses = 3 * (layer + 1);  // try up to this many IK poses.  try harder in lower levels
        for (int npose = 0; npose < maxPoses; ++npose)
        {
            Eigen::Affine2d link_pose;
            sampleEndEffectorPose(r, link_pose);

            if (layer > 0 && rng_.uniform01() < 0.5)  // bias end effector orientation
            {
                Eigen::Affine2d frame;
                manip_->FK(seedVals, frame);
                double theta = rng_.gaussian(acos(frame.matrix()(0, 0)), 0.2);

                double diff = theta - acos(link_pose.matrix()(0, 0));
                link_pose.rotate(diff);
            }

            std::vector<double> joint_seed;
            initializePartialSeed(layer, seedVals, joint_seed);

            // IK to reach end effector pose
            std::vector<double> solution;
            if (partialManip.FABRIK(solution, joint_seed, link_pose))  // about 2 orders of magnitude faster than
                                                                       // traditional ik.  Better success rate too
            {
                if (sampleRemainingJoints(layer, s, seedVals, solution))
                {
                    si_->getStateSpace()->enforceBounds(s);
                    return true;
                }
            }
        }

        return false;
    }

    /** \brief Project the given State into the XXLDecomposition */
    virtual void project(const ompl::base::State *s, std::vector<double> &coord, int layer = 0) const
    {
        std::vector<Eigen::Affine2d> frames;
        manip_->FK(s->as<PlanarManipulatorStateSpace::StateType>()->values, frames);

        coord.resize(3);
        coord[0] = frames[projectedJoints_[layer]].translation()[0];
        coord[1] = frames[projectedJoints_[layer]].translation()[1];
        coord[2] =
            atan2(frames[projectedJoints_[layer]].matrix()(1, 0), frames[projectedJoints_[layer]].matrix()(1, 1));
    }

    /** \brief Project the state into the decomposition and retrieve the region for all valid layers */
    virtual void project(const ompl::base::State *s, std::vector<int> &layers) const
    {
        std::vector<Eigen::Affine2d> frames;
        manip_->FK(s->as<PlanarManipulatorStateSpace::StateType>()->values, frames);

        layers.resize(projectedJoints_.size());
        std::vector<double> coord(3);
        for (size_t i = 0; i < projectedJoints_.size(); ++i)
        {
            coord[0] = frames[projectedJoints_[i]].translation()[0];
            coord[1] = frames[projectedJoints_[i]].translation()[1];
            coord[2] = atan2(frames[projectedJoints_[i]].matrix()(1, 0), frames[projectedJoints_[i]].matrix()(1, 1));
            layers[i] = coordToRegion(coord);
        }
    }

protected:
    ompl::base::SpaceInformationPtr si_;
    const PlanarManipulator *manip_;
    mutable std::vector<PlanarManipulator> partialManips_;
    std::vector<int> projectedJoints_;
};

#endif
