/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Caleb Voss and Wilson Beebe
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Authors: Caleb Voss, Wilson Beebe */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_VFRRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_VFRRT_

#include <limits>

#include <Eigen/Core>

#include <ompl/geometric/planners/rrt/RRT.h>

namespace ompl
{
    namespace geometric
    {
        /**
            \anchor gVFRRT
            \par Short description
            Vector Field Rapidly-exploring Random Tree (VFRRT) is a tree-based
            motion planner that tries to minimize the so-called upstream cost
            of a path. The upstream cost is defined by an integral over a
            user-defined vector field.

            \par External documentation
            I. Ko, B. Kim, and F. C. Park, Randomized path planning on vector fields, <em>Intl. J. of Robotics
           Research,</em> 33(13), pp. 1664–1682, 2014. DOI:
           [10.1177/0278364914545812](http://dx.doi.org/10.1177/0278364914545812)<br>

            [[PDF]](http://robotics.snu.ac.kr/fcp/files/_pdf_files_publications/201411_Randomized%20path%20planning.pdf)

        */
        class VFRRT : public RRT
        {
        public:
            using VectorField = std::function<Eigen::VectorXd(const base::State *)>;

            /** Constructor. */
            VFRRT(const base::SpaceInformationPtr &si, VectorField vf, double exploration, double initial_lambda,
                  unsigned int update_freq);

            /** Destructor. */
            ~VFRRT() override;

            /** Reset internal data. */
            void clear() override;

            /** Make a Monte Carlo estimate for the mean vector norm in the field. */
            double determineMeanNorm();

            /** Use the vector field to alter the direction of a sample. */
            Eigen::VectorXd getNewDirection(const base::State *qnear, const base::State *qrand);

            /** Calculates the weight omega which can be used to compute alpha and beta. */
            double biasedSampling(const Eigen::VectorXd &vrand, const Eigen::VectorXd &vfield, double lambdaScale);

            /**
             * Every nth time this function is called (where the nth step is the update
             * frequency given in the constructor) the value of lambda is updated and
             * the counts of efficient and inefficient samples added to the tree are
             * reset to 0. The measure for exploration inefficiency is also reset to 0.
             */
            void updateGain();

            /**
             * Computes alpha and beta, using these values to produce the vector returned by
             * getNewDirection. This produced vector can be used to determine the direction an
             * added state should be to maximize the upstream criterion of the produced path.
             */
            Eigen::VectorXd computeAlphaBeta(double omega, const Eigen::VectorXd &vrand, const Eigen::VectorXd &vfield);

            /**
             * This attempts to extend the tree from the motion m to a new motion
             * in the direction specified by the vector v.
             */
            Motion *extendTree(Motion *m, base::State *rstate, const Eigen::VectorXd &v);
            /**
             * Updates measures for exploration efficiency if a given motion m is added to the
             * nearest NearestNeighbors structure.
             */
            void updateExplorationEfficiency(Motion *m);

            /** Solve the planning problem. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void setup() override;

        private:
            /** Vector field for the environemnt. */
            const VectorField vf_;

            /** Number of efficient nodes. */
            unsigned int efficientCount_{0u};

            /** Number of inefficient nodes. */
            unsigned int inefficientCount_{0u};

            /** Current inefficiency. */
            double explorationInefficiency_{0.};

            /** User-specified exploration parameter. */
            double explorationSetting_;

            /** Current lambda value. */
            double lambda_;

            /** The number of steps until lambda is updated and efficiency metrics are reset. */
            unsigned int nth_step_;

            /** Current number of steps since lambda was updated/initialized. */
            unsigned int step_{0u};

            /** Average norm of vectors in the field. */
            double meanNorm_{0.};

            /** Dimensionality of vector field */
            unsigned int vfdim_{0u};
        };
    }
}
#endif
