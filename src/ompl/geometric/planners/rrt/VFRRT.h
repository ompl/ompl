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

#ifndef VFRRT_H
#define VFRRT_H

#include <limits>

#include <eigen3/Eigen/Core>

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/tools/config/SelfConfig.h>

namespace ompl
{
    namespace geometric
    {
        /** Vector Field Rapidly-exploring Random Tree. */
        class VFRRT : public RRT
        {
        public:
            
            /** Vector Field type. */
            class VectorField
            {
            public:

                /** Constructor. */
                VectorField (const base::StateSpacePtr &space,
                             boost::function<Eigen::VectorXd (const base::State *state)> getVector)
                    : space_(space), getVector(getVector)
                {
                }
    
                virtual ~VectorField ()
                {
                }
    
                /** Get the space over which this vector field is defined. */
                const base::StateSpacePtr &getStateSpace() const
                {
                    return space_;
                }
    
                /** Get the vector at the given state. */
                Eigen::VectorXd operator() (const base::State *state) const
                {
                    return getVector(state);
                }
    
            private:
    
                /** The space over which this vector field is defined. */
                const base::StateSpacePtr &space_;
    
                /** Function to get a vector for a state. */
                boost::function<Eigen::VectorXd (const base::State *state)> getVector;
            };


            /** Constructor. */
            VFRRT (const base::SpaceInformationPtr &si, const VectorField &vf, double exploration,
                   double initial_lambda, unsigned int update_freq);
            
            /** Destructor. */
            virtual ~VFRRT ();
            
            /** Reset internal data. */
            virtual void clear ();
            
            /** Make a Monte Carlo estimate for the mean vector norm in the field. */
            double determineMeanNorm();
            
            /** Use the vector field to alter the direction of a sample. */
            Eigen::VectorXd getNewDirection (const base::State *qnear, const base::State *qrand);
            
            /** Calculates the weight omega which can be used to compute alpha and beta. */
            double biasedSampling (const Eigen::VectorXd &vrand, const Eigen::VectorXd &vfield,
                                   double lambdaScale);
            
            /**
             * Every nth time this function is called (where the nth step is the update 
             * frequency given in the constructor) the value of lambda is updated and 
             * the counts of efficient and inefficient samples added to the tree are 
             * reset to 0. The measure for exploration inefficiency is also reset to 0.
             */
            void updateGain ();
            
            /**
             * Computes alpha and beta, using these values to produce the vector returned by 
             * getNewDirection. This produced vector can be used to determine the direction an
             * added state should be to maximize the upstream criterion of the produced path.
             */
            Eigen::VectorXd computeAlphaBeta (double omega, const Eigen::VectorXd &vrand,
                                              const Eigen::VectorXd &vfield);
            
            /**
             * This attempts to extend the tree from the motion m to a new motion 
             * in the direction specified by the vector v.   
             */
            Motion *extendTree (Motion *m, const Eigen::VectorXd &v);

            /**
             * Updates measures for exploration efficiency if a given motion m is added to the
             * nearest NearestNeighbors structure.
             */
            void updateExplorationEfficiency(Motion *m);

            /** Solve the planning problem. */
            virtual base::PlannerStatus solve (const base::PlannerTerminationCondition &ptc);

        private:
            
            /** Vector field for the environemnt. */
            const VectorField &vf_;
            
            /** Number of effcient nodes. */
            unsigned int efficientCount;
            
            /** Number of ineffcient nodes. */
            unsigned int inefficientCount;
            
            /** Current inefficiency. */
            double explorationInefficiency;
            
            /** User-specified exploration parameter. */
            double explorationSetting;
            
            /** Current lambda value. */
            double lambda;
            
            /** The number of steps until lambda is updated and efficiency metrics are reset. */
            unsigned int nth_step;
            
            /** Current number of steps since lambda was updated/initialized. */
            unsigned int step;
            
            /** Average norm of vectors in the field. */
            double meanNorm_;
        };
    }
}
#endif
