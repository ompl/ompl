/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Nikhil P, Vrushabh Zinage */

#ifndef OMPL_GEOMETRIC_PLANNERS_GSE_GSE_
#define OMPL_GEOMETRIC_PLANNERS_GSE_GSE_

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/datastructures/AdjacencyList.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/geometric/planners/PlannerIncludes.h"

namespace ompl
{
    namespace geometric
    {
        /**
         * @anchor GSE
         * GSE is a  path planning algorithm that uses the novel
         * concept of a gneralised shape to rapidly explore the
         * statespace in an efficient manner
         * 
         * @par External documentation
         * Vrushabh V Zinage and Satadal Ghosh. Generalized shape expansion-based motion
         * planning for uavs in three dimensional obstacle-cluttered environment. In 
         * AIAA Scitech 2020 Forum, page 0860, 2020.
         * DOI: [10.2514/6.2020-0860](https://doi.org/10.2514/6.2020-0860)<br>
        */ 

        /** \brief Generalised-Shape-Expansion */
        class GSE : public base::Planner
        {
        public:
            /** \brief Constructor */
            GSE(const base::SpaceInformationPtr &si, uint dim);

            ~GSE();

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            void clear(void);

            void setup(void);

            void getPlannerData(base::PlannerData &data) const;

            /** @brief add obstacles for the GSE planner 
             * 
             * Obtsacles are to be represented as an Eigen::MatrixXd of
             * size @b dim x  @b m where @b dim is the dimension of the
             * statespace and @b m is the no. of points sampled in each obstacle
            */
            void addObstacle(Eigen::MatrixXd &obs);

             /** @brief set the projector to be used in generating Eigen states (\f$R^{dim}\f$ state) */
            void setProjector(base::ProjectionEvaluatorPtr projector);

            void setResolution(double resolution);

        protected:
            /** \brief Representation of a valid explored states
             * 
             * Contains information about the represented state/ along
             * with information about the obstacles parameters about
             * that state. These Obstacle parameters are used in
             * generating the generalised shape function about this 
             * state
            */
    
            class Node
            {
            public:
                /** \brief Constructor that allocates memory for the state and also computes the obstacle parameters */
                Node(const Eigen::VectorXd &state, const std::vector<Eigen::MatrixXd> &obstacles, base::SpaceInformationPtr &si, double resolution);

                /** \brief Returns true if the argument state lies inside the generalised shape about the Node state */
                bool shape(Eigen::VectorXd &state);

                /** \brief Steers the argument state to the edge of the generalised shape of the Node state */
                void steer(Eigen::VectorXd &state);
                
                /** \brief Obstacle parameters of an object about the Node state */
                class obsParam
                {
                public:
                    /** \brief Constructor */
                    obsParam(double rmin, Eigen::VectorXd &normal, double theta)
                      : rmin_(rmin), normal_(normal), theta_(theta)
                    {
                    }

                    /** \brief Comparision class to help sort obsParam in terms of rmin 
                     * 
                     * return true if left's rmin is liss than right's rmin
                    */
                    struct ptrComp
                    {
                        bool operator()(const std::unique_ptr<obsParam> &left, const std::unique_ptr<obsParam> &right)
                        {
                            return left->rmin_ < right->rmin_;
                        }
                    };

                    /** \brief Minimum distance of given obstacle to Node state */
                    double rmin_;

                    /** \brief vector connecting the Node state to the closest point in the obstacle */
                    Eigen::VectorXd normal_;

                    /** \brief The half angle of the smallest cone bounding the obstacle  with the Node state as vertex */
                    double theta_;
                };

                /** \brief Desstructor */
                ~Node();

                /** \brief A pointer to the Space Information class. 
                 * 
                 * Thie Space Inforation class contains all the information about the space planning is done in
                */
                base::SpaceInformationPtr si_;

                /** \brief Eigen State, the Eigen::Vector representation of the state contained by the Node State */
                Eigen::VectorXd eigenState_;

                /** \brief The Node State i.e the state/vertetx represented by the Node class */
                base::State *state_;

                /** \brief Set of obstacle parameters corresponding to obstacles around the Node state */
                std::set<std::unique_ptr<obsParam>, obsParam::ptrComp> shapeParam_;

                /** \brief The number of obstacles around the Node state */
                uint obsSize;
            };

             /** \brief The reolution of the obstacle representation.
              * 
              * approximately equal to the step distance
              * between 2 neighbouring points in the obstacle
              */
            double resolution_;

            /** \brief The dimensionality of the planner */
            uint dim_;

            /** \brief The projector used to generate Eigen states (\f$R^{dim}\f$ state) */
            base::ProjectionEvaluatorPtr projector_;
            
            /** \brief The set of obstacles used in planning */
            std::vector<Eigen::MatrixXd> obstacles_;
            
            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief Vector containing set of explored states with corresponding obstacle parameters */
            std::vector<std::unique_ptr<Node>> vertices_;

            /** \brief A nearest-neighbors datastructure containing the set of explored vertices */
            std::shared_ptr<NearestNeighbors<std::pair<Eigen::VectorXd, int>>> nn_;
            
            /** \brief An adjacency list to store the graph of explored vertices */
            ompl::AdjacencyList adj_;

            /** \brief Add a particular state to the set of explored vertices */
            uint addVertex(const Eigen::VectorXd state);

            /** \brief Compute distances between 2 vertices (i.e the distance between states represented by the vertices) */
            double distanceFcn(const std::pair<Eigen::VectorXd, int> &a, const std::pair<Eigen::VectorXd, int> &b) const
            {
                return (a.first - b.first).norm();
            }
        };
    }  // namespace geometric
}  // namespace ompl

#endif