/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Stuttgart
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
 *   * Neither the name of the University of Stuttgart nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

/* Author: Andreas Orthey */

#ifndef OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_QUOTIENT_
#define OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_QUOTIENT_

#include <ompl/base/Planner.h>

namespace ompl
{
    namespace geometric
    {
        /** \brief A single quotient-space */
        class QuotientSpace : public ompl::base::Planner
        {
            using BaseT = ompl::base::Planner;
            enum QuotientSpaceType
            {
                UNKNOWN,
                ATOMIC,
                IDENTITY_SPACE_RN,
                IDENTITY_SPACE_SE2,
                IDENTITY_SPACE_SE2RN,
                IDENTITY_SPACE_SO2RN,
                IDENTITY_SPACE_SE3,
                IDENTITY_SPACE_SE3RN,
                RN_RM,
                SE2_R2,
                SE2RN_R2,
                SE2RN_SE2,
                SE2RN_SE2RM,
                SO2RN_SO2,
                SO2RN_SO2RM,
                SE3_R3,
                SE3RN_R3,
                SE3RN_SE3,
                SE3RN_SE3RM
            };

        public:
            /**  \brief Quotient Space contains three OMPL spaces, which we call Q1, Q0 and X1.

                 Q1 = Q0 x X1 is a product space of Q0 and X1 and
                      is the main quotient-space of this class
                 Q0 is a pointer to the next lower-dimensional quotient-space and
                 X1 is the quotient-space  Q1 / Q0

                 We assume that Q1 and Q0 have been given (as ompl::base::SpaceInformationPtr),
                 and we compute the inverse of the quotient map, i.e. X1 = Q1/Q0. */

            QuotientSpace(const ompl::base::SpaceInformationPtr &si, QuotientSpace *parent_ = nullptr);
            ~QuotientSpace();

            /** \brief solve disabled (use MultiQuotient::solve)
                final prevents subclasses to override */
            ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override final;
            virtual void setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef) override;

            virtual void grow() = 0;
            virtual bool getSolution(ompl::base::PathPtr &solution) = 0;
            virtual bool sampleQuotient(ompl::base::State *q_random);
            virtual bool sample(ompl::base::State *q_random);
            virtual bool hasSolution();
            virtual void clear() override;
            virtual void setup() override;

            virtual double getImportance() const;

            /** \brief reset counter for number of levels */
            static void resetCounter();

            /** \brief Get SpaceInformationPtr for X1
                 (Note: X1 is the second component of Q1 = Q0 x X1) */
            const ompl::base::SpaceInformationPtr &getX1() const;
            /** \brief Get SpaceInformationPtr for Q1
                 (Note: Q1 is the product space Q1 = Q0 x X1) */
            const ompl::base::SpaceInformationPtr &getQ1() const;
            /** \brief Get SpaceInformationPtr for Q0
                 (Note: Q0 is the first component of Q1 = Q0 x X1) */
            const ompl::base::SpaceInformationPtr &getQ0() const;

            /** \brief Dimension of space X1 */
            unsigned int getX1Dimension() const;
            /** \brief Dimension of space Q1 */
            unsigned int getQ1Dimension() const;
            /** \brief Dimension of space Q0 */
            unsigned int getQ0Dimension() const;
            /** \brief Dimension of space Q1 */
            unsigned int getDimension() const;

            const ompl::base::StateSamplerPtr &getX1SamplerPtr() const;
            const ompl::base::StateSamplerPtr &getQ1SamplerPtr() const;

            /** \brief Parent is a more simplified quotient-space
                (higher in abstraction hierarchy) */
            QuotientSpace *getParent() const;
            /** \brief Child is a less simplified quotient-space
                (lower in abstraction hierarchy) */
            QuotientSpace *getChild() const;
            /** \brief Level in abstraction hierarchy of quotient-spaces */
            unsigned int getLevel() const;
            /** \brief Change abstraction level */
            void setLevel(unsigned int);
            /** \brief Type of quotient-space */
            QuotientSpaceType getType() const;
            /** \brief Set pointer to less simplified quotient-space */
            void setChild(QuotientSpace *child_);
            /** \brief Set pointer to more simplified quotient-space */
            void setParent(QuotientSpace *parent_);

            /** \brief Number of samples drawn on space Q1 */
            unsigned int getTotalNumberOfSamples() const;
            /** \brief Number of feasible samples drawn on space Q1 */
            unsigned int getTotalNumberOfFeasibleSamples() const;

            /** \brief Quotient Space Projection Operator onto second component
                ProjectX1: Q0 \times X1 \rightarrow X1 */
            void projectX1(const ompl::base::State *q, ompl::base::State *qX1) const;
            /** \brief Quotient Space Projection Operator onto first component
                ProjectQ0: Q0 \times X1 \rightarrow Q0 */
            void projectQ0(const ompl::base::State *q, ompl::base::State *qQ0) const;
            /** \brief Merge a state from Q0 and X1 into a state on Q1 (concatenate) */
            void mergeStates(const ompl::base::State *qQ0, const ompl::base::State *qX1, ompl::base::State *qQ1) const;

            /** \brief Check if quotient-space is unbounded */
            void checkSpaceHasFiniteMeasure(const ompl::base::StateSpacePtr space) const;

            ompl::base::OptimizationObjectivePtr getOptimizationObjectivePtr() const;

            /** \brief Write class to stream (use as std::cout << *this << std::endl)
                 Actual implementation is in void print(std::ostream& out),
                 which can be inherited. */
            friend std::ostream &operator<<(std::ostream &out, const QuotientSpace &qtnt);

        protected:
            /** \brief Internal function implementing actual printing to stream */
            virtual void print(std::ostream &out) const;

            /**  \brief Compute the quotient Q1 / Q0 between two given spaces.
             * */
            const ompl::base::StateSpacePtr computeQuotientSpace(const ompl::base::StateSpacePtr Q1,
                                                                 const ompl::base::StateSpacePtr Q0);

            /** \brief Identify the type of the quotient Q1 / Q0 */
            QuotientSpaceType identifyQuotientSpaceType(const ompl::base::StateSpacePtr Q1,
                                                        const ompl::base::StateSpacePtr Q0);

            ompl::base::SpaceInformationPtr Q1{nullptr};
            ompl::base::SpaceInformationPtr Q0{nullptr};
            ompl::base::SpaceInformationPtr X1{nullptr};

            ompl::base::StateSamplerPtr X1_sampler_;
            ompl::base::StateSamplerPtr Q1_sampler_;
            ompl::base::ValidStateSamplerPtr Q1_valid_sampler_;

            ompl::base::OptimizationObjectivePtr opt_;

            /** \brief A temporary state on Q0 */
            ompl::base::State *s_Q0_tmp_{nullptr};
            /** \brief A temporary state on X1 */
            ompl::base::State *s_X1_tmp_{nullptr};

            QuotientSpaceType type_;
            unsigned int Q1_dimension_{0};
            unsigned int Q0_dimension_{0};
            unsigned int X1_dimension_{0};

            static unsigned int counter_;
            /** \brief Identity of space (to keep track of number of quotient-spaces created) */
            unsigned int id_{0};
            /** \brief Level in sequence of quotient-spaces */
            unsigned int level_{0};

            bool hasSolution_{false};
            bool firstRun_{true};

            QuotientSpace *parent_{nullptr};
            QuotientSpace *child_{nullptr};

            unsigned int totalNumberOfSamples_{0};
            unsigned int totalNumberOfFeasibleSamples_{0};
        };
    }  // namespace geometric
}  // namespace ompl
#endif
