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

#ifndef OMPL_GEOMETRIC_PLANNERS_FiberBundle_
#define OMPL_GEOMETRIC_PLANNERS_FiberBundle_

#include <ompl/base/Planner.h>

namespace ompl
{
    namespace geometric
    {
        /// \brief A single Fiber Bundle
        class FiberBundle : public ompl::base::Planner
        {
            using BaseT = ompl::base::Planner;
            enum FiberBundleType
            {
                UNKNOWN,
                IDENTITY_SPACE_RN,
                IDENTITY_SPACE_SE2,
                IDENTITY_SPACE_SE2RN,
                IDENTITY_SPACE_SO2RN,
                IDENTITY_SPACE_SE3,
                IDENTITY_SPACE_SE3RN,
                ATOMIC_RN,
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
            /**  \brief Fiber Bundle Space contains three OMPL spaces, which we call totalSpace, baseSpace and fiber.

                 totalSpace = baseSpace x fiber is locally a product space of baseSpace and fiber
                 baseSpace is a pointer to the next lower-dimensional fiber bundle and
                 fiber is the quotient-space of totalSpace/baseSpace

                 We assume that totalSpace and baseSpace have been given (as ompl::base::SpaceInformationPtr),
                 and we compute the inverse of the quotient map, i.e. fiber = totalSpace/baseSpace. */

            FiberBundle(const ompl::base::SpaceInformationPtr &si, FiberBundle *parent_ = nullptr);
            ~FiberBundle();

            /// \brief solve disabled (use MultiQuotient::solve)
            /// final prevents subclasses to override
            ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override final;
            virtual void setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef) override;

            virtual void grow() = 0;
            virtual bool getSolution(ompl::base::PathPtr &solution) = 0;
            virtual bool sampleBaseSpace(ompl::base::State *q_random);
            virtual bool sample(ompl::base::State *q_random);
            virtual bool hasSolution();
            virtual void clear() override;
            virtual void setup() override;

            virtual double getImportance() const;

            /// reset counter for number of levels
            static void resetCounter();

            /// \brief Get SpaceInformationPtr for fiber
            ///  (Note: fiber is the second component of totalSpace = baseSpace x fiber)
            const ompl::base::SpaceInformationPtr &getFiber() const;
            /// \brief Get SpaceInformationPtr for totalSpace
            ///  (Note: totalSpace is the product space totalSpace = baseSpace x fiber)
            const ompl::base::SpaceInformationPtr &getTotalSpace() const;
            /// \brief Get SpaceInformationPtr for baseSpace
            ///  (Note: baseSpace is the first component of totalSpace = baseSpace x fiber)
            const ompl::base::SpaceInformationPtr &getBaseSpace() const;

            /// Dimension of space fiber
            unsigned int getFiberDimension() const;
            /// Dimension of space totalSpace
            unsigned int getTotalSpaceDimension() const;
            /// Dimension of space baseSpace
            unsigned int getBaseSpaceDimension() const;
            /// Dimension of space totalSpace
            unsigned int getDimension() const;

            const ompl::base::StateSamplerPtr &getFiberSamplerPtr() const;
            const ompl::base::StateSamplerPtr &getTotalSpaceSamplerPtr() const;

            /// \brief Parent is a more simplified fiber bundle
            /// (higher in abstraction hierarchy)
            FiberBundle *getParent() const;
            /// \brief Child is a less simplified fiber bundle
            /// (lower in abstraction hierarchy)
            FiberBundle *getChild() const;

            bool hasParent() const;
            bool hasChild() const;

            /// Level in abstraction hierarchy of fiber bundles
            unsigned int getLevel() const;
            /// Change abstraction level
            void setLevel(unsigned int);
            /// Type of fiber bundle
            FiberBundleType getType() const;
            /// Set pointer to less simplified fiber bundle
            void setChild(FiberBundle *child_);
            /// Set pointer to more simplified fiber bundle
            void setParent(FiberBundle *parent_);

            /// Number of samples drawn on space totalSpace
            unsigned int getTotalNumberOfSamples() const;
            /// Number of feasible samples drawn on space totalSpace
            unsigned int getTotalNumberOfFeasibleSamples() const;

            /// \brief Quotient Space Projection Operator onto second component
            /// Projectfiber: baseSpace \times fiber \rightarrow fiber
            void projectFiber(const ompl::base::State *q, ompl::base::State *qfiber) const;
            /// \brief Quotient Space Projection Operator onto first component
            /// ProjectbaseSpace: baseSpace \times fiber \rightarrow baseSpace
            void projectBaseSpace(const ompl::base::State *q, ompl::base::State *qbaseSpace) const;
            /// Merge a state from baseSpace and fiber into a state on totalSpace (concatenate)
            void mergeStates(const ompl::base::State *qbaseSpace, const ompl::base::State *qfiber, ompl::base::State *qtotalSpace) const;

            /// Check if fiber bundle is unbounded
            void checkSpaceHasFiniteMeasure(const ompl::base::StateSpacePtr space) const;

            ompl::base::OptimizationObjectivePtr getOptimizationObjectivePtr() const;

            /// \brief Write class to stream (use as std::cout << *this << std::endl)
            ///  Actual implementation is in void print(std::ostream& out),
            ///  which can be inherited.
            friend std::ostream &operator<<(std::ostream &out, const FiberBundle &qtnt);

            bool isDynamic() const;

        protected:
            /// Internal function implementing actual printing to stream
            virtual void print(std::ostream &out) const;

            ///  \brief Compute the quotient totalSpace / baseSpace between two given spaces.
            const ompl::base::StateSpacePtr computeFiberSpace(const ompl::base::StateSpacePtr totalSpace,
                                                                 const ompl::base::StateSpacePtr baseSpace);

            /// Identify the type of the quotient totalSpace / baseSpace
            FiberBundleType identifyFiberSpaceType(const ompl::base::StateSpacePtr totalSpace,
                                                        const ompl::base::StateSpacePtr baseSpace);

            ompl::base::SpaceInformationPtr totalSpace{nullptr};
            ompl::base::SpaceInformationPtr baseSpace{nullptr};
            ompl::base::SpaceInformationPtr fiber{nullptr};

            ompl::base::StateSamplerPtr fiber_sampler_;
            ompl::base::StateSamplerPtr totalSpace_sampler_;
            ompl::base::ValidStateSamplerPtr totalSpace_valid_sampler_;

            ompl::base::OptimizationObjectivePtr opt_;

            /// A temporary state on baseSpace
            ompl::base::State *s_baseSpace_tmp_{nullptr};
            /// A temporary state on fiber
            ompl::base::State *s_fiber_tmp_{nullptr};

            FiberBundleType type_;
            unsigned int totalSpace_dimension_{0};
            unsigned int baseSpace_dimension_{0};
            unsigned int fiber_dimension_{0};

            static unsigned int counter_;
            /// Identity of space (to keep track of number of fiber bundles created)
            unsigned int id_{0};
            /// Level in sequence of fiber bundles
            unsigned int level_{0};

            bool hasSolution_{false};
            bool firstRun_{true};

            bool isDynamic_{false};

            FiberBundle *parent_{nullptr};
            FiberBundle *child_{nullptr};

            unsigned int totalNumberOfSamples_{0};
            unsigned int totalNumberOfFeasibleSamples_{0};

            /** \brief Goal state or goal region */
            ompl::base::Goal *goal_;
        };
    }  // namespace geometric
}  // namespace ompl
#endif
