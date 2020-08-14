/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020,
 *  Max Planck Institute for Intelligent Systems (MPI-IS).
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
 *   * Neither the name of the MPI-IS nor the names
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

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_

#include <ompl/base/Planner.h>
#include "BundleSpaceComponent.h"
#include "BundleSpaceComponentFactory.h"

namespace ompl
{
    /** \brief This namespace contains datastructures and planners to
         exploit multilevel abstractions, which you have to specify using
         a sequence of SpaceInformationPtr (each with their own StateValidityChecker). */
    namespace multilevel
    {
        OMPL_CLASS_FORWARD(BundleSpaceComponent);
        OMPL_CLASS_FORWARD(BundleSpaceMetric);
        OMPL_CLASS_FORWARD(BundleSpacePropagator);

        /// \brief A single Bundle-space
        class BundleSpace : public ompl::base::Planner
        {
        private:
            using BaseT = ompl::base::Planner;
            using BaseT::si_;  // make it private.
            using BaseT::getSpaceInformation;

            // Note: use getBundle(), getFiber() or getBase() to access the SpaceInformationPtr

            /// \brief solve is disabled (use BundleSequence::solve)
            ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override final;

        public:
            /**  \brief Bundle Space contains three OMPL spaces, which we call Bundle, Base and Fiber.

                 - Bundle is (locally) a product space of Base and Fiber
                 - Base is a pointer to the next lower-dimensional Bundle-space (if any)
                 - Fiber is the quotient space Bundle / Base

                 We assume that Bundle and Base have been given (as ompl::base::SpaceInformationPtr),
                 and we automatically compute the fiber */

            BundleSpace(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_ = nullptr);
            virtual ~BundleSpace();

            virtual void setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef) override;

            virtual void grow() = 0;
            virtual bool getSolution(ompl::base::PathPtr &solution) = 0;
            virtual void setMetric(const std::string &sMetric) = 0;
            virtual void setPropagator(const std::string &sPropagator) = 0;

            virtual void sampleFromDatastructure(ompl::base::State *xBase) = 0;
            virtual void sampleFiber(ompl::base::State *xFiber);
            virtual void sampleBundle(ompl::base::State *xRandom);
            bool sampleBundleValid(ompl::base::State *xRandom);

            virtual bool hasSolution();
            virtual bool isInfeasible();

            virtual void clear() override;
            virtual void setup() override;

            virtual double getImportance() const = 0;

            /// \brief Allocate State, set entries to Identity/Zero
            ompl::base::State *allocIdentityStateFiber() const;
            ompl::base::State *allocIdentityStateBundle() const;
            ompl::base::State *allocIdentityStateBase() const;
            ompl::base::State *allocIdentityState(ompl::base::StateSpacePtr) const;
            void allocIdentityState(ompl::base::State *, ompl::base::StateSpacePtr) const;

            /// \brief Print Information pertaining to why a state failed being
            /// valid
            void debugInvalidState(const ompl::base::State *);

            /// \brief reset counter for number of levels
            static void resetCounter();

            /// \brief Get SpaceInformationPtr for Fiber
            const ompl::base::SpaceInformationPtr &getFiber() const;
            /// \brief Get SpaceInformationPtr for Bundle
            const ompl::base::SpaceInformationPtr &getBundle() const;
            /// \brief Get SpaceInformationPtr for Base
            const ompl::base::SpaceInformationPtr &getBase() const;

            /// \brief Dimension of Fiber Space
            unsigned int getFiberDimension() const;
            /// \brief Dimension of Base Space
            unsigned int getBaseDimension() const;
            /// \brief Dimension of Bundle Space
            unsigned int getBundleDimension() const;

            const ompl::base::StateSamplerPtr &getFiberSamplerPtr() const;
            const ompl::base::StateSamplerPtr &getBundleSamplerPtr() const;

            /// \brief Parent is a more simplified Bundle-space
            /// (higher in abstraction hierarchy)
            BundleSpace *getParent() const;
            /// \brief Child is a less simplified Bundle-space
            /// (lower in abstraction hierarchy)
            BundleSpace *getChild() const;

            bool hasBaseSpace() const;
            bool hasParent() const;
            bool hasChild() const;

            /// Level in abstraction hierarchy of Bundle-spaces
            unsigned int getLevel() const;

            /// Change abstraction level
            void setLevel(unsigned int);

            /// Set pointer to next Bundle-space (more dimensional)
            void setChild(BundleSpace *child_);

            /// Set pointer to previous Bundle-space (less dimensional)
            void setParent(BundleSpace *parent_);

            /// \brief Bundle Space Projection Operator onto second component
            /// ProjectFiber: Base \times Fiber \rightarrow Fiber
            void projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const;

            /// \brief Bundle Space Projection Operator onto first component
            /// ProjectBase: Base \times Fiber \rightarrow Base
            void projectBase(const ompl::base::State *xBundle, ompl::base::State *xBase) const;

            /// \brief Lift a state from Base to Bundle using a Fiber State
            void liftState(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                           ompl::base::State *xBundle) const;

            /// \brief return xResult, being a state on the basePath at location
            //    Input: - basePath on getBase()
            //           - location in [0, basePath.length()]
            //    Output:- xResult
            //
            //    Example: location=0 returns basePath.front()
            //    Example: location=basePath.length() returns basePath.back()
            unsigned int interpolateAlongBasePath(const std::vector<base::State *> basePath, double location,
                                                  base::State *xResult) const;

            ompl::base::OptimizationObjectivePtr getOptimizationObjectivePtr() const;

            /// \brief Write class to stream (use as std::cout << *this << std::endl)
            ///  Actual implementation is in void print(std::ostream& out),
            ///  which can be inherited.
            friend std::ostream &operator<<(std::ostream &, const BundleSpace &);

            bool isDynamic() const;

        private:
            ompl::base::SpaceInformationPtr Bundle{nullptr};
            ompl::base::SpaceInformationPtr Base{nullptr};
            ompl::base::SpaceInformationPtr Fiber{nullptr};

            /// Level in sequence of Bundle-spaces
            unsigned int level_{0};

        protected:
            /// Check if Bundle-space is unbounded
            void checkBundleSpaceMeasure(std::string name, const ompl::base::StateSpacePtr space) const;
            void sanityChecks() const;
            void MakeFiberSpace();

            std::vector<BundleSpaceComponentPtr> components_;

            /// Internal function implementing actual printing to stream
            virtual void print(std::ostream &out) const;

            ompl::base::StateSamplerPtr Fiber_sampler_;
            ompl::base::StateSamplerPtr Bundle_sampler_;
            ompl::base::ValidStateSamplerPtr Bundle_valid_sampler_;

            ompl::base::OptimizationObjectivePtr opt_;

            /// A temporary state on Base
            ompl::base::State *xBaseTmp_{nullptr};
            /// A temporary state on Fiber
            ompl::base::State *xFiberTmp_{nullptr};
            /// A temporary state on Bundle
            ompl::base::State *xBundleTmp_{nullptr};

            static unsigned int counter_;

            /// Identity of space (to keep track of number of Bundle-spaces created)
            unsigned int id_{0};

            bool hasSolution_{false};
            bool firstRun_{true};

            bool isDynamic_{false};

            BundleSpace *parent_{nullptr};
            BundleSpace *child_{nullptr};

            /** \brief Goal state or goal region */
            ompl::base::Goal *goal_;

            /** \brief Metric on bundle space */
            BundleSpaceMetricPtr metric_;

            /** \brief Propagator (steering or interpolation) on bundle space.
             * Note: currently just a stub for base::StatePropagator*/
            BundleSpacePropagatorPtr propagator_;
        };
    }  // namespace multilevel
}  // namespace ompl
#endif
