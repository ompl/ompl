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

namespace ob = ompl::base;

//  Quotient Space contains three OMPL spaces, which we call Q1, Q0 and X1.
//
//  Q1 = Q0 x X1 is called the configuration space,
//  Q0 is the next lower-dimensional quotient-space and
//  X1 is the quotient Q1 / Q0.
//
//  We can visualize the relationships in the following diagram
//
//  [        ][ Q0 ]
//  [ Q1 ][____]
//  [        ][ X1 ]
//  [        ][      ]
//
//  We assume that Q1 and Q0 have been given (as ob::SpaceInformationPtr), 
//  and we compute the inverse of the quotient map, i.e. X1 = Q1/Q0. 
//
//  The quotient-space class is only implemented for a specific set of
//  combinations of Q1`and Q0. The following cases are currently implemented
//   ---- non-compound:
//   (1) Q1 Rn           , Q0 Rm             , 0<m<=n  => X1 = R(n-m) \union {0}
//   ---- compound:
//   (2) Q1 SE2          , Q0 R2                                 => X1 = SO2
//   (3) Q1 SE3          , Q0 R3                                 => X1 = SO3
//   (4) Q1 SE3xRn   , Q0 SE3                                => X1 = Rn
//   (5) Q1 SE3xRn   , Q0 R3                                 => X1 = SO3xRn
//   (6) Q1 SE3xRn   , Q0 SE3xRm     , 0<m<n     => X1 = R(n-m)
//  
//   (7) Q1 SE2xRn   , Q0 SE2                                => X1 = Rn
//   (8) Q1 SE2xRn   , Q0 R2                                 => X1 = SO2xRN
//   (9) Q1 SE2xRn   , Q0 SE2xRm     , 0<m<n     => X1 = R(n-m)

namespace ompl
{
    namespace geometric
    {
        class Quotient: public ob::Planner
        {
        typedef ob::Planner BaseT;
        enum QuotientSpaceType{ UNKNOWN, IDENTITY_SPACE, ATOMIC_RN, 
            RN_RM, SE2_R2, SE2RN_R2, SE2RN_SE2, SE2RN_SE2RM, 
            SE3_R3, SE3RN_R3, SE3RN_SE3, SE3RN_SE3RM };

        public:
            Quotient(const ob::SpaceInformationPtr &si, Quotient *parent_ = nullptr);
            ~Quotient();

            //final prevents subclasses to override
            ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override final; 

            virtual void Grow(double t) = 0;
            virtual bool GetSolution(ob::PathPtr &solution) = 0;
            virtual bool SampleQuotient(ob::State *q_random);

            virtual bool Sample(ob::State *q_random);
            virtual bool HasSolution();
            virtual void clear() override;
            virtual void setup() override;

            virtual double GetImportance() const;

            static void resetCounter();
            const ob::SpaceInformationPtr &GetX1() const;
            const ob::SpaceInformationPtr &GetQ1() const;
            const ob::SpaceInformationPtr &GetQ0() const;

            uint GetX1Dimension() const;
            uint GetQ1Dimension() const;
            uint GetQ0Dimension() const;
            uint GetDimension() const;

            const ob::StateSamplerPtr &GetX1SamplerPtr() const;
            const ob::StateSamplerPtr &GetQ1SamplerPtr() const;

            Quotient* GetParent() const;
            Quotient* GetChild() const;
            uint GetLevel() const;
            void SetLevel(uint);
            QuotientSpaceType GetType() const;
            void SetChild(Quotient *child_);
            void SetParent(Quotient *parent_);

            uint GetTotalNumberOfSamples() const;
            uint GetTotalNumberOfFeasibleSamples() const;
            friend std::ostream& operator<< (std::ostream& out, const ompl::geometric::Quotient& qtnt);
            virtual void Print(std::ostream& out) const;

            //Quotient Space Projection Operators
            //  ProjectX1Subspace: Q0 \times X1 \rightarrow X1
            //  ProjectQ0Subspace: Q0 \times X1 \rightarrow Q0
            void ProjectX1Subspace( const ob::State* q, ob::State* qX1 ) const;
            void ProjectQ0Subspace( const ob::State* q, ob::State* qQ0 ) const;
            void MergeStates(const ob::State *qQ0, const ob::State *qX1, ob::State *qQ1) const;

            void CheckSpaceHasFiniteMeasure(const ob::StateSpacePtr space) const;

            ob::OptimizationObjectivePtr GetOptimizationObjectivePtr() const;

        protected:

            const ob::StateSpacePtr ComputeQuotientSpace(const ob::StateSpacePtr Q1, const ob::StateSpacePtr Q0);
            QuotientSpaceType IdentifyQuotientSpaceType(const ob::StateSpacePtr Q1, const ob::StateSpacePtr Q0);

            ob::SpaceInformationPtr Q1{nullptr};
            ob::SpaceInformationPtr Q0{nullptr};
            ob::SpaceInformationPtr X1{nullptr};

            ob::StateSamplerPtr X1_sampler;
            ob::StateSamplerPtr Q1_sampler;
            ob::ValidStateSamplerPtr Q1_valid_sampler;

            ob::OptimizationObjectivePtr opt_;

            base::State *s_Q0_tmp{nullptr};
            base::State *s_X1_tmp{nullptr};

            QuotientSpaceType type;
            uint Q1_dimension{0};
            uint Q0_dimension{0};
            uint X1_dimension{0};

            static uint counter;
            uint id;
            uint level{0};

            bool hasSolution{false};
            bool firstRun{true};

            Quotient *parent{nullptr};
            Quotient *child{nullptr};

            uint totalNumberOfSamples{0};
            uint totalNumberOfFeasibleSamples{0};

        };
    }
}
#endif
