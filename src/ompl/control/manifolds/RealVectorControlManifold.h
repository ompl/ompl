/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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

/* Author: Ioan Sucan */

#ifndef OMPL_CONTROL_MANIFOLDS_REAL_VECTOR_CONTROL_MANIFOLD_
#define OMPL_CONTROL_MANIFOLDS_REAL_VECTOR_CONTROL_MANIFOLD_

#include "ompl/control/ControlManifold.h"
#include "ompl/base/manifolds/RealVectorBounds.h"
#include <vector>

namespace ompl
{
    namespace control
    {

        /** \brief Uniform sampler for the R<sup>n</sup> manifold */
        class RealVectorControlUniformSampler : public ControlSampler
        {
        public:

            /** \brief Constructor */
            RealVectorControlUniformSampler(const ControlManifold *manifold) : ControlSampler(manifold)
            {
            }

            virtual void sample(Control *control);
        };

        /** \brief A manifold representing R<sup>n</sup>. The distance function is the L2 norm. */
        class RealVectorControlManifold : public ControlManifold
        {
        public:

            /** \brief The definition of a control in R<sup>n</sup> */
            class ControlType : public Control
            {
            public:

                /** \brief Access element i of values.  This does not
                    check whether the index is within bounds */
                double operator[](unsigned int i) const
                {
                    return values[i];
                }

                /** \brief Access element i of values.  This does not
                    check whether the index is within bounds */
                double& operator[](unsigned int i)
                {
                    return values[i];
                }

                /** \brief An array of length \e n, representing the value of the control */
                double *values;
            };

            /** \brief Constructor takes the state manifold the controls correspond to and the dimension of the space of controls, \e dim */
            RealVectorControlManifold(const base::StateManifoldPtr &stateManifold, unsigned int dim) :
                ControlManifold(stateManifold), dimension_(dim), bounds_(dim), controlBytes_(dim * sizeof(double))
            {
                name_ = "RealVector" + name_;
            }

            virtual ~RealVectorControlManifold(void)
            {
            }

            /** \brief Set the bounds (min max values for each dimension) for the control */
            void setBounds(const base::RealVectorBounds &bounds);

            /** \brief Get the bounds (min max values for each dimension) for the control */
            const base::RealVectorBounds& getBounds(void) const
            {
                return bounds_;
            }

            virtual unsigned int getDimension(void) const;

            virtual void copyControl(Control *destination, const Control *source) const;

            virtual bool equalControls(const Control *control1, const Control *control2) const;

            virtual ControlSamplerPtr allocControlSampler(void) const;

            virtual Control* allocControl(void) const;

            virtual void freeControl(Control *control) const;

            virtual void nullControl(Control *control) const;

            virtual void copyToReals(const Control *control, std::vector<double> &reals) const;

            virtual unsigned int copyFromReals(Control *control, const std::vector<double> &reals) const;

            virtual void printControl(const Control *control, std::ostream &out) const;

            virtual void printSettings(std::ostream &out) const;

            virtual void setup(void);

        protected:

            /** \brief The dimension of the manifold */
            unsigned int           dimension_;

            /** \brief The bounds on controls */
            base::RealVectorBounds bounds_;

        private:
            std::size_t            controlBytes_;
        };
    }
}

#endif
