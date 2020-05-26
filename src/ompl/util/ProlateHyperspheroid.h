/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, University of Toronto
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
*   * Neither the name of the University of Toronto nor the names of its
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

/* Author: Jonathan Gammell */

#ifndef OMPL_UTIL_PROLATE_HYPERSPHEROID_
#define OMPL_UTIL_PROLATE_HYPERSPHEROID_

#include <memory>

// For ease-of-use shared_ptr definition
#include <ompl/util/ClassForward.h>

namespace ompl
{
    /// @cond IGNORE
    // A forward declaration of the prolate hyperspheroid class
    OMPL_CLASS_FORWARD(ProlateHyperspheroid);
    /// @endcond

    /** \brief A class describing a prolate hyperspheroid, a special symmetric type of n-dimensional ellipse,
    for use in direct informed sampling for problems seeking to minimize path length.
    @par J. D. Gammell, T. D. Barfoot, S. S. Srinivasa, "Informed sampling for asymptotically optimal path planning."
    IEEE Transactions on Robotics (T-RO), 34(4): 966-984, Aug. 2018.
    DOI: <a href="https://doi.org/10.1109/TRO.2018.2830331">TRO.2018.2830331</a>.
    arXiv: <a href="https://arxiv.org/pdf/1706.06454">1706.06454 [cs.RO]</a>.
    <a href="https://www.youtube.com/watch?v=d7dX5MvDYTc">Illustration video</a>.
    <a href="https://www.youtube.com/watch?v=nsl-5MZfwu4">Short description video</a>. */
    class ProlateHyperspheroid
    {
    public:
        /** \brief The description of an n-dimensional prolate hyperspheroid */
        ProlateHyperspheroid(unsigned int n, const double focus1[], const double focus2[]);

        /** \brief Set the transverse diameter of the PHS */
        void setTransverseDiameter(double transverseDiameter);

        /** \brief Transform a point from a sphere to PHS. The return variable \e phs is expected to already exist.  */
        void transform(const double sphere[], double phs[]) const;

        /** \brief Check if the given point lies \e in the PHS. */
        bool isInPhs(const double point[]) const;

        /** \brief Check if the given point lies \e on the PHS. */
        bool isOnPhs(const double point[]) const;

        /** \brief The dimension of the PHS */
        unsigned int getPhsDimension() const;

        /** \brief The measure of the PHS */
        double getPhsMeasure() const;

        /** \brief The measure of the PHS for a given transverse diameter */
        double getPhsMeasure(double tranDiam) const;

        /** \brief The minimum transverse diameter of the PHS, i.e., the distance between the foci */
        double getMinTransverseDiameter() const;

        /** \brief Calculate length of a line that originates from one focus, passes through the given point, and
         * terminates at the other focus, i.e., the transverse diameter of the ellipse on which the given sample lies*/
        double getPathLength(const double point[]) const;

        /** \brief The state dimension of the PHS */
        unsigned int getDimension() const;

    protected:
    private:
        /** \brief A forward declaration to the data structure class for the PIMPL idiom. */
        struct PhsData;

        /** \brief A shared pointer to the actual data of a ProlateHyperspheroid. Used to hide Eigen from the header. */
        std::shared_ptr<PhsData> dataPtr_;

        // Functions
        /** \brief Calculate the rotation from the PHS frame to the world frame via singular-value decomposition using
         * the transverse symmetry of the PHS. */
        void updateRotation();

        /** \brief Calculate the hyperspheroid to PHS transformation matrix */
        void updateTransformation();
    };
}

#endif
