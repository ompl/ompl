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

/* \author Ioan Sucan */

#ifndef OMPL_DATASTRUCTURES_JL_MATRIX_
#define OMPL_DATASTRUCTURES_JL_MATRIX_

#include <Eigen/SVD>
#include "ompl/datastructures/RandomNumbers.h"
#include <cassert>

namespace ompl
{

    /** \brief Create a random matrix that can be used for the Johnson Lindenstrauss Lemma */
    class JLMatrix
    {
    public:

	JLMatrix(unsigned int k, unsigned int n)
	{
	    m_n = n;
	    m_k = k;
	    assert(m_k < m_n);
	    compute();
	}
	
	~JLMatrix(void)
	{
	}
	
	void compute(void);
	
	const Eigen::MatrixXd& getMatrix(void) const
	{
	    return m_R;
	}
	
	const Eigen::MatrixXd& getPseudoInverse(void) const
	{
	    return m_piR;
	}
	
    private:
	
	RNG                          m_rng;
	
	Eigen::MatrixXd              m_R;
	Eigen::MatrixXd              m_piR;
	
	unsigned int                 m_n;
	unsigned int                 m_k;
    };
}

#endif
