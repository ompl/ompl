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

/** \author Ioan Sucan  */

#include "ompl/datastructures/JLMatrix.h"
#include <ros/console.h>
	
void ompl::JLMatrix::compute(void)
{
    m_R.resize(m_k, m_n);
    
    // sample a random matrix with elements of mean 0 and variance 1
    for (unsigned int i = 0 ; i < m_k ; ++i)
	for (unsigned int j = 0 ; j < m_n ; ++j)
	    m_R(i, j) = m_rng.gaussian(0.0, 1.0);
    
    
    // make the rows orthonormal
    for (unsigned int row = 0 ; row < m_k ; ++row)
    {
	Eigen::MatrixXd::RowXpr rowVec = m_R.row(row);
	for (unsigned int prevRow = 0 ; prevRow < row ; ++prevRow)
	{
	    Eigen::MatrixXd::RowXpr prevRowVec = m_R.row(prevRow);
	    rowVec -= rowVec.dot(prevRowVec) * prevRowVec;
	}
	m_R.row(row) = rowVec.normalized();
    }
    
    // compute SVD of matrix
    Eigen::SVD<Eigen::MatrixXd> svd(m_R.transpose());
    // m_R' = m_U * m_Sigma * m_V'
    
    // compute pseudo-inverse
    Eigen::MatrixXd sigma = svd.singularValues().asDiagonal();
    for (int i = 0 ; i < sigma.rows() ; ++i)
	if (fabs(sigma(i, i)) < 1e-6)
	{
	    // if by chance we got a matrix with rank < m_k
	    // we recompute it
	    ROS_WARN("Recomputing JL matrix");
	    compute();
	    return;
	}
	else
	    sigma(i,i) = 1.0 / sigma(i, i);
    
    // take U and V reversed since we have the SVD of the transpose
    Eigen::MatrixXd U = svd.matrixV();
    Eigen::MatrixXd V = svd.matrixU();
    
    // the pseudo-inverse
    m_piR = V * sigma * U.transpose();
    
    // compute basis for null-space
    // this is (m_n - m_k) m_n-dimensional vectors
    // that have no components of the column vectors in matrix V
    /*
      Eigen::MatrixXd B = Eigen::MatrixXd::Identity(m_n, m_n);
      
      for (int i = 0 ; i < V.cols() ; ++i)
      {
      int row, col;
      V.col(i).maxCoeff(&row, &col);
      B.col(row) = V.col(i);
      }
    */
    //	    std::cout << V << " M = " << row << ", " << col << std::endl;
    
}
