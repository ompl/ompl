/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2019, Robert Bosch GmbH
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
*   * Neither the name of the Robert Bosch GmbH nor the names of its
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

/* Author: Luigi Palmieri */

#ifndef OMPL_TEST_HALTON_XD_
#define OMPL_TEST_HALTON_XD_

#include <ompl/base/samplers/deterministic/HaltonSequence.h>
#include <ompl/util/Console.h>

#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>
#include <sstream>
#include <string>

/** \brief Representation of XD Halton sequence read from file */
class HaltonXD
{
public:
    HaltonXD(unsigned int dim)
    {
        dimensions_ = dim;
    }

    void loadSequence(const char *filename)
    {
        std::ifstream input(filename);
        std::vector<double> line;
        std::string sline;
        while (std::getline(input, sline, '\n'))
        {
            std::istringstream in(sline);
            std::vector<double> line((std::istream_iterator<double>(in)), std::istream_iterator<double>());
            if (line.size() != dimensions_)
            {
                OMPL_ERROR("Error the read line has a different dimension of %d", line.size());
                OMPL_ERROR("Expected dimension of %d", dimensions_);
            }
            sequence_.push_back(line);
        }
    }

    std::vector<std::vector<double>> getSequence()
    {
        return sequence_;
    }

private:
    std::vector<std::vector<double>> sequence_;
    unsigned int dimensions_;
};

#endif
