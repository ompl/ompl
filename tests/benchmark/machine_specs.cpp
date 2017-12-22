/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University, Inc.
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

#define BOOST_TEST_MODULE "Memory"
#include <boost/test/unit_test.hpp>
#include "ompl/tools/benchmark/MachineSpecs.h"

#include <cstring>
#include <cstdlib>
#include <iostream>

using namespace ompl;

/* Just test we get some random values */
BOOST_AUTO_TEST_CASE(Simple)
{
    BOOST_CHECK(!machine::getHostname().empty());

    BOOST_CHECK(!machine::getCPUInfo().empty());

    machine::MemUsage_t start = machine::getProcessMemoryUsage();

    const unsigned int mb = 39;
    machine::MemUsage_t size = mb * 1024 * 1024 / sizeof(char);
    auto *data = (char*)malloc(size);
    memset(data, 0, size);

    machine::MemUsage_t u = machine::getProcessMemoryUsage() - start;

    int allocated_MB = (size/1024)/1024;
    int used_MB =  (u/1024)/1024;

    BOOST_CHECK(abs(used_MB - allocated_MB) < 2);

    free(data);
}
