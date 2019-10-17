/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Rice University
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

/* Author: Mark Moll */

#ifndef OMPL_UTIL_DISABLE_COMPILER_WARNING_
#define OMPL_UTIL_DISABLE_COMPILER_WARNING_

#define OMPL_PRAGMA_HELPER0(x) #x
#define OMPL_PRAGMA_HELPER1(x, y) OMPL_PRAGMA_HELPER0(x diagnostic ignored y)

// do nothing
#define OMPL_PUSH_DISABLE_GCC_WARNING(warning)
#define OMPL_POP_GCC
#define OMPL_PUSH_DISABLE_CLANG_WARNING(warning)
#define OMPL_POP_CLANG

#if defined(__clang__)
    #undef OMPL_PUSH_DISABLE_CLANG_WARNING
    #undef OMPL_POP_CLANG
    #define OMPL_PUSH_DISABLE_CLANG_WARNING(warning) \
        _Pragma("clang diagnostic push") \
        _Pragma(OMPL_PRAGMA_HELPER1(clang, OMPL_PRAGMA_HELPER0(warning)))
    #define OMPL_POP_CLANG \
        _Pragma("GCC diagnostic pop")
#elif defined __GNUC__
    #undef OMPL_PUSH_DISABLE_GCC_WARNING
    #undef OMPL_POP_GCC
    #define OMPL_PUSH_DISABLE_GCC_WARNING(warning) \
        _Pragma("GCC diagnostic push") \
        _Pragma(OMPL_PRAGMA_HELPER1(GCC, OMPL_PRAGMA_HELPER0(warning)))
    #define OMPL_POP_GCC \
        _Pragma("GCC diagnostic pop")
#endif

#endif
