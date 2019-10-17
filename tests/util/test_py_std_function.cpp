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

/* Author: Mark Moll */

#include <boost/python.hpp>
#include <memory>
#include "../../py-bindings/py_std_function.hpp"

namespace bp = boost::python;

struct IntClass
{
    IntClass(int i=0) : value(i) {}
    int value;
};

IntClass intClassFun0(IntClass i, int j) { i.value = j; return i; }
IntClass intClassFun1(IntClass& i, int j) { i.value = j; return i; }
IntClass intClassFun2(const IntClass& i, int j) { IntClass i2(i); i2.value = j; return i2; }
IntClass intClassFun3(IntClass& i, int& j) { i.value = j; return i; }
void intClassFun4(IntClass* i, int j) { i->value = j; }

std::function<IntClass(IntClass,int)>            intClassFun0_obj(intClassFun0);
std::function<IntClass(IntClass&,int)>           intClassFun1_obj(intClassFun1);
std::function<IntClass(const IntClass&,int)>     intClassFun2_obj(intClassFun2);
std::function<IntClass(IntClass&,int&)>          intClassFun3_obj(intClassFun3);
std::function<void(IntClass*,int)>               intClassFun4_obj(intClassFun4);


BOOST_PYTHON_MODULE(py_std_function)
{
    bp::class_<IntClass>("IntClass", bp::init<int>())
        .def_readwrite("value", &IntClass::value);

    PYREGISTER_FUNCTION(IntClass(IntClass,int),         IntClassFun0_t, "IntClassFun0_t")
    PYREGISTER_FUNCTION(IntClass(IntClass&,int),        IntClassFun1_t, "IntClassFun1_t")
    PYREGISTER_FUNCTION(IntClass(const IntClass&,int),  IntClassFun2_t, "IntClassFun2_t")
    PYREGISTER_FUNCTION(IntClass(IntClass&,int&),       IntClassFun3_t, "IntClassFun3_t")
    PYREGISTER_FUNCTION(void(IntClass*,int),            IntClassFun4_t, "IntClassFun4_t")

    bp::scope().attr("intClassFun0_obj") = intClassFun0_obj;
    bp::scope().attr("intClassFun1_obj") = intClassFun1_obj;
    bp::scope().attr("intClassFun2_obj") = intClassFun2_obj;
    bp::scope().attr("intClassFun3_obj") = intClassFun3_obj;
    bp::scope().attr("intClassFun4_obj") = intClassFun4_obj;
}