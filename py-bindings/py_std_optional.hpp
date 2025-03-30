/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024,  
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

/*********************************************************************
 * How to wrap std::optional objects with boost.python and use them both from
 * C++ and Python.
 *
 * https://stackoverflow.com/questions/36485840/wrap-boostoptional-using-boostpython
 *
 *********************************************************************/


#ifndef PY_BINDINGS_PY_STD_OPTIONAL_
#define PY_BINDINGS_PY_STD_OPTIONAL_

#include <exception>
#include <optional>
#include <type_traits>

#include <boost/python.hpp>
#include <boost/python/errors.hpp>

namespace detail
{
    template <typename T>
    struct to_python_optional
    {
        static PyObject* convert(const std::optional<T>& obj)
        {
            if(obj) {
              return boost::python::incref(boost::python::object(*obj).ptr());
            }
            else {
              // return None if the value is not set
              return boost::python::incref(boost::python::object().ptr());
            }
        }
    };

}  // namespace detail

#define PYREGISTER_OPTIONAL(T)            \
    boost::python::to_python_converter<   \
    std::optional<T>,                     \
    detail::to_python_optional<T> >();

#endif  // PY_BINDINGS_PY_STD_OPTIONAL_
