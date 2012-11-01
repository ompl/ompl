#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, Rice University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Author: Mark Moll

from os.path import basename, abspath, dirname, join
# Most likely, the OMPL Python module is not in the PYTHONPATH, so we add it
# using sys.path.insert.
ompl_binding_dir = join(
    dirname(dirname(dirname(dirname(dirname(dirname(
        abspath(__file__))))))),'py-bindings')
import sys
sys.path.insert(0, ompl_binding_dir)
from ompl.bindings_generator import code_generator_t, default_replacement

class ompl_rrtstar_generator_t(code_generator_t):
    def __init__(self):
        replacement = default_replacement
        code_generator_t.__init__(self, 'rrtstar',
            [join(ompl_binding_dir, 'bindings/geometric')], replacement)

    def filter_declarations(self):
        code_generator_t.filter_declarations(self)
        # don't export variables that need a wrapper
        self.ompl_ns.variables(lambda decl: decl.is_wrapper_needed()).exclude()
        # make objects printable that have a print function
        self.replace_member_functions(self.ompl_ns.member_functions('print'))
        # exclude solve() methods that take a "const PlannerTerminationCondition &"
        # as first argument; only keep the solve() that just takes a double argument
        self.ompl_ns.member_functions('solve', arg_types=['::ompl::base::PlannerTerminationCondition const &']).exclude()

        # Py++ seems to get confused by virtual methods declared in one module
        # that are *not* overridden in a derived class in another module. The
        # Planner class is defined in ompl::base and two of its virtual methods,
        # setProblemDefinition and checkValidity, are not overridden by most
        # planners. The code below forces Py++ to do the right thing (or at
        # least make it work). It seems rather hacky and there may be a better
        # solution.

        # do this for all planners
        for planner in ['BallTreeRRTstar', 'RRTstar']:
            self.ompl_ns.class_(planner).add_registration_code("""
            def("setProblemDefinition",&::ompl::base::Planner::setProblemDefinition,
                &%s_wrapper::default_setProblemDefinition, (bp::arg("pdef")) )""" % planner)
            self.ompl_ns.class_(planner).add_registration_code("""
            def("checkValidity",&::ompl::base::Planner::checkValidity,
                &%s_wrapper::default_checkValidity )""" % planner)

if __name__ == '__main__':
    sys.setrecursionlimit(50000)
    ompl_rrtstar_generator_t()
