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

from sys import argv, setrecursionlimit
from pygccxml import declarations
from pyplusplus.module_builder import call_policies
from ompl.bindings_generator import code_generator_t, default_replacement


class ompl_base_generator_t(code_generator_t):
    """Class for generating the ompl.base python module."""

    def __init__(self):
        replacement = default_replacement
        # special case for abstract base class Path with pure virtual print method:
        replacement['::ompl::base::Path::print'] = ('def("__str__", bp::pure_virtual(&__str__))', """
        std::string __str__(%s* obj)
        {
            std::ostringstream s;
            obj->print(s);
            return s.str();
        }
        """)
        # A C++ call like "foo.printState(state, std::cout)" will be replaced with
        # something more pythonesque: "print foo.string(state)"
        replacement['printState'] = ('def("string", &__printState)', """
        std::string __printState(%s* space, ompl::base::State* state)
        {
            std::ostringstream s;
            space->printState(state, s);
            return s.str();
        }
        """)
        # A C++ call like "foo.printProjections(std::cout)" will be replaced with
        # something more pythonesque: "print foo.projections()"
        replacement['printProjections'] = ('def("projections", &__printProjections)', """
        std::string __printProjections(%s* obj)
        {
            std::ostringstream s;
            obj->printProjections(s);
            return s.str();
        }
        """)
        # A C++ call like "foo.printProjection(projection, std::cout)" will be replaced with
        # something more pythonesque: "print foo.projection()"
        replacement['printProjection'] = ('def("projection", &__printProjection)', """
        std::string __printProjection(%s* obj, const ompl::base::EuclideanProjection &projection)
        {
            std::ostringstream s;
            obj->printProjection(projection, s);
            return s.str();
        }
        """)
        # add a wrapper for the
        # ompl::base::SpaceInformation::setStateValidityChecker. This wrapper
        # makes a SpaceInformation reference accessible to the validity checker and
        # also deals correctly with C-style pointers.
        replacement['setStateValidityChecker'] = ('def("setStateValidityChecker", &setStateValidityCheckerWrapper)', """
        struct IsValidFunPyWrapper
        {
            IsValidFunPyWrapper( bp::object callable ) : callable_( callable ) {}

            bool operator()(const ompl::base::SpaceInformation* si, const ompl::base::State* state)
            {
                PyGILState_STATE gstate = PyGILState_Ensure();
                bool ret = bp::extract<bool>(callable_(bp::ptr(si), bp::ptr(state)));
                PyGILState_Release( gstate );
                return ret;
            }

            bp::object callable_;
        };

        void setStateValidityCheckerWrapper(%s* obj, bp::object function)
        {
            obj->setStateValidityChecker( boost::bind(
            boost::function<bool (const ompl::base::SpaceInformation*, const ompl::base::State*)>(IsValidFunPyWrapper(function)),
                obj, _1));
        }
        """)
        # add a wrapper for the ompl::base::SpaceInformation::setValidStateSamplerAllocator.
        replacement['setValidStateSamplerAllocator'] = ('def("setValidStateSamplerAllocator", &setValidStateSamplerAllocatorWrapper)', """
        struct SetValidStateSamplerPyWrapper
        {
            SetValidStateSamplerPyWrapper( bp::object callable ) : callable_( callable ) {}

            ompl::base::ValidStateSamplerPtr operator()(const ompl::base::SpaceInformation* si)
            {
                PyGILState_STATE gstate = PyGILState_Ensure();
                ompl::base::ValidStateSamplerPtr ret = bp::extract<ompl::base::ValidStateSamplerPtr>(callable_(bp::ptr(si)));
                PyGILState_Release( gstate );
                return ret;
            }

            bp::object callable_;
        };

        void setValidStateSamplerAllocatorWrapper(%s* obj, bp::object function)
        {
            obj->setValidStateSamplerAllocator( boost::bind(
            ompl::base::ValidStateSamplerAllocator(SetValidStateSamplerPyWrapper(function)), _1));
        }
        """)
        code_generator_t.__init__(self, 'base', None, replacement)

    def filter_declarations(self):
        # force ProblemDefinition to be included, because it is used by other modules
        self.ompl_ns.class_('ProblemDefinition').include()
        # force the abstract base class Path to be included, because it is used by other modules
        self.ompl_ns.class_('Path').include()
        code_generator_t.filter_declarations(self)
        # rename STL vectors of certain types
        self.std_ns.class_('vector< int >').rename('vectorInt')
        self.std_ns.class_('vector< double >').rename('vectorDouble')
        self.std_ns.class_('vector< unsigned int >').rename('vectorUint')
        self.std_ns.class_('vector< std::vector<unsigned int> >').rename('vectorVectorUint')
        self.std_ns.class_('vector< std::valarray<double> >').rename('vectorValarrayDouble')
        self.std_ns.class_('vector< ompl::base::State* >').rename('vectorState')
        self.std_ns.class_('vector< ompl::base::State const* >').rename('vectorConstState')
        self.std_ns.class_('vector< boost::shared_ptr<ompl::base::StateSpace> >').rename('vectorStateSpacePtr')
        self.std_ns.class_('map< std::string, std::string>').rename('mapStringToString')
        # don't export variables that need a wrapper
        self.ompl_ns.variables(lambda decl: decl.is_wrapper_needed()).exclude()
        # force StateSpace::allocState to be exported.
        # (not sure why this is necessary)
        allocStateFn = self.ompl_ns.class_('StateSpace').member_function('allocState')
        allocStateFn.include()
        allocStateFn.call_policies = \
            call_policies.return_value_policy(call_policies.reference_existing_object)
        # rename the abstract base class State to AbstractState
        state = self.ompl_ns.class_('State')
        state.rename('AbstractState')
        # don't export components which is of type State**
        state = self.ompl_ns.class_('CompoundState')
        state.variable('components').exclude()
        state.rename('CompoundStateInternal')
        # rename a ScopedState<> to State
        bstate = self.ompl_ns.class_('ScopedState< ompl::base::StateSpace >')
        bstate.rename('State')
        bstate.operator('=', arg_types=['::ompl::base::State const &']).exclude()
        # add array access to double components of state
        self.add_array_access(bstate,'double')
        # loop over all predefined state spaces
        for stype in ['Compound', 'RealVector', 'SO2', 'SO3', 'SE2', 'SE3']:
            # create a python type for each of their corresponding state types
            state = self.ompl_ns.class_('ScopedState< ompl::base::%sStateSpace >' % stype)
            state.rename(stype+'State')
            state.operator('=', arg_types=['::ompl::base::State const &']).exclude()
            # add a constructor that allows, e.g., an SE3State to be constructed from a State
            state.add_registration_code(
                'def(bp::init<ompl::base::ScopedState<ompl::base::StateSpace> const &>(( bp::arg("other") )))')
            # mark the space statetype as 'internal' to emphasize that it
            # shouldn't typically be used by a regular python user
            self.ompl_ns.class_(stype + 'StateSpace').decls('StateType').rename(
                stype + 'StateInternal')
            # add a constructor that allows, e.g., an State to be constructed from a SE3State
            bstate.add_registration_code(
                'def(bp::init<ompl::base::ScopedState<ompl::base::%sStateSpace> const &>(( bp::arg("other") )))' % stype)
            # add array access to double components of state
            self.add_array_access(state,'double')
        # don't this utility function
        self.ompl_ns.member_functions('getValueAddressAtIndex').exclude()
        # don't expose double*
        self.ompl_ns.class_('RealVectorStateSpace').class_(
            'StateType').variable('values').exclude()
        # don't expose std::map< const State *, unsigned int >
        self.ompl_ns.class_('PlannerData').variable('stateIndex').exclude()
        # add array indexing to the RealVectorState
        self.add_array_access(self.ompl_ns.class_('RealVectorStateSpace').class_('StateType'))
        # add array indexing to the EuclideanProjection
        self.add_array_access(self.ompl_ns.class_('EuclideanProjection'))
        # make objects printable that have a print function
        self.replace_member_functions(self.ompl_ns.member_functions('print'))
        # handle special case (abstract base class with pure virtual method)
        self.ompl_ns.class_('Path').add_wrapper_code(
            'virtual void print(std::ostream&) const {}')
        # make settings printable
        self.replace_member_functions(self.ompl_ns.member_functions('printSettings'))
        # make states printable
        self.replace_member_functions(self.ompl_ns.member_functions('printState'))
        # make list of available projections printable
        self.replace_member_functions(self.ompl_ns.member_functions('printProjections'))
        # make projections projections
        self.replace_member_functions(self.ompl_ns.member_functions('printProjection'))
        # add wrapper code for setStateValidityChecker
        self.replace_member_functions(self.ompl_ns.namespace('base').class_(
            'SpaceInformation').member_functions('setStateValidityChecker',
            arg_types=['::ompl::base::StateValidityCheckerFn const &']))
        # add wrapper code for setValidStateSamplerAllocator
        self.replace_member_functions(self.ompl_ns.namespace('base').class_(
                'SpaceInformation').member_functions('setValidStateSamplerAllocator'))
        # exclude solve() methods that take a "const PlannerTerminationConditionFn &"
        # as first argument; only keep the solve() that just takes a double argument
        self.ompl_ns.member_functions('solve', arg_types=['::ompl::base::PlannerTerminationConditionFn const &', 'double']).exclude()

        # wrap the GoalLazySamples constructor so that we can pass a python
        # function that will be converted to a ompl::base::GoalSamplingFn.
        # THIS IS BROKEN. Uncomment the code below when the thread issues are fixed.
        # self.ompl_ns.class_('GoalLazySamples').add_declaration_code("""
        # struct GoalLazySamplesConstructorPyWrapper
        # {
        #     GoalLazySamplesConstructorPyWrapper( bp::object callable ) : callable_( callable ) {}
        # 
        #     bool operator()(const ompl::base::GoalLazySamples* gls, ompl::base::State* s) const
        #     {
        #         PyGILState_STATE gstate = PyGILState_Ensure();
        #         std::cerr<<"."<<std::endl;
        #         bool ret = bp::extract<bool>(callable_(bp::ptr(gls), bp::ptr(s)));
        #         PyGILState_Release( gstate );
        #         return ret;
        #     }
        # 
        #     bp::object callable_;
        # };
        # 
        # static boost::shared_ptr<GoalLazySamples_wrapper> 
        # GoalLazySamples_constructorWrapper(const ompl::base::SpaceInformationPtr& si, 
        #     bp::object samplerFunc, bool autoStart, double epsilon)
        # {
        #     std::cerr<<"x"<<std::endl;
        #     return boost::shared_ptr<GoalLazySamples_wrapper>(new GoalLazySamples_wrapper(si,
        #         ompl::base::GoalSamplingFn(GoalLazySamplesConstructorPyWrapper(samplerFunc)), 
        #         autoStart, epsilon));
        # }
        # """)
        # self.ompl_ns.class_('GoalLazySamples').add_registration_code(
        #    'def("__init__", bp::make_constructor(GoalLazySamples_constructorWrapper))')

class ompl_control_generator_t(code_generator_t):
    def __init__(self):
        replacement = default_replacement
        # A C++ call like "foo.printControl(control, std::cout)" will be replaced with
        # something more pythonesque: "print foo.string(control)"
        replacement['printControl'] = ('def("string", &__printControl)', """
        std::string __printControl(%s* space, ompl::control::Control* control)
        {
            std::ostringstream s;
            space->printControl(control, s);
            return s.str();
        }
        """)
        # add a wrapper for the setStateValidityChecker. This wrapper makes a
        # SpaceInformation reference accessible to the validity checker and also deals
        # correctly with C-style pointers.
        replacement['::ompl::control::SimpleSetup::setStateValidityChecker'] = ('def("setStateValidityChecker", &setStateValidityCheckerWrapper)', """
        struct IsValidFunPyWrapper
        {
            IsValidFunPyWrapper( bp::object callable ) : callable_( callable ) {}

            bool operator()(const ompl::control::SpaceInformation* si, const ompl::base::State* state)
            {
                PyGILState_STATE gstate = PyGILState_Ensure();
                bool ret = bp::extract<bool>(callable_(bp::ptr(si), bp::ptr(state)));
                PyGILState_Release( gstate );
                return ret;
            }

            bp::object callable_;
        };

        void setStateValidityCheckerWrapper(%s* obj, bp::object function)
        {
            obj->setStateValidityChecker( boost::bind(
            boost::function<bool (const ompl::control::SpaceInformation*, const ompl::base::State*)>(IsValidFunPyWrapper(function)),
                obj->getSpaceInformation().get(), _1));
        }
        """)
        # add a wrapper for the setPropagationFunction. This wrapper deals correctly
        # with C-style pointers.
        replacement['setPropagationFunction'] = ('def("setPropagationFunction", &setPropagationFunctionWrapper)', """
        struct PropagatePyWrapper
        {
            PropagatePyWrapper( bp::object callable ) : callable_( callable ) {}

            void operator()(const ompl::control::ControlSpace* cspace, const ompl::base::State* start, const ompl::control::Control* control, const double duration, ompl::base::State* result)
            {
                PyGILState_STATE gstate = PyGILState_Ensure();
                callable_(bp::ptr(cspace), bp::ptr(start), bp::ptr(control), duration, bp::ptr(result));
                PyGILState_Release( gstate );
            }

            bp::object callable_;
        };

        void setPropagationFunctionWrapper(%s* obj, bp::object function)
        {
            obj->setPropagationFunction( boost::bind(
            boost::function<void (const ompl::control::ControlSpace*, const ompl::base::State*, const ompl::control::Control*, const double, ompl::base::State*)>(PropagatePyWrapper(function)),
                obj, _1, _2, _3, _4));
        }
        """)

        code_generator_t.__init__(self, 'control', ['bindings/base', 'bindings/geometric'], replacement)

    def filter_declarations(self):
        code_generator_t.filter_declarations(self)
        # rename STL vectors of certain types
        self.std_ns.class_('vector< int >').rename('vectorInt')
        self.std_ns.class_('vector< double >').rename('vectorDouble')
        self.std_ns.class_('vector< ompl::control::Control* >').rename('vectorControlPtr')
        # don't export variables that need a wrapper
        self.ompl_ns.variables(lambda decl: decl.is_wrapper_needed()).exclude()
        # force ControlSpace::allocState to be exported.
        # (not sure why this is necessary)
        allocControlFn = self.ompl_ns.class_('ControlSpace').member_function('allocControl')
        allocControlFn.include()
        allocControlFn.call_policies = \
            call_policies.return_value_policy(call_policies.reference_existing_object)
        # don't export components which is of type Control**
        self.ompl_ns.class_('CompoundControl').variable('components').exclude()
        # don't export some internal data structure
        self.ompl_ns.class_('OrderCellsByImportance').exclude()
        # don't expose this utility function
        self.ompl_ns.member_functions('getValueAddressAtIndex').exclude()
        self.ompl_ns.class_('KPIECE1').member_functions('freeGridMotions').exclude()
        # add array indexing to the RealVectorState
        self.add_array_access(self.ompl_ns.class_('RealVectorControlSpace').class_('ControlType'))
        # make objects printable that have a print function
        self.replace_member_functions(self.ompl_ns.member_functions('print'))
        # make settings printable
        self.replace_member_functions(self.ompl_ns.member_functions('printSettings'))
        # make controls printable
        self.replace_member_functions(self.ompl_ns.member_functions('printControl'))
        # add wrapper code for setStateValidityChecker
        self.replace_member_functions(self.ompl_ns.namespace('control').class_(
            'SimpleSetup').member_functions('setStateValidityChecker',
            arg_types=['::ompl::base::StateValidityCheckerFn const &']))
        # add wrapper code for setPropagationFunction
        self.replace_member_functions(self.ompl_ns.namespace('control').class_(
            'ControlSpace').member_functions('setPropagationFunction'))
        # LLVM's clang++ compiler doesn't like exporting this method because
        # the argument type (Grid::Cell) is protected
        self.ompl_ns.member_functions('computeImportance').exclude()
        # exclude solve() methods that take a "const PlannerTerminationCondition &"
        # as first argument; only keep the solve() that just takes a double argument
        self.ompl_ns.member_functions('solve', arg_types=['::ompl::base::PlannerTerminationCondition const &']).exclude()

        # do this for all classes that exist with the same name in another namespace
        for cls in ['SimpleSetup', 'KPIECE1', 'RRT']:
            self.ompl_ns.class_(cls).wrapper_alias = 'Control%s_wrapper' % cls

        # Py++ seems to get confused by virtual methods declared in one module
        # that are *not* overridden in a derived class in another module. The
        # Planner class is defined in ompl::base and two of its virtual methods,
        # setProblemDefinition and checkValidity, and not overridden by most
        # planners. The code below forces Py++ to do the right thing (or at
        # least make it work). It seems rather hacky and there may be a better
        # solution.

        # do this for all planners
        for planner in ['KPIECE1', 'RRT']:
            self.ompl_ns.class_(planner).add_registration_code("""
            def("setProblemDefinition",&::ompl::base::Planner::setProblemDefinition,
                    &Control%s_wrapper::default_setProblemDefinition, (bp::arg("pdef")) )""" % planner)
            self.ompl_ns.class_(planner).add_registration_code("""
            def("checkValidity",&::ompl::base::Planner::checkValidity,
                    &Control%s_wrapper::default_checkValidity )""" % planner)

class ompl_geometric_generator_t(code_generator_t):
    def __init__(self):
        replacement = default_replacement
        # add a wrapper for the
        # ompl::geometric::SimpleSetup::setStateValidityChecker. This wrapper makes a
        # SpaceInformation reference accessible to the validity checker and also deals
        # correctly with C-style pointers.
        replacement['::ompl::geometric::SimpleSetup::setStateValidityChecker'] = ('def("setStateValidityChecker", &setStateValidityCheckerWrapper)', """
        struct IsValidFunPyWrapper
        {
            IsValidFunPyWrapper( bp::object callable ) : callable_( callable ) {}

            bool operator()(const ompl::base::SpaceInformation* si, const ompl::base::State* state)
            {
                PyGILState_STATE gstate = PyGILState_Ensure();
                bool ret = bp::extract<bool>(callable_(bp::ptr(si), bp::ptr(state)));
                PyGILState_Release( gstate );
                return ret;
            }

            bp::object callable_;
        };

        void setStateValidityCheckerWrapper(%s* obj, bp::object function)
        {
            obj->setStateValidityChecker( boost::bind(
            boost::function<bool (const ompl::base::SpaceInformation*, const ompl::base::State*)>(IsValidFunPyWrapper(function)),
                obj->getSpaceInformation().get(), _1));
        }
        """)
        code_generator_t.__init__(self, 'geometric', ['bindings/base'], replacement)

    def filter_declarations(self):
        code_generator_t.filter_declarations(self)
        # rename STL vectors of certain types
        self.std_ns.class_('vector< int >').rename('vectorInt')
        self.std_ns.class_('vector< double >').rename('vectorDouble')
        self.std_ns.class_('vector< ompl::geometric::BasicPRM::Milestone* >').rename('vectorBasicPRMMileStonePtr')

        # don't export variables that need a wrapper
        self.ompl_ns.variables(lambda decl: decl.is_wrapper_needed()).exclude()
        # make objects printable that have a print function
        self.replace_member_functions(self.ompl_ns.member_functions('print'))
        self.ompl_ns.member_functions('freeGridMotions').exclude()
        self.ompl_ns.class_('BasicPRM').member_functions('haveSolution').exclude()
        self.ompl_ns.class_('BasicPRM').member_functions('growRoadmap',
                function=declarations.access_type_matcher_t('protected')).exclude()
        # don't export some internal data structure
        self.ompl_ns.classes('OrderCellsByImportance').exclude()
        # LLVM's clang++ compiler doesn't like exporting this method because
        # the argument type (Grid::Cell) is protected
        self.ompl_ns.member_functions('computeImportance').exclude()
        # add wrapper code for setStateValidityChecker
        self.replace_member_functions(self.ompl_ns.namespace('geometric').class_(
            'SimpleSetup').member_functions('setStateValidityChecker',
            arg_types=['::ompl::base::StateValidityCheckerFn const &']))
        # exclude solve() methods that take a "const PlannerTerminationCondition &"
        # as first argument; only keep the solve() that just takes a double argument
        self.ompl_ns.member_functions('solve', arg_types=['::ompl::base::PlannerTerminationCondition const &']).exclude()

        # Py++ seems to get confused by virtual methods declared in one module
        # that are *not* overridden in a derived class in another module. The
        # Planner class is defined in ompl::base and two of its virtual methods,
        # setProblemDefinition and checkValidity, and not overridden by most
        # planners. The code below forces Py++ to do the right thing (or at
        # least make it work). It seems rather hacky and there may be a better
        # solution.

        # do this for all planners
        for planner in ['EST', 'KPIECE1', 'BKPIECE1', 'LBKPIECE1', 'BasicPRM', 'LazyRRT', 'pRRT', 'RRT', 'RRTConnect', 'pSBL', 'SBL']:
            if planner!='BasicPRM':
                # BasicPRM overrides setProblemDefinition, so we don't need to add this code
                self.ompl_ns.class_(planner).add_registration_code("""
                def("setProblemDefinition",&::ompl::base::Planner::setProblemDefinition,
                    &%s_wrapper::default_setProblemDefinition, (bp::arg("pdef")) )""" % planner)
            self.ompl_ns.class_(planner).add_registration_code("""
            def("checkValidity",&::ompl::base::Planner::checkValidity,
                &%s_wrapper::default_checkValidity )""" % planner)


class ompl_util_generator_t(code_generator_t):
    def __init__(self):
        code_generator_t.__init__(self, 'util')


if __name__ == '__main__':
    setrecursionlimit(50000)
    if len(argv)==1:
        print "Usage: generatebindings.py <modulename>"
    else:
        for module in argv[1:]:
            try:
                globals()['ompl_'+module+'_generator_t']()
            except KeyError:
                print "Error: can't generate code for module ", module
