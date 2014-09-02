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
        # something more pythonesque: "print(foo.string(state))"
        replacement['printState'] = ('def("string", &__printState)', """
        std::string __printState(%s* space, ompl::base::State* state)
        {
            std::ostringstream s;
            space->printState(state, s);
            return s.str();
        }
        """)
        # A C++ call like "foo.printProperties(std::cout)" will be replaced with
        # something more pythonesque: "print(foo.properties())"
        default_replacement['printProperties'] = ('def("properties", &__printProperties)', """
        std::string __printProperties(%s* obj)
        {
            std::ostringstream s;
            obj->printProperties(s);
            return s.str();
        }
        """)
        # A C++ call like "foo.printProjections(std::cout)" will be replaced with
        # something more pythonesque: "print(foo.projections())"
        replacement['printProjections'] = ('def("projections", &__printProjections)', """
        std::string __printProjections(%s* obj)
        {
            std::ostringstream s;
            obj->printProjections(s);
            return s.str();
        }
        """)
        # A C++ call like "foo.printProjection(projection, std::cout)" will be replaced with
        # something more pythonesque: "print(foo.projection())"
        replacement['printProjection'] = ('def("projection", &__printProjection)', """
        std::string __printProjection(%s* obj, const ompl::base::EuclideanProjection &projection)
        {
            std::ostringstream s;
            obj->printProjection(projection, s);
            return s.str();
        }
        """)
        # "StateSpace::Diagram(std::cout)" will be replaced with
        # something more pythonesque: "print(StateSpace.Diagram())"
        replacement['::ompl::base::StateSpace::Diagram'] = ('def("Diagram", &DiagramWrapper)', """
        std::string DiagramWrapper(%s* obj)
        {
            std::ostringstream s;
            obj->Diagram(s);
            return s.str();
        }
        """)
        # "StateSpace::List(std::cout)" will be replaced with
        # something more pythonesque: "print(StateSpace.List())"
        replacement['::ompl::base::StateSpace::List'] = ('def("List", &ListWrapper)', """
        std::string ListWrapper(%s* obj)
        {
            std::ostringstream s;
            obj->List(s);
            return s.str();
        }
        """)
        # "PlannerData::printGraphML(std::cout)" will be replaced with
        # something more pythonesque: "print(PlannerData.printGraphML())"
        replacement['::ompl::base::PlannerData::printGraphML'] = ('def("printGraphML", &__printGraphML)', """
        std::string __printGraphML(%s* obj)
        {
            std::ostringstream s;
            obj->printGraphML(s);
            return s.str();
        }
        """)
        # "PlannerData::printGraphviz(std::cout)" will be replaced with
        # something more pythonesque: "print(PlannerData.printGraphviz())"
        replacement['::ompl::base::PlannerData::printGraphviz'] = ('def("printGraphviz", &__printGraphviz)', """
        std::string __printGraphviz(%s* obj)
        {
            std::ostringstream s;
            obj->printGraphviz(s);
            return s.str();
        }
        """)
        code_generator_t.__init__(self, 'base', ['bindings/util'], replacement)

    def filter_declarations(self):
        # force ProblemDefinition to be included, because it is used by other modules
        self.ompl_ns.class_('ProblemDefinition').include()
        # force the abstract base class Path to be included, because it is used by other modules
        self.ompl_ns.class_('Path').include()
        code_generator_t.filter_declarations(self)
        # rename STL vectors of certain types
        self.std_ns.class_('map< std::string, boost::shared_ptr< ompl::base::ProjectionEvaluator > >').rename('mapStringToProjectionEvaluator')
        self.std_ns.class_('vector< ompl::base::State* >').rename('vectorState')
        try:
            self.std_ns.class_('vector< ompl::base::State const* >').rename('vectorConstState')
        except: pass
        self.std_ns.class_('vector< boost::shared_ptr<ompl::base::StateSpace> >').rename('vectorStateSpacePtr')
        #self.std_ns.class_('vector< <ompl::base::PlannerSolution> >').rename('vectorPlannerSolution')
        self.std_ns.class_('map< std::string, boost::shared_ptr<ompl::base::GenericParam> >').rename('mapStringToGenericParam')
        self.std_ns.class_('map< std::string, ompl::base::StateSpace::SubstateLocation >').rename('mapStringToSubstateLocation')
        self.std_ns.class_('vector<ompl::base::PlannerSolution>').rename('vectorPlannerSolution')
        # rename some templated types
        self.ompl_ns.class_('SpecificParam< bool >').rename('SpecificParamBool')
        self.ompl_ns.class_('SpecificParam< char >').rename('SpecificParamChar')
        self.ompl_ns.class_('SpecificParam< int >').rename('SpecificParamInt')
        self.ompl_ns.class_('SpecificParam< unsigned int >').rename('SpecificParamUint')
        self.ompl_ns.class_('SpecificParam< float >').rename('SpecificParamFloat')
        self.ompl_ns.class_('SpecificParam< double >').rename('SpecificParamDouble')
        self.ompl_ns.class_('SpecificParam< std::string >').rename('SpecificParamString')
        for cls in self.ompl_ns.classes(lambda decl: decl.name.startswith('SpecificParam')):
            cls.constructors().exclude()
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
        for stype in ['Compound', 'RealVector', 'SO2', 'SO3', 'SE2', 'SE3', 'Discrete', 'Time', 'Dubins', 'ReedsShepp']:
            # create a python type for each of their corresponding state types
            state = self.ompl_ns.class_('ScopedState< ompl::base::%sStateSpace >' % stype)
            state.rename(stype+'State')
            state.operator('=', arg_types=['::ompl::base::State const &']).exclude()
            # add a constructor that allows, e.g., an SE3State to be constructed from a State
            state.add_registration_code(
                'def(bp::init<ompl::base::ScopedState<ompl::base::StateSpace> const &>(( bp::arg("other") )))')
            # mark the space statetype as 'internal' to emphasize that it
            # shouldn't typically be used by a regular python user
            if stype!='Dubins' and stype!='ReedsShepp':
                self.ompl_ns.class_(stype + 'StateSpace').decls('StateType').rename(
                    stype + 'StateInternal')
            # add a constructor that allows, e.g., a State to be constructed from a SE3State
            bstate.add_registration_code(
                'def(bp::init<ompl::base::ScopedState<ompl::base::%sStateSpace> const &>(( bp::arg("other") )))' % stype)
            # add array access to double components of state
            self.add_array_access(state,'double')
        # I don't know how to export a C-style array of an enum type
        for stype in ['Dubins', 'ReedsShepp']:
            self.ompl_ns.enumeration(stype + 'PathSegmentType').exclude()
            self.ompl_ns.class_(stype + 'Path').exclude()
            self.ompl_ns.class_(stype + 'StateSpace').member_function(
                stype[0].lower()+stype[1:]).exclude()
        # don't expose these utility functions that return double*
        self.ompl_ns.member_functions('getValueAddressAtIndex').exclude()
        self.ompl_ns.member_functions('getValueAddressAtName').exclude()
        self.ompl_ns.member_functions('getValueAddressAtLocation').exclude()
        # don't export vector<ValueLocation>
        self.ompl_ns.member_functions('getValueLocations').exclude()
        # don't export map<std::string, ValueLocation>
        self.ompl_ns.member_functions('getValueLocationsByName').exclude()
        # don't expose double*
        self.ompl_ns.class_('RealVectorStateSpace').class_(
            'StateType').variable('values').exclude()
        try:
            stateStorage = self.ompl_ns.class_('StateStorage')
            stateStorage.member_function('getStateSamplerAllocatorRange').exclude()
            stateStorage.add_registration_code('def("getStateSamplerAllocatorRange", &ompl::base::StateStorage::getStateSamplerAllocatorRange)')
        except:
            pass

        cls = self.ompl_ns.class_('PlannerStatus')
        # rename to something more memorable than the default Py++ name for
        # the casting operator:
        # as__scope_ompl_scope_base_scope_PlannerStatus_scope_StatusType
        cls.operator(lambda decl: decl.name=='operator ::ompl::base::PlannerStatus::StatusType').rename('getStatus')
        # for python 2.x
        cls.add_registration_code(
            'def("__nonzero__", &ompl::base::PlannerStatus::operator bool)')
        # for python 3.x
        cls.add_registration_code(
            'def("__bool__", &ompl::base::PlannerStatus::operator bool)')

        # Exclude PlannerData::getEdges function that returns a map of PlannerDataEdge* for now
        #self.ompl_ns.class_('PlannerData').member_functions('getEdges').exclude()
        #self.std_ns.class_('map< unsigned int, ompl::base::PlannerDataEdge const*>').include()
        mapUintToPlannerDataEdge_cls = self.std_ns.class_('map< unsigned int, ompl::base::PlannerDataEdge const*>')
        mapUintToPlannerDataEdge_cls.rename('mapUintToPlannerDataEdge')
        mapUintToPlannerDataEdge_cls.indexing_suite.call_policies = \
            call_policies.return_value_policy(call_policies.reference_existing_object)
        # Remove Boost.Graph representation from PlannerData
        self.ompl_ns.class_('PlannerData').member_functions('toBoostGraph').exclude()
        # Make PlannerData printable
        self.replace_member_function(self.ompl_ns.class_('PlannerData').member_function('printGraphviz'))
        self.replace_member_function(self.ompl_ns.class_('PlannerData').member_function('printGraphML'))

        # add array indexing to the RealVectorState
        self.add_array_access(self.ompl_ns.class_('RealVectorStateSpace').class_('StateType'))
        # typedef's are not handled by Py++, so we need to explicitly rename uBLAS vector to EuclideanProjection
        cls = self.mb.namespace('ublas').class_(
            'vector<double, boost::numeric::ublas::unbounded_array<double, std::allocator<double> > >')
        cls.include()
        cls.rename('EuclideanProjection')
        cls.member_functions().exclude()
        cls.operators().exclude()
        self.add_array_access(cls,'double')
        # make objects printable that have a print function
        self.replace_member_functions(self.ompl_ns.member_functions('print'))
        # handle special case (abstract base class with pure virtual method)
        self.ompl_ns.class_('Path').add_wrapper_code(
            'virtual void print(std::ostream&) const {}')
        # make settings printable
        self.replace_member_functions(self.ompl_ns.member_functions('printSettings'))
        # make properties printable
        self.replace_member_functions(self.ompl_ns.member_functions('printProperties'))
        # make states printable
        self.replace_member_functions(self.ompl_ns.member_functions('printState'))
        # make list of available projections printable
        self.replace_member_functions(self.ompl_ns.member_functions('printProjections'))
        # make projections printable
        self.replace_member_functions(self.ompl_ns.member_functions('printProjection'))
        # make state space diagram printable
        self.replace_member_function(self.ompl_ns.class_('StateSpace').member_function('Diagram'))
        # make state space list printable
        self.replace_member_function(self.ompl_ns.class_('StateSpace').member_function('List'))
        # add wrappers for boost::function types
        self.add_boost_function('bool(const ompl::base::GoalLazySamples*, ompl::base::State*)',
            'GoalSamplingFn', 'Goal sampling function')
        self.add_boost_function('void(const ompl::base::State*)',
            'NewStateCallbackFn', 'New state callback function')
        self.add_boost_function('ompl::base::PlannerPtr(const ompl::base::SpaceInformationPtr&)',
            'PlannerAllocator', 'Planner allocator')
        self.add_boost_function('bool()',
            'PlannerTerminationConditionFn','Planner termination condition function')
        self.add_boost_function('bool(const ompl::base::State*)',
            'StateValidityCheckerFn', 'State validity checker function')
        self.add_boost_function('ompl::base::StateSamplerPtr(const ompl::base::StateSpace*)',
            'StateSamplerAllocator', 'State sampler allocator')
        self.add_boost_function('ompl::base::ValidStateSamplerPtr(ompl::base::SpaceInformation const*)',
            'ValidStateSamplerAllocator', 'Valid state allocator function')
        self.add_boost_function('double(const ompl::base::PlannerDataVertex&, const ompl::base::PlannerDataVertex&, const ompl::base::PlannerDataEdge&)',
            'EdgeWeightFn', 'Edge weight function')
        self.add_boost_function('ompl::base::Cost(const ompl::base::State*, const ompl::base::Goal*)',
            'CostToGoHeuristic', 'Cost-to-go heuristic for optimizing planners')
        self.add_boost_function('std::string()', 'PlannerProgressProperty',
            'Function that returns stringified value of a property while a planner is running')

        # rename SamplerSelectors
        self.ompl_ns.class_('SamplerSelector< ompl::base::StateSampler >').rename('StateSamplerSelector')
        self.ompl_ns.class_('SamplerSelector< ompl::base::ValidStateSampler >').rename('ValidStateSamplerSelector')
        try:
            cls = self.ompl_ns.class_('StateStorage').member_functions('load')
            self.ompl_ns.class_('StateStorage').member_function('load', arg_types=['::std::istream &']).exclude()
            self.ompl_ns.class_('StateStorage').member_function('store', arg_types=['::std::ostream &']).exclude()
        except:
            pass

class ompl_control_generator_t(code_generator_t):
    def __init__(self):
        replacement = default_replacement
        # A C++ call like "foo.printControl(control, std::cout)" will be replaced with
        # something more pythonesque: "print(foo.string(control))"
        replacement['printControl'] = ('def("string", &__printControl)', """
        std::string __printControl(%s* space, ompl::control::Control* control)
        {
            std::ostringstream s;
            space->printControl(control, s);
            return s.str();
        }
        """)
        replacement['printAsMatrix'] = ('def("printAsMatrix", &__printAsMatrix)', """
        std::string __printAsMatrix(%s* path)
        {
            std::ostringstream s;
            path->printAsMatrix(s);
            return s.str();
        }
        """)
        replacement['::ompl::control::ODESolver::getStatePropagator'] = ("""
        def("getStatePropagator", &getStatePropagator1);
        ODESolver_exposer.def("getStatePropagator", &getStatePropagator2);
        ODESolver_exposer.staticmethod( "getStatePropagator" )""", """
        // %s
        ompl::control::StatePropagatorPtr getStatePropagator2(ompl::control::ODESolverPtr solver,
            const ompl::control::ODESolver::PostPropagationEvent &postEvent)
        {
            return ompl::control::ODESolver::getStatePropagator(solver, postEvent);
        }
        ompl::control::StatePropagatorPtr getStatePropagator1(ompl::control::ODESolverPtr solver)
        {
            return ompl::control::ODESolver::getStatePropagator(solver);
        }
        """)
        code_generator_t.__init__(self, 'control', ['bindings/util', 'bindings/base', 'bindings/geometric'], replacement)

    def filter_declarations(self):
        code_generator_t.filter_declarations(self)
        # rename STL vectors of certain types
        self.std_ns.class_('vector< ompl::control::Control* >').rename('vectorControlPtr')
        # don't export variables that need a wrapper
        self.ompl_ns.variables(lambda decl: decl.is_wrapper_needed()).exclude()
        # force ControlSpace::allocControl to be exported.
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
        # print paths as matrices
        self.replace_member_functions(self.ompl_ns.member_functions('printAsMatrix'))
        try:
            # export ODESolver-derived classes that use Boost.OdeInt
            for odesolver in ['ODEBasicSolver', 'ODEErrorSolver', 'ODEAdaptiveSolver']:
                self.ompl_ns.class_(lambda cls: cls.name.startswith(odesolver)).rename(odesolver)
            # Somehow, Py++ changes the type of the ODE's first argument. Weird...
            self.add_boost_function('void(ompl::control::ODESolver::StateType, const ompl::control::Control*, ompl::control::ODESolver::StateType &)',
                'ODE','Ordinary differential equation')
            # workaround for default argument for PostPropagationEvent
            self.replace_member_function(self.ompl_ns.class_('ODESolver').member_function(
                'getStatePropagator'))
        except declarations.matcher.declaration_not_found_t:
            # not available for boost < 1.44, so ignore this
            pass
        # LLVM's clang++ compiler doesn't like exporting this method because
        # the argument type (Grid::Cell) is protected
        self.ompl_ns.member_functions('computeImportance').exclude()

        # export pure virtual member functions, otherwise code doesn't compile
        syclop = self.ompl_ns.class_('Syclop')
        syclop.add_wrapper_code("""
        virtual ompl::control::Syclop::Motion* addRoot(const ompl::base::State* s)
        {
            bp::override func_addRoot = this->get_override("addRoot");
            return func_addRoot(s);
        }
        virtual void selectAndExtend(ompl::control::Syclop::Region& region, std::vector<ompl::control::Syclop::Motion*>& newMotions)
        {
            bp::override func_selectAndExtend = this->get_override("selectAndExtend");
            func_selectAndExtend(region, newMotions);
        }""")
        # omit ompl::control::Syclop::Defaults nested subclass, otherwise
        # code doesn't compile (don't know why)
        syclop.class_('Defaults').exclude()

        # add wrappers for boost::function types
        self.add_boost_function('ompl::control::ControlSamplerPtr(const ompl::control::ControlSpace*)',
            'ControlSamplerAllocator', 'Control sampler allocator')
        self.add_boost_function('ompl::control::DirectedControlSamplerPtr(const ompl::control::SpaceInformation*)',
            'DirectedControlSamplerAllocator','Directed control sampler allocator')
        # same type as StatePropagatorFn, so no need to export this. Instead, we just define a type alias in the python module.
        #self.add_boost_function('void(const ompl::base::State*, const ompl::control::Control*, const double, ompl::base::State*)',
        #    'PostPropagationEvent','Post-propagation event')
        self.add_boost_function('void(const ompl::base::State*, const ompl::control::Control*, const double, ompl::base::State*)',
            'StatePropagatorFn','State propagator function')
        self.add_boost_function('double(int, int)','EdgeCostFactorFn',
            'Syclop edge cost factor function')
        self.add_boost_function('void(int, int, std::vector<int>&)','LeadComputeFn',
            'Syclop lead compute function')
        # code generation fails because of same bug in gxxcml that requires us
        # to patch the generated code with workaround_for_gccxml_bug.cmake
        self.ompl_ns.member_functions('getPlannerAllocator').exclude()
        self.ompl_ns.member_functions('setPlannerAllocator').exclude()
        self.ompl_ns.namespace('control').class_('SimpleSetup').add_registration_code(
            'def("setPlannerAllocator", &ompl::control::SimpleSetup::setPlannerAllocator)')
        self.ompl_ns.namespace('control').class_('SimpleSetup').add_registration_code(
            'def("getPlannerAllocator", &ompl::control::SimpleSetup::getPlannerAllocator, bp::return_value_policy< bp::copy_const_reference >())')

        # do this for all classes that exist with the same name in another namespace
        for cls in ['SimpleSetup', 'KPIECE1', 'PDST', 'RRT', 'EST', 'SpaceInformation', 'Syclop', 'SyclopEST', 'SyclopRRT']:
            self.ompl_ns.namespace('control').class_(cls).wrapper_alias = 'Control%s_wrapper' % cls

        # Py++ seems to get confused by some methods declared in one module
        # that are *not* overridden in a derived class in another module. The
        # Planner class is defined in ompl::base and two of its virtual methods,
        # setProblemDefinition and checkValidity, and not overridden by most
        # planners. The code below forces Py++ to do the right thing (or at
        # least make it work). It seems rather hacky and there may be a better
        # solution.

        # do this for all planners
        for planner in ['KPIECE1', 'PDST', 'RRT', 'EST', 'Syclop', 'SyclopEST', 'SyclopRRT']:
            self.ompl_ns.class_(planner).add_registration_code("""
            def("solve", (::ompl::base::PlannerStatus(::ompl::base::Planner::*)( double ))(&::ompl::base::Planner::solve), (bp::arg("solveTime")) )""")
            self.ompl_ns.class_(planner).add_registration_code("""
            def("setProblemDefinition",&::ompl::base::Planner::setProblemDefinition,
                    &Control%s_wrapper::default_setProblemDefinition, (bp::arg("pdef")) )""" % planner)
            self.ompl_ns.class_(planner).add_registration_code("""
            def("checkValidity",&::ompl::base::Planner::checkValidity,
                    &Control%s_wrapper::default_checkValidity )""" % planner)

class ompl_geometric_generator_t(code_generator_t):
    def __init__(self):
        replacement = default_replacement
        replacement['printAsMatrix'] = ('def("printAsMatrix", &__printAsMatrix)', """
        std::string __printAsMatrix(%s* path)
        {
            std::ostringstream s;
            path->printAsMatrix(s);
            return s.str();
        }
        """)
        replacement['printDebug'] = ('def("printDebug", &__printDebug)', """
        std::string __printDebug(%s* obj)
        {
            std::ostringstream s;
            obj->printDebug(s);
            return s.str();
        }
        """)
        code_generator_t.__init__(self, 'geometric', ['bindings/util', 'bindings/base'], replacement)

    def filter_declarations(self):
        code_generator_t.filter_declarations(self)
        # don't export variables that need a wrapper
        self.ompl_ns.variables(lambda decl: decl.is_wrapper_needed()).exclude()
        # make objects printable that have a print function
        self.replace_member_functions(self.ompl_ns.member_functions('print'))
        # print paths as matrices
        self.replace_member_functions(self.ompl_ns.member_functions('printAsMatrix'))
        # print debug info
        self.replace_member_functions(self.ompl_ns.member_functions('printDebug'))
        self.ompl_ns.member_functions('freeGridMotions').exclude()
        self.ompl_ns.class_('PRM').member_functions('haveSolution').exclude()
        self.ompl_ns.class_('PRM').member_functions('growRoadmap',
                function=declarations.access_type_matcher_t('protected')).exclude()
        self.ompl_ns.class_('PRM').member_functions('expandRoadmap',
                function=declarations.access_type_matcher_t('protected')).exclude()
        # don't export some internal data structure
        self.ompl_ns.classes('OrderCellsByImportance').exclude()
        # LLVM's clang++ compiler doesn't like exporting this method because
        # the argument type (Grid::Cell) is protected
        self.ompl_ns.member_functions('computeImportance').exclude()
        # add wrappers for boost::function types
        self.add_boost_function('unsigned int()',
            'NumNeighborsFn', 'Number of neighbors function')
        # self.add_boost_function('std::vector<ompl::geometric::PRM::Vertex>&(const ompl::geometric::PRM::Vertex)',
        #     'ConnectionStrategy', 'Connection strategy')
        self.add_boost_function('bool(const ompl::geometric::PRM::Vertex&, const ompl::geometric::PRM::Vertex&)',
            'ConnectionFilter', 'Connection filter')
        # code generation fails because of same bug in gxxcml that requires us
        # to patch the generated code with workaround_for_gccxml_bug.cmake
        self.ompl_ns.member_functions('getPlannerAllocator').exclude()
        self.ompl_ns.member_functions('setPlannerAllocator').exclude()
        self.ompl_ns.namespace('geometric').class_('SimpleSetup').add_registration_code(
            'def("setPlannerAllocator", &ompl::geometric::SimpleSetup::setPlannerAllocator)')
        self.ompl_ns.namespace('geometric').class_('SimpleSetup').add_registration_code(
            'def("getPlannerAllocator", &ompl::geometric::SimpleSetup::getPlannerAllocator, bp::return_value_policy< bp::copy_const_reference >())')

        # The OMPL implementation of PRM uses two threads: one for constructing
        # the roadmap and another for checking for a solution. This causes
        # problems when both threads try to access the python interpreter
        # simultaneously. This is a known limitation of Boost.Python. We
        # therefore use a single-threaded version of PRM in python.
        PRM_cls = self.ompl_ns.class_('PRM')
        PRM_cls.member_function('solve').exclude()
        PRM_cls.add_wrapper_code("""
            virtual ::ompl::base::PlannerStatus solve( ::ompl::base::PlannerTerminationCondition const & ptc ) {
                if( bp::override func_solve = this->get_override( "solve" ) )
                    return func_solve( boost::ref(ptc) );
                else{
                    return default_solve( boost::ref(ptc) );
                }
            }

            ::ompl::base::PlannerStatus default_solve( ::ompl::base::PlannerTerminationCondition const & ptc );
            """)
        PRM_cls.add_declaration_code(open('PRM.SingleThreadSolve.cpp','r').read())
        PRM_cls.add_registration_code("""def("solve",
            (::ompl::base::PlannerStatus(::ompl::geometric::PRM::*)( ::ompl::base::PlannerTerminationCondition const &))(&PRM_wrapper::solve),
            (::ompl::base::PlannerStatus(PRM_wrapper::*)( ::ompl::base::PlannerTerminationCondition const & ))(&PRM_wrapper::default_solve), bp::arg("ptc") )""")
        # exclude PRM*, define it in python to use the single-threaded version
        # of PRM with the k* connection strategy
        self.ompl_ns.class_('PRMstar').exclude()

        # Py++ seems to get confused by some methods declared in one module
        # that are *not* overridden in a derived class in another module. The
        # Planner class is defined in ompl::base and two of its virtual methods,
        # setProblemDefinition and checkValidity, and not overridden by most
        # planners. The code below forces Py++ to do the right thing (or at
        # least make it work). It seems rather hacky and there may be a better
        # solution.

        # do this for all planners
        for planner in ['EST', 'KPIECE1', 'BKPIECE1', 'LBKPIECE1', 'PRM', 'PDST', 'LazyRRT', 'RRT', 'RRTConnect', 'TRRT', 'RRTstar', 'LBTRRT', 'SBL', 'SPARS', 'SPARStwo', 'STRIDE', 'FMT']:
            self.ompl_ns.class_(planner).add_registration_code("""
            def("solve", (::ompl::base::PlannerStatus(::ompl::base::Planner::*)( double ))(&::ompl::base::Planner::solve), (bp::arg("solveTime")) )""")
            if planner!='PRM':
                # PRM overrides setProblemDefinition, so we don't need to add this code
                self.ompl_ns.class_(planner).add_registration_code("""
                def("setProblemDefinition",&::ompl::base::Planner::setProblemDefinition,
                    &%s_wrapper::default_setProblemDefinition, (bp::arg("pdef")) )""" % planner)
            self.ompl_ns.class_(planner).add_registration_code("""
            def("checkValidity",&::ompl::base::Planner::checkValidity,
                &%s_wrapper::default_checkValidity )""" % planner)

        # needed to able to set connection strategy for PRM
        # the PRM::Vertex type is typedef-ed to boost::graph_traits<Graph>::vertex_descriptor. This can
        # be equal to an unsigned long or unsigned int, depending on architecture (or version of boost?)
        try:
            self.ompl_ns.class_('NearestNeighbors<unsigned long>').include()
            self.ompl_ns.class_('NearestNeighbors<unsigned long>').rename('NearestNeighbors')
            self.ompl_ns.class_('NearestNeighborsLinear<unsigned long>').rename('NearestNeighborsLinear')
            self.ompl_ns.class_('KStrategy<unsigned long>').rename('KStrategy')
            self.ompl_ns.class_('KStarStrategy<unsigned long>').rename('KStarStrategy')
        except:
            self.ompl_ns.class_('NearestNeighbors<unsigned int>').include()
            self.ompl_ns.class_('NearestNeighbors<unsigned int>').rename('NearestNeighbors')
            self.ompl_ns.class_('NearestNeighborsLinear<unsigned int>').rename('NearestNeighborsLinear')
            self.ompl_ns.class_('KStrategy<unsigned int>').rename('KStrategy')
            self.ompl_ns.class_('KStarStrategy<unsigned int>').rename('KStarStrategy')

class ompl_tools_generator_t(code_generator_t):
    def __init__(self):
        replacement = default_replacement
        replacement['::ompl::tools::Benchmark::benchmark'] = ('def("benchmark", &benchmarkWrapper)', """
        void benchmarkWrapper(%s* obj, const ompl::tools::Benchmark::Request& request)
        {
            ompl::tools::Benchmark::Request req(request);
            req.useThreads = false;
            obj->benchmark(request);
        }
        """)

        code_generator_t.__init__(self, 'tools',
            ['bindings/util', 'bindings/base', 'bindings/geometric', 'bindings/control'], replacement, 1)
    def filter_declarations(self):
        code_generator_t.filter_declarations(self)
        # rename STL vectors/maps of certain types
        self.std_ns.class_('vector< ompl::tools::Benchmark::PlannerExperiment >').rename('vectorPlannerExperiment')
        self.std_ns.class_('vector< std::vector< std::map<std::string, std::string> > >').rename('vectorRunProgressData')
        # make objects printable that have a print function
        self.replace_member_functions(self.ompl_ns.member_functions('print'))

        benchmark_cls = self.ompl_ns.class_('Benchmark')
        self.replace_member_function(benchmark_cls.member_function('benchmark'))
        # next five statements take care of weird error in default value for name argument in constructor
        benchmark_cls.constructors().exclude()
        benchmark_cls.add_registration_code(
            'def(bp::init< ompl::geometric::SimpleSetup &, bp::optional< std::string const & > >(( bp::arg("setup"), bp::arg("name")=std::basic_string<char, std::char_traits<char>, std::allocator<char> >() )) )')
        benchmark_cls.add_wrapper_code(
            """Benchmark_wrapper(::ompl::geometric::SimpleSetup & setup, const ::std::string & name=std::string() )
        : ompl::tools::Benchmark( boost::ref(setup), name )
          , bp::wrapper< ompl::tools::Benchmark >(){}""")
        benchmark_cls.add_registration_code(
            'def(bp::init< ompl::control::SimpleSetup &, bp::optional< std::string const & > >(( bp::arg("setup"), bp::arg("name")=std::basic_string<char, std::char_traits<char>, std::allocator<char> >() )) )')
        benchmark_cls.add_wrapper_code(
            """Benchmark_wrapper(::ompl::control::SimpleSetup & setup, const ::std::string & name=std::string() )
          : ompl::tools::Benchmark( boost::ref(setup), name )
            , bp::wrapper< ompl::tools::Benchmark >(){}""")
        # don't want to export iostream
        benchmark_cls.member_function('saveResultsToStream').exclude()
        # code generation fails because of same bug in gxxcml that requires us
        # to patch the generated code with workaround_for_gccxml_bug.cmake
        self.ompl_ns.member_functions('addPlannerAllocator').exclude()
        benchmark_cls.member_functions(lambda method: method.name.startswith('set') and method.name.endswith('Event')).exclude()
        benchmark_cls.add_registration_code(
            'def("addPlannerAllocator", &ompl::tools::Benchmark::addPlannerAllocator)')
        self.ompl_ns.class_('OptimizePlan').add_registration_code(
            'def("addPlannerAllocator", &ompl::tools::OptimizePlan::addPlannerAllocator)')
        benchmark_cls.add_registration_code(
            'def("setPlannerSwitchEvent", &ompl::tools::Benchmark::setPlannerSwitchEvent)')
        benchmark_cls.add_registration_code(
            'def("setPreRunEvent", &ompl::tools::Benchmark::setPreRunEvent)')
        benchmark_cls.add_registration_code(
            'def("setPostRunEvent", &ompl::tools::Benchmark::setPostRunEvent)')
        self.add_boost_function('void(const ompl::base::PlannerPtr&)',
            'PreSetupEvent', 'Pre-setup event')
        self.add_boost_function('void(const ompl::base::PlannerPtr&, ompl::tools::Benchmark::RunProperties&)',
            'PostSetupEvent', 'Post-setup event')
        benchmark_cls.class_('Request').no_init = False

class ompl_util_generator_t(code_generator_t):
    def __init__(self):
        replacement = default_replacement
        code_generator_t.__init__(self, 'util', None, replacement, 1)
    def filter_declarations(self):
        code_generator_t.filter_declarations(self)
        # rename STL vectors of certain types
        self.std_ns.class_('vector< unsigned long >').include()
        self.std_ns.class_('vector< unsigned long >').rename('vectorSizeT')
        # not needed; causes problems when compiling in C++11 mode
        #self.std_ns.class_('vector< bool >').include()
        #self.std_ns.class_('vector< bool >').rename('vectorBool')
        self.std_ns.class_('vector< int >').include()
        self.std_ns.class_('vector< int >').rename('vectorInt')
        self.std_ns.class_('vector< double >').include()
        self.std_ns.class_('vector< double >').rename('vectorDouble')
        self.std_ns.class_('vector< unsigned int >').include()
        self.std_ns.class_('vector< unsigned int >').rename('vectorUint')
        self.std_ns.class_('vector< std::string >').include()
        self.std_ns.class_('vector< std::string >').rename('vectorString')
        self.std_ns.class_('vector< std::vector<int> >').include()
        self.std_ns.class_('vector< std::vector<int> >').rename('vectorVectorInt')
        self.std_ns.class_('vector< std::vector<unsigned int> >').include()
        self.std_ns.class_('vector< std::vector<unsigned int> >').rename('vectorVectorUint')
        self.std_ns.class_('vector< std::vector<double> >').include()
        self.std_ns.class_('vector< std::vector<double> >').rename('vectorVectorDouble')
        self.std_ns.class_('vector< std::map<std::string, std::string > >').include()
        self.std_ns.class_('vector< std::map<std::string, std::string > >').rename('vectorMapStringToString')
        self.std_ns.class_('map<std::string, std::string >').include()
        self.std_ns.class_('map<std::string, std::string >').rename('mapStringToString')
        self.std_ns.class_('vector< ompl::PPM::Color >').rename('vectorPPMColor')

class ompl_morse_generator_t(code_generator_t):
    def __init__(self):
        replacement = default_replacement
        code_generator_t.__init__(self, 'morse',
            ['bindings/util', 'bindings/base', 'bindings/geometric', 'bindings/control'],
            replacement)
    def filter_declarations(self):
        stype = 'Morse'
        # create a python type for each of the corresponding state type
        state = self.ompl_ns.class_('ScopedState< ompl::base::%sStateSpace >' % stype)
        state.rename(stype+'State')
        state.operator('=', arg_types=['::ompl::base::State const &']).exclude()
        # add a constructor that allows a MorseState to be constructed from a State
        state.add_registration_code(
            'def(bp::init<ompl::base::ScopedState<ompl::base::StateSpace> const &>(( bp::arg("other") )))')
        # add a constructor that allows, e.g., a State to be constructed from a MorseState
        bstate = self.ompl_ns.class_('ScopedState< ompl::base::StateSpace >')
        bstate.add_registration_code(
            'def(bp::init<ompl::base::ScopedState<ompl::base::%sStateSpace> const &>(( bp::arg("other") )))' % stype)
        # add array access to double components of state
        self.add_array_access(state,'double')


if __name__ == '__main__':
    setrecursionlimit(50000)
    if len(argv)==1:
        print("Usage: generatebindings.py <modulename>")
    else:
        for module in argv[1:]:
            try:
                globals()['ompl_'+module+'_generator_t']()
            except KeyError:
                print("Error: can't generate code for module %s" % module)
