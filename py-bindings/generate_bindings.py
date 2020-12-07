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

from os.path import join, dirname
from sys import argv, setrecursionlimit
from pygccxml import declarations
from pygccxml.declarations.runtime_errors import declaration_not_found_t
from pyplusplus.module_builder import call_policies
from pyplusplus import function_transformers as FT
from ompl.bindings_generator import code_generator_t, default_replacement

class ompl_base_generator_t(code_generator_t):
    """Class for generating the ompl.base python module."""

    def __init__(self):
        replacement = default_replacement
        # special case for abstract base class Path with pure virtual print method:
        replacement['::ompl::base::Path::print'] = ('def("__str__", bp::pure_virtual(&__str__))', \
        """std::string __str__(%s* obj)
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
        replacement['printProperties'] = ('def("properties", &__printProperties)', """
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
        std::string __printProjection(%s* obj, const Eigen::Ref<Eigen::VectorXd> &projection)
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
        replacement['::ompl::base::PlannerData::printGraphML'] = \
        ('def("printGraphML", &__printGraphML)', """
        std::string __printGraphML(%s* obj)
        {
            std::ostringstream s;
            obj->printGraphML(s);
            return s.str();
        }
        """)
        # "PlannerData::printGraphviz(std::cout)" will be replaced with
        # something more pythonesque: "print(PlannerData.printGraphviz())"
        replacement['::ompl::base::PlannerData::printGraphviz'] = \
        ('def("printGraphviz", &__printGraphviz)', """
        std::string __printGraphviz(%s* obj)
        {
            std::ostringstream s;
            obj->printGraphviz(s);
            return s.str();
        }
        """)
        # "Atlas::printPLY(std::cout)" will be replaced with
        # something more pythonesque: "print(PlannerData.printGraphviz())"
        replacement['::ompl::base::AtlasStateSpace::printPLY'] = \
        ('def("printPLY", &__printPLY)', """
        std::string __printPLY(%s* obj)
        {
            std::ostringstream s;
            obj->printPLY(s);
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
        self.std_ns.class_(
            'map< std::string, std::shared_ptr< ompl::base::ProjectionEvaluator > >').rename(
                'mapStringToProjectionEvaluator')
        self.std_ns.class_('vector< ompl::base::State * >').rename('vectorState')
        try:
            self.std_ns.class_('vector< ompl::base::State const* >').rename('vectorConstState')
        except declaration_not_found_t:
            pass
        self.std_ns.class_('vector< std::shared_ptr<ompl::base::StateSpace> >').rename(
            'vectorStateSpacePtr')
        #self.std_ns.class_('vector< <ompl::base::PlannerSolution> >').rename(
        # 'vectorPlannerSolution')
        self.std_ns.class_('map< std::string, std::shared_ptr<ompl::base::GenericParam> >').rename(
            'mapStringToGenericParam')
        self.std_ns.class_('map< std::string, ompl::base::StateSpace::SubstateLocation >').rename(
            'mapStringToSubstateLocation')
        self.std_ns.class_('vector<ompl::base::PlannerSolution>').rename('vectorPlannerSolution')

        pairStateDouble = self.std_ns.class_('pair<ompl::base::State *, double>')
        pairStateDouble.rename('pairStateDouble')
        pairStateDouble.include()
        # this operator seems to cause problems with g++-6
        pairStateDouble.operators('=').exclude()
        # rename some templated types
        self.ompl_ns.class_('SpecificParam< bool >').rename('SpecificParamBool')
        self.ompl_ns.class_('SpecificParam< char >').rename('SpecificParamChar')
        self.ompl_ns.class_('SpecificParam< int >').rename('SpecificParamInt')
        self.ompl_ns.class_('SpecificParam< unsigned int >').rename('SpecificParamUint')
        self.ompl_ns.class_('SpecificParam< float >').rename('SpecificParamFloat')
        self.ompl_ns.class_('SpecificParam< double >').rename('SpecificParamDouble')
        self.ompl_ns.class_('SpecificParam< long double >').rename('SpecificParamLongDouble')
        self.ompl_ns.class_(lambda decl: decl.name.startswith('SpecificParam<std::basic_string')).rename(
            'SpecificParamString')
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
        self.add_array_access(bstate, 'double')
        # loop over all predefined state spaces
        spaces = [s.related_class.name.replace('StateSpace', '') \
            for s in self.ompl_ns.class_('StateSpace').recursive_derived]
        for stype in spaces:
            try:
                # create a python type for each of their corresponding state types
                state = self.ompl_ns.class_('ScopedState< ompl::base::%sStateSpace >' % stype)
            except:
                # ignore errors because of missing Boost.Numpy
                continue
            state.rename(stype+'State')
            state.operator('=', arg_types=['::ompl::base::State const &']).exclude()
            # add a constructor that allows, e.g., an SE3State to be constructed from a State
            state.add_registration_code(
                'def(bp::init<ompl::base::ScopedState<ompl::base::StateSpace> const &>' \
                '(( bp::arg("other") )))')
            # mark the space statetype as 'internal' to emphasize that it
            # shouldn't typically be used by a regular python user
            try:
                self.ompl_ns.class_(stype + 'StateSpace').decls('StateType').rename(
                    stype + 'StateInternal')
            except:
                # ignore derived statespaces that do not define their own StateType
                pass
            # add a constructor that allows, e.g., a State to be constructed from a SE3State
            bstate.add_registration_code(
                'def(bp::init<ompl::base::ScopedState<ompl::base::%sStateSpace> const &>' \
                '(( bp::arg("other") )))' % stype)
            # add array access to double components of state
            self.add_array_access(state, 'double')

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
        # exclude member function for which there are multiple signatures
        self.ompl_ns.class_('Goal').member_function(
            'isSatisfied',
            arg_types=['::ompl::base::State const *', 'double *']).exclude()

        # don't expose double*
        self.ompl_ns.class_('RealVectorStateSpace').class_(
            'StateType').variable('values').exclude()
        try:
            stateStorage = self.ompl_ns.class_('StateStorage')
            stateStorage.member_function('getStateSamplerAllocatorRange').exclude()
            stateStorage.add_registration_code('def("getStateSamplerAllocatorRange", ' \
            '&ompl::base::StateStorage::getStateSamplerAllocatorRange)')
        except declaration_not_found_t:
            pass

        cls = self.ompl_ns.class_('PlannerStatus')
        # rename to something more memorable than the default Py++ name for
        # the casting operator:
        # as__scope_ompl_scope_base_scope_PlannerStatus_scope_StatusType
        cls.operator(
            lambda decl: decl.name == 'operator ::ompl::base::PlannerStatus::StatusType').rename(
                'getStatus')
        # for python 2.x
        cls.add_registration_code(
            'def("__nonzero__", &ompl::base::PlannerStatus::operator bool)')
        # for python 3.x
        cls.add_registration_code(
            'def("__bool__", &ompl::base::PlannerStatus::operator bool)')

        # exclude the non-const version of  getProblemDefinition
        self.ompl_ns.member_functions(
            'getProblemDefinition',
            return_type='::ompl::base::ProblemDefinitionPtr &').exclude()

        # Using nullptr as a default value in method arguments causes
        # problems with Boost.Python.
        # See https://github.com/boostorg/python/issues/60
        self.ompl_ns.class_('ProblemDefinition').add_declaration_code('#define nullptr NULL\n')
        try:
            for cls in ['AtlasChart', 'AtlasStateSpace', 'ConstrainedStateSpace', \
                'ProjectedStateSpace', 'TangentBundleStateSpace']:
                self.ompl_ns.class_(cls).add_declaration_code('#define nullptr NULL\n')
            self.ompl_ns.class_('AtlasChart').member_function('toPolygon').exclude()
            self.replace_member_function(self.ompl_ns.class_(
                'AtlasStateSpace').member_function('printPLY'))
            self.add_function_wrapper(
                'double(ompl::base::AtlasChart *)', 'AtlasChartBiasFunction',
                'Bias function for sampling a chart from an atlas.')
                    # add code for numpy.array <-> Eigen conversions
            self.mb.add_declaration_code(open(join(dirname(__file__), \
                'numpy_eigen.cpp'), 'r').read())
            self.mb.add_registration_code("""
                EIGEN_ARRAY_CONVERTER(Eigen::MatrixXd, 2)
                EIGEN_ARRAY_CONVERTER(Eigen::VectorXd, 1)
            """)
            self.mb.add_registration_code('np::initialize();', tail=False)
            self.add_array_access(self.ompl_ns.class_(
                'ConstrainedStateSpace').class_('StateType'), 'double')
            # \todo: figure why commented-out code causes a problem.
            self.ompl_ns.class_('ConstraintIntersection').exclude()
            for cls in [self.ompl_ns.class_('Constraint')]: #,
#                        self.ompl_ns.class_('ConstraintIntersection')]:
                for method in ['function', 'jacobian']:
                    cls.member_function(method, arg_types=[
                        '::Eigen::Ref<const Eigen::Matrix<double, -1, 1, 0, -1, 1>, '
                        '0, Eigen::InnerStride<1> > const &',
                        None]).add_transformation(FT.input(0))
            cls = self.ompl_ns.class_('Constraint')
            for method in ['distance', 'isSatisfied']:
                cls.member_function(method, arg_types=[
                    '::Eigen::Ref<const Eigen::Matrix<double, -1, 1, 0, -1, 1>, '
                    '0, Eigen::InnerStride<1> > const &']).add_transformation(FT.input(0))
        except:
            # python bindings for constrained planning code is only generated
            # if boost.numpy was found
            pass

        # Exclude PlannerData::getEdges function that returns a map of PlannerDataEdge* for now
        #self.ompl_ns.class_('PlannerData').member_functions('getEdges').exclude()
        #self.std_ns.class_('map< unsigned int, ompl::base::PlannerDataEdge const*>').include()
        mapUintToPlannerDataEdge_cls = self.std_ns.class_(
            'map< unsigned int, const ompl::base::PlannerDataEdge *>')
        mapUintToPlannerDataEdge_cls.rename('mapUintToPlannerDataEdge')
        mapUintToPlannerDataEdge_cls.indexing_suite.call_policies = \
            call_policies.return_value_policy(call_policies.reference_existing_object)
        # Remove Boost.Graph representation from PlannerData
        plannerData = self.ompl_ns.class_('PlannerData')
        plannerData.member_functions('toBoostGraph').exclude()
        # Make PlannerData printable
        self.replace_member_function(plannerData.member_function('printGraphviz'))
        self.replace_member_function(plannerData.member_function('printGraphML'))
        # serialize passes archive by reference which causes problems
        self.ompl_ns.class_('PlannerDataVertex').member_functions('serialize').exclude()
        self.ompl_ns.class_('PlannerDataEdge').member_functions('serialize').exclude()

        # add array indexing to the RealVectorState
        self.add_array_access(self.ompl_ns.class_('RealVectorStateSpace').class_('StateType'))
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
        # add wrappers for std::function types
        self.add_function_wrapper('bool(const ompl::base::GoalLazySamples*, ompl::base::State*)', \
            'GoalSamplingFn', 'Goal sampling function')
        self.add_function_wrapper('void(const ompl::base::State*)', \
            'NewStateCallbackFn', 'New state callback function')
        self.add_function_wrapper(
            'ompl::base::PlannerPtr(const ompl::base::SpaceInformationPtr&)',
            'PlannerAllocator', 'Planner allocator')
        self.add_function_wrapper('bool()', \
            'PlannerTerminationConditionFn', 'Planner termination condition function')
        self.add_function_wrapper('bool(const ompl::base::State*)', \
            'StateValidityCheckerFn', 'State validity checker function')
        self.add_function_wrapper('ompl::base::StateSamplerPtr(const ompl::base::StateSpace*)', \
            'StateSamplerAllocator', 'State sampler allocator')
        self.add_function_wrapper(
            'ompl::base::ValidStateSamplerPtr(ompl::base::SpaceInformation const*)',
            'ValidStateSamplerAllocator', 'Valid state allocator function')
        self.add_function_wrapper('double(const ompl::base::PlannerDataVertex&, ' \
            'const ompl::base::PlannerDataVertex&, const ompl::base::PlannerDataEdge&)', \
            'EdgeWeightFn', 'Edge weight function')
        self.add_function_wrapper(
            'ompl::base::Cost(const ompl::base::State*, const ompl::base::Goal*)',
            'CostToGoHeuristic', 'Cost-to-go heuristic for optimizing planners')
        self.add_function_wrapper('std::string()', 'PlannerProgressProperty', \
            'Function that returns stringified value of a property while a planner is running')

        # rename SamplerSelectors
        self.ompl_ns.class_('SamplerSelector< ompl::base::StateSampler >').rename(
            'StateSamplerSelector')
        self.ompl_ns.class_('SamplerSelector< ompl::base::ValidStateSampler >').rename(
            'ValidStateSamplerSelector')
        try:
            cls = self.ompl_ns.class_('StateStorage').member_functions('load')
            self.ompl_ns.class_('StateStorage').member_function('load', \
                arg_types=['::std::istream &']).exclude()
            self.ompl_ns.class_('StateStorage').member_function('store', \
                arg_types=['::std::ostream &']).exclude()
        except declaration_not_found_t:
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
        code_generator_t.__init__(self, 'control', \
            ['bindings/util', 'bindings/base', 'bindings/geometric'], replacement)

    def filter_declarations(self):
        code_generator_t.filter_declarations(self)
        # rename STL vectors of certain types
        self.std_ns.class_('vector< ompl::control::Control * >').rename('vectorControlPtr')
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
        # export ODESolver-derived classes that use Boost.OdeInt
        for odesolver in ['ODEBasicSolver', 'ODEErrorSolver', 'ODEAdaptiveSolver']:
            cls = self.ompl_ns.class_(lambda cls, slv=odesolver: cls.name.startswith(slv))
            cls.rename(odesolver)
            if odesolver == 'ODEAdaptiveSolver':
                cls.include_files.append('boost/numeric/odeint.hpp')
        self.add_function_wrapper(
            'void(const ompl::control::ODESolver::StateType &, const ompl::control::Control*, ' \
            'ompl::control::ODESolver::StateType &)',
            'ODE', 'Ordinary differential equation')
        # workaround for default argument for PostPropagationEvent
        self.replace_member_function(self.ompl_ns.class_('ODESolver').member_function(
            'getStatePropagator'))
        # LLVM's clang++ compiler doesn't like exporting this method because
        # the argument type (Grid::Cell) is protected
        self.ompl_ns.member_functions('computeImportance').exclude()

        # this method requires ompl::Grid::Coord (aka Eigen::VectorXi) to be exported
        self.ompl_ns.class_('KPIECE1').member_function('findNextMotion').exclude()

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

        # add wrappers for std::function types
        self.add_function_wrapper(
            'ompl::control::ControlSamplerPtr(const ompl::control::ControlSpace*)',
            'ControlSamplerAllocator', 'Control sampler allocator')
        self.add_function_wrapper(
            'ompl::control::DirectedControlSamplerPtr(const ompl::control::SpaceInformation*)',
            'DirectedControlSamplerAllocator', 'Directed control sampler allocator')
        self.add_function_wrapper(
            'void(const ompl::base::State*, const ompl::control::Control*, const double, '
            'ompl::base::State*)',
            'StatePropagatorFn', 'State propagator function')
        self.add_function_wrapper('double(int, int)', 'EdgeCostFactorFn', \
            'Syclop edge cost factor function')
        self.add_function_wrapper('void(int, int, std::vector<int>&)', 'LeadComputeFn', \
            'Syclop lead compute function')
        # code generation fails to compile, most likely because of a bug in
        # Py++'s generation of exposed_decl.pypp.txt.
        self.ompl_ns.member_functions('getPlannerAllocator').exclude()
        self.ompl_ns.member_functions('setPlannerAllocator').exclude()
        self.ompl_ns.namespace('control').class_('SimpleSetup').add_registration_code(
            'def("setPlannerAllocator", &ompl::control::SimpleSetup::setPlannerAllocator)')
        self.ompl_ns.namespace('control').class_('SimpleSetup').add_registration_code(
            'def("getPlannerAllocator", &ompl::control::SimpleSetup::getPlannerAllocator, ' \
            'bp::return_value_policy< bp::copy_const_reference >())')

        # Do this for all classes that exist with the same name in another namespace
        # (We also do it for all planners; see below)
        for cls in ['SimpleSetup', 'SpaceInformation']:
            self.ompl_ns.namespace('control').class_(cls).wrapper_alias = 'Control%s_wrapper' % cls

        # exclude the non-const version of  getProblemDefinition
        self.ompl_ns.member_functions(
            'getProblemDefinition',
            return_type='::ompl::base::ProblemDefinitionPtr &').exclude()

        # Py++ seems to get confused by some methods declared in one module
        # that are *not* overridden in a derived class in another module. The
        # Planner class is defined in ompl::base and two of its virtual methods,
        # setProblemDefinition and checkValidity, and not overridden by most
        # planners. The code below forces Py++ to do the right thing (or at
        # least make it work). It seems rather hacky and there may be a better
        # solution.

        planners = [p.related_class for p in self.ompl_ns.class_('Planner').recursive_derived]
        for planner in planners:
            # many planners exist with the same name in another namespace
            planner.wrapper_alias = 'Control%s_wrapper' % planner.name
            planner.add_registration_code(
                'def("solve", (::ompl::base::PlannerStatus(::ompl::base::Planner::*)( double ))' \
                '(&::ompl::base::Planner::solve), (bp::arg("solveTime")) )')
            planner.add_registration_code("""
            def("setProblemDefinition",&::ompl::base::Planner::setProblemDefinition,
                    &Control%s_wrapper::default_setProblemDefinition, (bp::arg("pdef")) )""" % \
                    planner.name)
            planner.add_registration_code("""
            def("checkValidity",&::ompl::base::Planner::checkValidity,
                    &Control%s_wrapper::default_checkValidity )""" % planner.name)

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
        code_generator_t.__init__(self, 'geometric', ['bindings/util', 'bindings/base'], \
            replacement)

    def filter_declarations(self):
        code_generator_t.filter_declarations(self)
        #self.ompl_ns.namespace('util').exclude()
        # don't export variables that need a wrapper
        self.ompl_ns.variables(lambda decl: decl.is_wrapper_needed()).exclude()
        # make objects printable that have a print function
        self.replace_member_functions(self.ompl_ns.member_functions('print'))
        # print paths as matrices
        self.replace_member_functions(self.ompl_ns.member_functions('printAsMatrix'))
        # make settings printable
        self.replace_member_functions(self.ompl_ns.member_functions('printSettings'))
        # print debug info
        self.replace_member_functions(self.ompl_ns.member_functions('printDebug'))
        self.ompl_ns.member_functions('freeGridMotions').exclude()
        self.ompl_ns.class_('PRM').member_functions('maybeConstructSolution').exclude()
        self.ompl_ns.class_('PRM').member_functions('growRoadmap', \
            function=declarations.access_type_matcher_t('protected')).exclude()
        self.ompl_ns.class_('PRM').member_functions('expandRoadmap', \
            function=declarations.access_type_matcher_t('protected')).exclude()
        # don't export some internal data structure
        self.ompl_ns.classes('OrderCellsByImportance').exclude()
        # LLVM's clang++ compiler doesn't like exporting this method because
        # the argument type (Grid::Cell) is protected
        self.ompl_ns.member_functions('computeImportance').exclude()
        # add wrappers for std::function types
        self.add_function_wrapper('unsigned int()', \
            'NumNeighborsFn', 'Number of neighbors function')
        # self.add_function_wrapper(
        # 'std::vector<ompl::geometric::PRM::Vertex>&(const ompl::geometric::PRM::Vertex)',
        #     'ConnectionStrategy', 'Connection strategy')
        self.add_function_wrapper(
            'bool(const ompl::geometric::PRM::Vertex&, const ompl::geometric::PRM::Vertex&)',
            'ConnectionFilter', 'Connection filter')
        # code generation fails to compile, most likely because of a bug in
        # Py++'s generation of exposed_decl.pypp.txt.
        self.ompl_ns.member_functions('getPlannerAllocator').exclude()
        self.ompl_ns.member_functions('setPlannerAllocator').exclude()
        self.ompl_ns.namespace('geometric').class_('SimpleSetup').add_registration_code(
            'def("setPlannerAllocator", &ompl::geometric::SimpleSetup::setPlannerAllocator)')
        self.ompl_ns.namespace('geometric').class_('SimpleSetup').add_registration_code( \
            'def("getPlannerAllocator", &ompl::geometric::SimpleSetup::getPlannerAllocator, ' \
            'bp::return_value_policy< bp::copy_const_reference >())')
        self.std_ns.class_('vector< std::shared_ptr<ompl::geometric::BITstar::Vertex> >').exclude()
        self.std_ns.class_('vector< std::shared_ptr<ompl::geometric::aitstar::Vertex> >').exclude()
        self.std_ns.class_('vector<const ompl::base::State *>').exclude()

        self.std_ns.class_('vector< std::shared_ptr<ompl::base::SpaceInformation> >').rename('vectorSpaceInformation')

        # Using nullptr as a default value in method arguments causes
        # problems with Boost.Python.
        # See https://github.com/boostorg/python/issues/60
        self.ompl_ns.class_('PathSimplifier').add_declaration_code('#define nullptr NULL\n')

        # exclude the non-const version of  getProblemDefinition
        self.ompl_ns.member_functions(
            'getProblemDefinition',
            return_type='::ompl::base::ProblemDefinitionPtr &').exclude()

        # Py++ seems to get confused by some methods declared in one module
        # that are *not* overridden in a derived class in another module. The
        # Planner class is defined in ompl::base and two of its virtual methods,
        # setProblemDefinition and checkValidity, are not overridden by most
        # planners. The code below forces Py++ to do the right thing (or at
        # least make it work). It seems rather hacky and there may be a better
        # solution.
        planners = [p.related_class for p in self.ompl_ns.class_('Planner').recursive_derived]
        for planner in planners:
            planner.add_registration_code(
                'def("solve", (::ompl::base::PlannerStatus(::ompl::base::Planner::*)( double ))' \
                '(&::ompl::base::Planner::solve), (bp::arg("solveTime")) )')
            if planner.name != 'PRM' and planner.name != 'QRRT':
                # PRM and QRRT override setProblemDefinition, so we don't need to add this code
                planner.add_registration_code("""
                def("setProblemDefinition",&::ompl::base::Planner::setProblemDefinition,
                    &%s::default_setProblemDefinition, (bp::arg("pdef")) )""" %
                                              planner.wrapper_alias)
            planner.add_registration_code("""
            def("checkValidity",&::ompl::base::Planner::checkValidity,
                &%s::default_checkValidity )""" % planner.wrapper_alias)

        # The OMPL implementation of PRM uses two threads: one for constructing
        # the roadmap and another for checking for a solution. This causes
        # problems when both threads try to access the python interpreter
        # simultaneously. This is a known limitation of Boost.Python. We
        # therefore use a single-threaded version of PRM in python.
        PRM_cls = self.ompl_ns.class_('PRM')
        PRM_cls.member_function('solve').exclude()
        PRM_cls.add_wrapper_code("""
            virtual ::ompl::base::PlannerStatus solve(
                ::ompl::base::PlannerTerminationCondition const & ptc ) {
                if( bp::override func_solve = this->get_override( "solve" ) )
                    return func_solve( boost::ref(ptc) );
                else{
                    return default_solve( boost::ref(ptc) );
                }
            }

            ::ompl::base::PlannerStatus default_solve(
                ::ompl::base::PlannerTerminationCondition const & ptc );
            """)
        PRM_cls.add_declaration_code(open(join(dirname(__file__), \
            'PRM.SingleThreadSolve.cpp'), 'r').read())
        # This needs to be the last registration code added to the PRM_cls to the ugly hack below.
        PRM_cls.add_registration_code("""def("solve",
            (::ompl::base::PlannerStatus(::ompl::geometric::PRM::*)(
                ::ompl::base::PlannerTerminationCondition const &))(&PRM_wrapper::solve),
            (::ompl::base::PlannerStatus(PRM_wrapper::*)(
                ::ompl::base::PlannerTerminationCondition const & ))(&PRM_wrapper::default_solve), bp::arg("ptc") );

            // HACK ALERT: closing brace destroys bp::scope,
            // so that PRMstar is not a nested class of PRM
            }
            {
                // wrapper for PRMstar, derived from single-threaded PRM_wrapper
                bp::class_<PRMstar_wrapper, bp::bases< PRM_wrapper >, boost::noncopyable >("PRMstar", bp::init< ompl::base::SpaceInformationPtr const & >( bp::arg("si") ) )
            """)
        # Add wrapper code for PRM*
        PRM_cls.add_declaration_code("""
        class PRMstar_wrapper : public PRM_wrapper
        {
        public:
            PRMstar_wrapper(const ompl::base::SpaceInformationPtr &si) : PRM_wrapper(si, true)
            {
                setName("PRMstar");
                params_.remove("max_nearest_neighbors");
            }
        };
        """)
        # LazyPRM's Vertex type is void* so exclude addMilestone which has return type void*
        self.ompl_ns.class_('LazyPRM').member_function('addMilestone').exclude()
        # avoid difficulties in exporting the return type std::vector<base::PlannerDataPtr>
        # do this for all multithreaded planners
        for planner in ['SPARS', 'SPARStwo']:
            cls = self.ompl_ns.class_(planner)
            cls.constructor(arg_types=["::ompl::base::SpaceInformationPtr const &"]).exclude()
            cls.add_registration_code(
                'def(bp::init<ompl::base::SpaceInformationPtr const &>(bp::arg("si")))')
            cls.add_wrapper_code("""
            {0}_wrapper(::ompl::base::SpaceInformationPtr const &si) : ompl::geometric::{0}(si),
                bp::wrapper<ompl::geometric::{0}>()
            {{
                OMPL_WARN("%s: this planner uses multiple threads and might crash if your StateValidityChecker, OptimizationObjective, etc., are allocated within Python.", getName().c_str());
            }}
            """.format(planner))

        # exclude methods that use problematic types
        cls = self.ompl_ns.class_('SPARS')
        cls.member_function('addPathToSpanner').exclude()
        cls.member_function('computeDensePath').exclude()
        self.ompl_ns.class_('SPARStwo').member_function('findCloseRepresentatives').exclude()
        cls = self.ompl_ns.class_('AITstar')
        cls.member_function('getVerticesInQueue').exclude()
        cls.member_function('getVerticesInReverseSearchTree').exclude()

        # needed to able to set connection strategy for PRM
        # the PRM::Vertex type is typedef-ed to boost::graph_traits<Graph>::vertex_descriptor. This
        # can be equal to an unsigned long or unsigned int, depending on architecture (or version
        # of boost?)
        try:
            self.ompl_ns.class_('NearestNeighbors<unsigned long>').include()
            self.ompl_ns.class_('NearestNeighbors<unsigned long>').rename('NearestNeighbors')
            self.ompl_ns.class_('NearestNeighborsLinear<unsigned long>').rename(
                'NearestNeighborsLinear')
            self.ompl_ns.class_('KStrategy<unsigned long>').rename('KStrategy')
            self.ompl_ns.class_('KStarStrategy<unsigned long>').rename('KStarStrategy')
        except declaration_not_found_t:
            self.ompl_ns.class_('NearestNeighbors<unsigned int>').include()
            self.ompl_ns.class_('NearestNeighbors<unsigned int>').rename('NearestNeighbors')
            self.ompl_ns.class_('NearestNeighborsLinear<unsigned int>').rename(
                'NearestNeighborsLinear')
            self.ompl_ns.class_('KStrategy<unsigned int>').rename('KStrategy')
            self.ompl_ns.class_('KStarStrategy<unsigned int>').rename('KStarStrategy')

        try:
            # Exclude some functions from BIT* that cause some Py++ compilation problems
            # (#I don't know why this doesn't work):
            self.ompl_ns.class_('BITstar').member_functions('getEdgeQueue').exclude()
        except declaration_not_found_t:
            pass

class ompl_tools_generator_t(code_generator_t):
    def __init__(self):
        replacement = default_replacement
        replacement['printResultsInfo'] = ('def("printResultsInfo", &__printResultsInfo)', """
        std::string __printResultsInfo(%s* obj)
        {
            std::ostringstream s;
            obj->printResultsInfo(s);
            return s.str();
        }
        """)
        replacement['printLogs'] = ('def("printLogs", &__printLogs)', """
        std::string __printLogs(%s* obj)
        {
            std::ostringstream s;
            obj->printLogs(s);
            return s.str();
        }
        """)
        replacement['saveDataLog'] = ('def("saveDataLog", &__saveDataLog)', """
        std::string __saveDataLog(%s* obj)
        {
            std::ostringstream s;
            obj->saveDataLog(s);
            return s.str();
        }
        """)
        replacement['setPlannerSwitchEvent'] = (
            'def("setPlannerSwitchEvent", &__setPlannerSwitchEvent)', """
        void __setPlannerSwitchEvent(%s* obj, std::function<void(ompl::base::PlannerPtr)> event)
        {
            obj->setPlannerSwitchEvent(event);
        }
        """)
        replacement['setPreRunEvent'] = ('def("setPreRunEvent", &__setPreRunEvent)', """
        void __setPreRunEvent(%s* obj, std::function<void(ompl::base::PlannerPtr)> event)
        {
            obj->setPreRunEvent(event);
        }
        """)
        replacement['setPostRunEvent'] = ('def("setPostRunEvent", &__setPostRunEvent)', """
        void __setPostRunEvent(%s* obj,
        std::function<void(ompl::base::PlannerPtr, ompl::tools::Benchmark::RunProperties &)> event)
        {
            obj->setPostRunEvent(event);
        }
        """)
        replacement['saveResultsToStream'] = ('def("results", &__saveResultsToStream)', """
        std::string __saveResultsToStream(%s* obj)
        {
            std::ostringstream s;
            obj->saveResultsToStream(s);
            return s.str();
        }
        """)

        code_generator_t.__init__(self, 'tools', \
            ['bindings/util', 'bindings/base', 'bindings/geometric', 'bindings/control'], \
            replacement, 1)
    def filter_declarations(self):
        code_generator_t.filter_declarations(self)
        # rename STL vectors/maps of certain types
        self.std_ns.class_('vector< ompl::tools::Benchmark::PlannerExperiment >').rename(
            'vectorPlannerExperiment')
        self.std_ns.class_('vector< std::vector< std::map<std::string, std::string> > >').rename(
            'vectorRunProgressData')
        # make objects printable that have a print function
        self.replace_member_functions(self.ompl_ns.member_functions('print'))

        benchmark_cls = self.ompl_ns.class_('Benchmark')
        self.replace_member_function(benchmark_cls.member_function('saveResultsToStream'))
        for constructor in benchmark_cls.constructors(arg_types=[None, "::std::string const &"]):
            constructor.add_transformation(FT.input(1))

        self.ompl_ns.member_functions('addPlannerAllocator').exclude()
        self.replace_member_functions(benchmark_cls.member_functions(
            lambda method: method.name.startswith('set') and method.name.endswith('Event')))
        benchmark_cls.add_registration_code(
            'def("addPlannerAllocator", &ompl::tools::Benchmark::addPlannerAllocator)')
        self.ompl_ns.class_('OptimizePlan').add_registration_code(
            'def("addPlannerAllocator", &ompl::tools::OptimizePlan::addPlannerAllocator)')
        self.add_function_wrapper('void(const ompl::base::PlannerPtr)', \
            'PreSetupEvent', 'Pre-setup event')
        self.add_function_wrapper(
            'void(ompl::base::PlannerPtr, ompl::tools::Benchmark::RunProperties&)',
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
        self.std_ns.class_('vector< std::map<std::string, std::string > >').rename(
            'vectorMapStringToString')
        self.std_ns.class_('map<std::string, std::string >').include()
        self.std_ns.class_('map<std::string, std::string >').rename('mapStringToString')
        self.std_ns.class_('vector< ompl::PPM::Color >').rename('vectorPPMColor')
        try:
            # Exclude the ProlateHyperspheroid Class which needs Eigen, and the associated member
            # functions in the RNG
            self.ompl_ns.class_('ProlateHyperspheroid').exclude()
            self.ompl_ns.class_('RNG').member_functions(
                'uniformProlateHyperspheroidSurface').exclude()
            self.ompl_ns.class_('RNG').member_functions(
                'uniformProlateHyperspheroid').exclude()
        except declaration_not_found_t:
            pass

class ompl_morse_generator_t(code_generator_t):
    def __init__(self):
        replacement = default_replacement
        code_generator_t.__init__(self, 'morse', \
            ['bindings/util', 'bindings/base', 'bindings/geometric', 'bindings/control'], \
            replacement)
    def filter_declarations(self):
        stype = 'Morse'
        # create a python type for each of the corresponding state type
        state = self.ompl_ns.class_('ScopedState< ompl::base::%sStateSpace >' % stype)
        state.rename(stype+'State')
        state.operator('=', arg_types=['::ompl::base::State const &']).exclude()
        # add a constructor that allows a MorseState to be constructed from a State
        state.add_registration_code(
            'def(bp::init<ompl::base::ScopedState<ompl::base::StateSpace> const &>(( '
            'bp::arg("other") )))')
        # add a constructor that allows, e.g., a State to be constructed from a MorseState
        bstate = self.ompl_ns.class_('ScopedState< ompl::base::StateSpace >')
        bstate.add_registration_code(
            'def(bp::init<ompl::base::ScopedState<ompl::base::%sStateSpace> const &>(( '
            'bp::arg("other") )))' % stype)
        # add array access to double components of state
        self.add_array_access(state, 'double')


if __name__ == '__main__':
    setrecursionlimit(50000)
    if len(argv) == 1:
        print("Usage: generatebindings.py <modulename>")
    else:
        for module in argv[1:]:
            try:
                globals()['ompl_'+module+'_generator_t']()
            except KeyError:
                print("Error: can't generate code for module %s" % module)
