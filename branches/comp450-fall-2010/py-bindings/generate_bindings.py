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
		std::string __printState(%s* manifold, ompl::base::State* state)
		{
			std::ostringstream s;
			manifold->printState(state, s);
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

		# hide optional argument of nextGoal, since we don't want to expose boost::posix_time
		replacement['nextGoal'] = ('def("nextGoal", &nextGoalWrapper, bp::return_value_policy< bp::reference_existing_object >())', """
		const ompl::base::State* nextGoalWrapper(%s* obj)
		{
			return obj->nextGoal();
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
		# don't export variables that need a wrapper
		self.ompl_ns.variables(lambda decl: decl.is_wrapper_needed()).exclude()	
		# force StateManifold::allocState to be exported.
		# (not sure why this is necessary)
		allocStateFn = self.ompl_ns.class_('StateManifold').member_function('allocState')
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
		bstate = self.ompl_ns.class_('ScopedState< ompl::base::StateManifold >')
		bstate.rename('State')
		bstate.operator('=', arg_types=['::ompl::base::State const &']).exclude()
		# loop over all predefined state manifolds
		for stype in ['Compound', 'RealVector', 'SO2', 'SO3', 'SE2', 'SE3']:
			# create a python type for each of their corresponding state types
			state = self.ompl_ns.class_('ScopedState< ompl::base::%sStateManifold >' % stype)
			state.rename(stype+'State')
			state.operator('=', arg_types=['::ompl::base::State const &']).exclude()
			# add a constructor that allows, e.g., an SE3State to be constructed from a State
			state.add_registration_code(
				'def(bp::init<ompl::base::ScopedState<ompl::base::StateManifold> const &>(( bp::arg("other") )))')
			# mark the manifold statetype as 'internal' to emphasize that it 
			# shouldn't typically be used by a regular python user
			self.ompl_ns.class_(stype + 'StateManifold').decls('StateType').rename(
				stype + 'StateInternal')
			# add a constructor that allows, e.g., an State to be constructed from a SE3State
			bstate.add_registration_code(
				'def(bp::init<ompl::base::ScopedState<ompl::base::%sStateManifold> const &>(( bp::arg("other") )))' % stype)
		# don't expose double*
		self.ompl_ns.class_('RealVectorStateManifold').class_(
			'StateType').variable('values').exclude()
		# add array indexing to the RealVectorState
		self.add_array_access(self.ompl_ns.class_('RealVectorStateManifold').class_('StateType'))
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
		# hide optional argument of nextGoal, since we don't want to expose boost::posix_time
		self.replace_member_function(self.ompl_ns.class_('PlannerInputStates').member_function('nextGoal'))
		# add wrapper code for setValidStateSamplerAllocator
		self.replace_member_functions(self.ompl_ns.namespace('base').class_(
				'SpaceInformation').member_functions('setValidStateSamplerAllocator'))
		
class ompl_control_generator_t(code_generator_t):
	def __init__(self):
		replacement = default_replacement
		# A C++ call like "foo.printControl(control, std::cout)" will be replaced with
		# something more pythonesque: "print foo.string(control)"
		replacement['printControl'] = ('def("string", &__printControl)', """
		std::string __printControl(%s* manifold, ompl::control::Control* control)
		{
			std::ostringstream s;
			manifold->printControl(control, s);
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

		    ompl::control::PropagationResult operator()(const ompl::base::State* start, const ompl::control::Control* control, const double duration, ompl::base::State* result)
		    {
				PyGILState_STATE gstate = PyGILState_Ensure();
				ompl::control::PropagationResult ret = bp::extract<ompl::control::PropagationResult>(callable_(bp::ptr(start), bp::ptr(control), duration, bp::ptr(result)));
				PyGILState_Release( gstate );
				return ret;
		    }

		    bp::object callable_;
		};

		void setPropagationFunctionWrapper(%s* obj, bp::object function)
		{
			obj->setPropagationFunction(ompl::control::StatePropagationFn(PropagatePyWrapper(function)));
		}
		""")
		code_generator_t.__init__(self, 'control', ['bindings/base'], replacement)
	
	def filter_declarations(self):
		code_generator_t.filter_declarations(self)
		# rename STL vectors of certain types
		self.std_ns.class_('vector< int >').rename('vectorInt')
		self.std_ns.class_('vector< double >').rename('vectorDouble')
		self.std_ns.class_('vector< ompl::control::Control* >').rename('vectorControlPtr')
		# don't export variables that need a wrapper
		self.ompl_ns.variables(lambda decl: decl.is_wrapper_needed()).exclude()	
		# force ControlManifold::allocState to be exported.
		# (not sure why this is necessary)
		allocControlFn = self.ompl_ns.class_('ControlManifold').member_function('allocControl')
		allocControlFn.include()
		allocControlFn.call_policies = \
			call_policies.return_value_policy(call_policies.reference_existing_object)
		# don't export components which is of type Control**
		self.ompl_ns.class_('CompoundControl').variable('components').exclude()
		# don't export some internal data structure
		self.ompl_ns.class_('OrderCellsByImportance').exclude()
		# add array indexing to the RealVectorState
		self.add_array_access(self.ompl_ns.class_('RealVectorControlManifold').class_('ControlType'))
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
			'ControlManifold').member_functions('setPropagationFunction'))
		
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
		# don't export variables that need a wrapper
		self.ompl_ns.variables(lambda decl: decl.is_wrapper_needed()).exclude()	
		# make objects printable that have a print function
		self.replace_member_functions(self.ompl_ns.member_functions('print'))
		# don't export some internal data structure
		self.ompl_ns.classes('OrderCellsByImportance').exclude()
		# add wrapper code for setStateValidityChecker
		self.replace_member_functions(self.ompl_ns.namespace('geometric').class_(
			'SimpleSetup').member_functions('setStateValidityChecker', 
			arg_types=['::ompl::base::StateValidityCheckerFn const &']))

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
