find_package(Boost COMPONENTS python)
# The python version needs to match the one used to build Boost.Python. 
# You can optionally specify the desired version like so:
# 	find_package(Python 2.6)
find_package(Python)
find_python_module(pyplusplus)
find_python_module(pygccxml)
find_package(GCCXML)

if(PYTHON_FOUND AND Boost_PYTHON_LIBRARY)
	set(PY_OMPL_COMPILE ON CACHE BOOL
		"Whether the OMPL Python modules should be built")
	set(OMPL_PYTHON_INSTALL_DIR "lib/python${PYTHON_VERSION}/site-packages")
endif()
if(PYTHON_FOUND AND Boost_PYTHON_LIBRARY AND PY_PYPLUSPLUS
	AND PY_PYGCCXML AND GCCXML AND NOT BUILD_PY_OMPL)
	set(PY_OMPL_GENERATE ON CACHE BOOL 
		"Whether the C++ code for the OMPL Python module should be generated")
endif()

set(OMPL_MODULES base control geometric util ${OMPL_APP_MODULE})
if(PYTHON_INCLUDE_DIRS)
  include_directories(${PYTHON_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/bindings" ".")
endif(PYTHON_INCLUDE_DIRS)
if(APPLE)
	# For some reason gccxml generates 32-bit binaries by default on OS X
	# (maybe because the kernel is still 32-bit?)
	set(PYOMPL_EXTRA_CFLAGS "-m64")
endif(APPLE)

if(PY_OMPL_GENERATE)	
	# need to run cmake to regenerate generate_bindings.py if 
	# generate_bindings.py is changed.
	configure_file("${CMAKE_CURRENT_SOURCE_DIR}/generate_bindings.py.in"
		"${CMAKE_CURRENT_SOURCE_DIR}/generate_bindings.py" @ONLY)

	# target for updating all-in-one header file for each module
	add_custom_target(generate_headers)
	# target for regenerating code for all python modules
	add_custom_target(update_bindings)

	foreach(module ${OMPL_MODULES})
		# create list of absolute paths to header files, which we
		# will add as a list of dependencies for the next target
		file(READ "headers_${module}.txt" headers_string)
		separate_arguments(rel_headers UNIX_COMMAND "${headers_string}")
		set(headers "")
		foreach(header ${rel_headers})
			if(EXISTS "${OMPL_INCLUDE_DIR}/ompl/${header}")
				list(APPEND headers "${OMPL_INCLUDE_DIR}/ompl/${header}")
			else()
				list(APPEND headers "${OMPL_APP_INCLUDE_DIR}/${header}")				
			endif()
		endforeach(header)
		# target for all-in-one header for module
		add_custom_target(${module}.h
			COMMAND ${CMAKE_COMMAND}
			-D module=${module} -D "OMPL_INCLUDE_DIR=${OMPL_INCLUDE_DIR}"
			-D "OMPL_APP_INCLUDE_DIR=${OMPL_APP_INCLUDE_DIR}"
			-P generate_header.cmake
			DEPENDS ${headers}
			WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
		add_dependencies(generate_headers ${module}.h)
		
		# target for regenerating code. Cmake is run so that the list of 
		# sources for the py_ompl_${module} target (see below) is updated.
		add_custom_target(update_${module}_bindings 
			COMMAND ${PYTHON_EXEC} 
			"${CMAKE_CURRENT_SOURCE_DIR}/generate_bindings.py" "${module}"
			COMMAND ${CMAKE_COMMAND} .
	 		WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
		add_dependencies(update_${module}_bindings ${module}.h)
		add_dependencies(update_bindings update_${module}_bindings)
	endforeach(module)
	add_dependencies(update_base_bindings update_util_bindings)
	add_dependencies(update_control_bindings update_base_bindings)
	add_dependencies(update_geometric_bindings update_base_bindings)
	if(OMPL_APP_MODULE)
		add_dependencies(update_${OMPL_APP_MODULE}_bindings update_geometric_bindings)
	endif()
	
endif(PY_OMPL_GENERATE)

if(PY_OMPL_COMPILE)
	# target for compiling python modules
	add_custom_target(py_ompl)
	
	foreach(module ${OMPL_MODULES})
		# target for each python module
		aux_source_directory("${CMAKE_CURRENT_SOURCE_DIR}/bindings/${module}" PY${module}BINDINGS)
		list(LENGTH PY${module}BINDINGS NUM_SOURCE_FILES)
		if(NUM_SOURCE_FILES GREATER 0)
			add_library(py_ompl_${module} MODULE ${PY${module}BINDINGS})
			target_link_libraries(py_ompl_${module} 
				ompl_static
				${OMPL_APP_ARCHIVE}
				${OMPL_APP_MODULE_LIBRARIES}
				${Boost_PYTHON_LIBRARY}
				${PYTHON_LIBRARIES})
			add_dependencies(py_ompl py_ompl_${module})
			get_target_property(PY${module}_NAME py_ompl_${module} LOCATION)
			add_custom_command(TARGET py_ompl_${module} POST_BUILD
		        COMMAND ${CMAKE_COMMAND} -E copy "${PY${module}_NAME}" 
		        "${CMAKE_CURRENT_SOURCE_DIR}/ompl/${module}/_${module}${CMAKE_SHARED_MODULE_SUFFIX}"
		        WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH})
    		include_directories("${CMAKE_CURRENT_SOURCE_DIR}/bindings/${module}")
			# python unit tests
			if(NOT module STREQUAL "${OMPL_APP_MODULE}")
				add_test(test_py_ompl_${module} "${PYTHON_EXEC}" 
					"${CMAKE_CURRENT_SOURCE_DIR}/tests/test_${module}.py")
			endif()
		else(NUM_SOURCE_FILES GREATER 0)
			if(PY_OMPL_GENERATE)
				message(STATUS "Code for module ${module} not found; type \"make update_bindings\"")
			else()
				message(STATUS "Code for module ${module} not found")
			endif()
		endif(NUM_SOURCE_FILES GREATER 0)
	endforeach(module)
	
	
	install(DIRECTORY demos/ DESTINATION ${OMPL_DEMO_INSTALL_DIR}/python
		PATTERN .svn EXCLUDE)
	install(DIRECTORY ompl DESTINATION ${OMPL_PYTHON_INSTALL_DIR}
		PATTERN .svn EXCLUDE PATTERN "*.pyc" EXCLUDE)
endif(PY_OMPL_COMPILE)
