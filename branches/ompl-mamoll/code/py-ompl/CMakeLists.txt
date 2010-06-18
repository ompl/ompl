find_package(Boost COMPONENTS python)
# The python version needs to match the one used to build Boost.Python
find_package(Python 2.6)
find_python_module(pyplusplus)
find_python_module(pygccxml)
find_package(GCCXML)

if(PYTHON_FOUND AND Boost_PYTHON_LIBRARY AND PY_PYPLUSPLUS
	AND PY_PYGCCXML AND GCCXML AND NOT BUILD_PY_OMPL)
	message(STATUS "Building OMPL Python module")
	set(BUILD_PY_OMPL ON CACHE BOOL 
		"Whether the OMPL Python module should be built")
endif()

if(BUILD_PY_OMPL)
	macro(glob_headers _var _dir)
		file(GLOB_RECURSE ${_var} "${CMAKE_SOURCE_DIR}/code/ompl/${_dir}/*.h")
	endmacro(glob_headers)

	glob_headers(OMPL_BASE_HEADERS base)
	glob_headers(OMPL_DYNAMIC_HEADERS dynamic)
	glob_headers(OMPL_KINEMATIC_HEADERS kinematic)
	glob_headers(OMPL_UTIL_HEADERS util)

	#glob_headers(OMPL_DATASTRUCTURES_HEADERS datastructures)
	# need to remove this header for Py++
	#list(REMOVE_ITEM OMPL_DATASTRUCTURES_HEADERS
	#	"${CMAKE_CURRENT_SOURCE_DIR}/datastructures/GridAux.h")

	# \todo hack alert: need to figure why base/planner.h is needed for 
	# dynamic and kinematic python modules
	file(GLOB OMPL_BASE_PLANNER_H "${CMAKE_SOURCE_DIR}/code/ompl/base/Planner.h")

	set(OMPL_MODULES base dynamic kinematic util)
	include_directories(${PYTHON_INCLUDE_DIRS} .)
	
	if(APPLE)
		# For some reason gccxml generates 32-bit binaries by default on OS X
		# (maybe because the kernel is still 32-bit?)
		set(PYOMPL_EXTRA_CFLAGS "-m64")
	endif(APPLE)
	
	# need to run cmake to regenerate generate_bindings.py if 
	# generate_bindings.py is changed.
	foreach(module ${OMPL_MODULES})
		string(TOUPPER ${module} module_upper)
		foreach(header ${OMPL_${module_upper}_HEADERS})
			if(OMPL_${module_upper}_LIST)
				set(OMPL_${module_upper}_LIST
					"${OMPL_${module_upper}_LIST}, \"${header}\"")
			else(OMPL_${module_upper}_LIST)
				set(OMPL_${module_upper}_LIST "\"${header}\"")
			endif(OMPL_${module_upper}_LIST)
		endforeach(header)
	endforeach(module)
	
	configure_file("${CMAKE_CURRENT_SOURCE_DIR}/generate_bindings.py.in"
		"${CMAKE_CURRENT_SOURCE_DIR}/generate_bindings.py" @ONLY)
	
	# bootstrap bindings generation
	find_file(BINDINGS_MAIN "_base.main.cpp"
		PATHS "${CMAKE_CURRENT_SOURCE_DIR}/bindings/_base" 
		NO_DEFAULT_PATH)
	if(NOT BINDINGS_MAIN)
		execute_process(COMMAND ${PYTHON_EXEC} "./generate_bindings.py"
			WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
	endif(NOT BINDINGS_MAIN)
	
	# re-generate bindings
	add_custom_target(update_bindings 
		COMMAND ${PYTHON_EXEC} "./generate_bindings.py"
		WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
	
	# targets for python modules
	foreach(module ${OMPL_MODULES})
		string(TOUPPER ${module} module_upper)
		aux_source_directory("bindings/_${module}" PY${module}BINDINGS)
		add_library(py_ompl_${module} MODULE ${PY${module}BINDINGS})
		target_link_libraries(py_ompl_${module} 
			ompl ${Boost_PYTHON_LIBRARY} ${PYTHON_LIBRARIES})
		add_dependencies(update_bindings ${OMPL_${module_upper}_HEADERS})
		get_target_property(PY${module}_NAME py_ompl_${module} LOCATION)
		add_custom_command(TARGET py_ompl_${module} POST_BUILD
		        COMMAND cp "${PY${module}_NAME}" 
		        "${CMAKE_CURRENT_SOURCE_DIR}/ompl/${module}/_${module}${CMAKE_SHARED_MODULE_SUFFIX}"
		        WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH})
	endforeach(module)

	# python unit tests
	add_test(test_py_ompl "${PYTHON_EXEC}" "${CMAKE_CURRENT_SOURCE_DIR}/test.py")
endif(BUILD_PY_OMPL)

