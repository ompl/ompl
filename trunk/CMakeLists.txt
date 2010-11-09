option(OMPL_IN_ROS "Build OMPL against ROS" OFF)

set(OMPL_MAJOR_VERSION 0)
set(OMPL_MINOR_VERSION 3)
set(OMPL_PATCH_VERSION 0)
set(OMPL_VERSION "${OMPL_MAJOR_VERSION}.${OMPL_MINOR_VERSION}.${OMPL_PATCH_VERSION}")

if(OMPL_IN_ROS)
	rosbuild_add_boost_directories()
	set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
else(OMPL_IN_ROS)
	if(NOT OMPLAPP_VERSION)
		cmake_minimum_required(VERSION 2.8)
		project(ompl CXX)
		set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")
		include(CompilerSettings)
		find_package(Boost COMPONENTS date_time thread filesystem system REQUIRED)
		include_directories(${Boost_INCLUDE_DIR})
		
		# ODE is only needed for ODE bindings
		find_package(ODE QUIET)
		set(OMPL_EXTENSION_ODE ${ODE_FOUND})
		if (ODE_FOUND)
 		        add_definitions(${ODE_DEFINITIONS})
 			include_directories(${ODE_INCLUDE})
 		endif()
	
		enable_testing()
	endif(NOT OMPLAPP_VERSION)
endif(OMPL_IN_ROS)

set(OMPL_CMAKE_UTIL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules"
	CACHE FILEPATH "Path to directory with auxiliary CMake scripts for OMPL")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

set(OMPL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(OMPL_DEMO_INSTALL_DIR share/ompl/demos)
set(OMPL_DOC_INSTALL_DIR share/ompl/doc)

include_directories(${OMPL_INCLUDE_DIR})
add_subdirectory(src)
add_subdirectory(py-bindings)
add_subdirectory(tests)
add_subdirectory(demos)
add_subdirectory(doc)

install(DIRECTORY demos/ DESTINATION ${OMPL_DEMO_INSTALL_DIR}
        PATTERN CMakeLists.txt EXCLUDE
	PATTERN .svn EXCLUDE
	PATTERN .DS_Store EXCLUDE)
install(DIRECTORY doc/ DESTINATION ${OMPL_DOC_INSTALL_DIR}
        PATTERN dox EXCLUDE 
	PATTERN header.html EXCLUDE 
	PATTERN footer.html EXCLUDE
	PATTERN mkwebdocs.sh EXCLUDE
	PATTERN CMakeLists.txt EXCLUDE
	PATTERN blueprint EXCLUDE
	PATTERN .svn EXCLUDE
	PATTERN .md5 EXCLUDE
	PATTERN .DS_Store EXCLUDE
	PATTERN Doxyfile EXCLUDE)

if(NOT OMPL_IN_ROS AND NOT OMPLAPP_VERSION)
	include(CPackSettings)
endif()
