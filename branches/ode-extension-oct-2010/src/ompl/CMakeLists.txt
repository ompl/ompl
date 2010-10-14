# recursively find all *.cpp files that correspond to the ompl library
file(GLOB_RECURSE OMPL_SOURCE_CODE datastructures/*.cpp util/*.cpp base/*.cpp geometric/*.cpp control/*.cpp)

if (NOT ODE_FOUND AND ROS)
  include(CheckIncludeFileCXX)
  # add ODE extension if ODE found
  CHECK_INCLUDE_FILE_CXX(ode/ode.h ODE_FOUND)
endif()

if (ODE_FOUND)
  message(STATUS "Building ODE extension")
  file(GLOB_RECURSE OMPL_ODE_EXTENSION_SOURCE_CODE extensions/ode/*.cpp)
  set(OMPL_SOURCE_CODE ${OMPL_SOURCE_CODE} ${OMPL_ODE_EXTENSION_SOURCE_CODE})
  if (NOT ROS)
    add_definitions(${ODE_DEFINITIONS})
    include_directories(${ODE_INCLUDE})
    list(APPEND OMPL_LINK_LIBRARIES ${ODE_LIBRARY})
  endif()
endif()

file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" VERSION_DIR)
configure_file("${VERSION_DIR}/version.h.in" "${VERSION_DIR}/version.h")

if(ROS)  
	rosbuild_add_library(ompl ${OMPL_SOURCE_CODE})
	rosbuild_add_compile_flags(ompl -Wextra)
	rosbuild_link_boost(ompl thread date_time)
else(ROS)
	add_definitions(-fPIC)
	add_library(ompl SHARED ${OMPL_SOURCE_CODE})
	target_link_libraries(ompl
		${Boost_THREAD_LIBRARY}
		${Boost_DATE_TIME_LIBRARY}
		${OMPL_LINK_LIBRARIES}
	)
endif(ROS)

install(TARGETS ompl DESTINATION lib/)
get_target_property(OMPL_LIBRARY_LOC ompl LOCATION)
add_custom_command(TARGET ompl POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${OMPL_LIBRARY_LOC}" 
    "${CMAKE_CURRENT_SOURCE_DIR}/../../py-bindings/ompl/util/libompl${CMAKE_SHARED_LIBRARY_SUFFIX}"
    WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH})
