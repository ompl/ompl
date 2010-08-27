file(GLOB_RECURSE OMPL_SOURCE_CODE *.cpp)

file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" VERSION_DIR)
configure_file("${VERSION_DIR}/version.h.in" "${VERSION_DIR}/version.h")

if(ROS)  
	rosbuild_add_library(ompl ${OMPL_SOURCE_CODE})
	rosbuild_add_compile_flags(ompl -Wextra)
	rosbuild_link_boost(ompl thread date_time)
	install(TARGETS ompl DESTINATION lib/)
else(ROS)
	add_definitions(-fPIC)
	add_library(ompl SHARED ${OMPL_SOURCE_CODE})
	target_link_libraries(ompl
		${Boost_THREAD_LIBRARY}
		${Boost_DATE_TIME_LIBRARY}
	)
endif(ROS)

get_target_property(OMPL_LIBRARY_LOC ompl LOCATION)
add_custom_command(TARGET ompl POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${OMPL_LIBRARY_LOC}" 
    "${CMAKE_CURRENT_SOURCE_DIR}/../../py-bindings/ompl/libompl${CMAKE_SHARED_LIBRARY_SUFFIX}"
    WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH})
