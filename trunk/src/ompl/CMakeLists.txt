file(GLOB_RECURSE OMPL_SOURCE_CODE *.cpp)

file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" VERSION_DIR)
configure_file("${VERSION_DIR}/version.h.in" "${VERSION_DIR}/version.h")

if(ROS)  
	rosbuild_add_library(ompl ${OMPL_SOURCE_CODE})
	rosbuild_add_compile_flags(ompl -Wextra)
	rosbuild_link_boost(ompl thread date_time)
else(ROS)
	add_definitions(-fPIC)
	add_library(ompl SHARED ${OMPL_SOURCE_CODE})
	add_library(ompl_static STATIC ${OMPL_SOURCE_CODE})
	set_target_properties(ompl_static PROPERTIES OUTPUT_NAME "ompl")
	# needed for MS Windows, see:
	# http://www.cmake.org/Wiki/CMake_FAQ#How_do_I_make_my_shared_and_static_libraries_have_the_same_root_name.2C_but_different_suffixes.3F
	set_target_properties(ompl_static PROPERTIES PREFIX "lib")
	target_link_libraries(ompl
		${Boost_THREAD_LIBRARY}
		${Boost_DATE_TIME_LIBRARY}
		${PROF_LIBRARY}
	)
	target_link_libraries(ompl_static
		${Boost_THREAD_LIBRARY}
		${Boost_DATE_TIME_LIBRARY}
		${PROF_LIBRARY}
	)
endif(ROS)

install(TARGETS ompl ompl_static LIBRARY DESTINATION lib/ ARCHIVE DESTINATION lib/)
