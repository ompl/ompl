#############################################
# Construct configuration file              #
#############################################

file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" VERSION_DIR)
configure_file("${VERSION_DIR}/config.h.in" "${VERSION_DIR}/config.h")


#############################################
# recursively find all *.cpp files that     #
# correspond to the basic ompl library      #
#############################################
file(GLOB_RECURSE OMPL_SOURCE_CODE datastructures/*.cpp util/*.cpp base/*.cpp geometric/*.cpp control/*.cpp benchmark/*.cpp)



#############################################
# Add enabled extensions to the source code #
#############################################

## replace XXX by the extension
#if (OMPL_EXTENSION_XXX)
#    file(GLOB_RECURSE OMPL_XXX_EXTENSION_SOURCE_CODE extensions/XXX/*.cpp)
#    set(OMPL_SOURCE_CODE ${OMPL_SOURCE_CODE} ${OMPL_XXX_EXTENSION_SOURCE_CODE})
#
##    if additional libraries need to be linked, they are added to manifest.xml for ROS
##    or they are added to the list of OMPL_LINK_LIBRARIES
#
#endif()

if (OMPL_EXTENSION_ODE)
    file(GLOB_RECURSE OMPL_ODE_EXTENSION_SOURCE_CODE extensions/ode/*.cpp)
    set(OMPL_SOURCE_CODE ${OMPL_SOURCE_CODE} ${OMPL_ODE_EXTENSION_SOURCE_CODE})
    if (NOT OMPL_IN_ROS)
            list(APPEND OMPL_LINK_LIBRARIES ${ODE_LIBRARY})
    endif()
endif()

#############################################
# Add enabled contribs to the source code   #
#############################################

## replace XXX by the identifier of the contributor
#if (OMPL_CONTRIB_XXX)
#    file(GLOB_RECURSE OMPL_XXX_CONTRIB_SOURCE_CODE ${CMAKE_CURRENT_SOURCE_DIR}/../../contribs/XXX/src/*.cpp)
#    set(OMPL_SOURCE_CODE ${OMPL_SOURCE_CODE} ${OMPL_XXX_CONTRIB_SOURCE_CODE})
#
##    if additional libraries need to be linked, they are added to manifest.xml for ROS
##    or they are added to the list of OMPL_LINK_LIBRARIES
#
#endif()


#############################################
# Build and install the library             #
#############################################
if(OMPL_IN_ROS)
    rosbuild_check_for_sse()
    rosbuild_add_library(ompl ${OMPL_SOURCE_CODE})
    rosbuild_add_compile_flags(ompl -Wextra)
    rosbuild_add_compile_flags(ompl ${SSE_FLAGS})
    rosbuild_link_boost(ompl thread date_time)
else(OMPL_IN_ROS)
    if(MSVC)
        add_library(ompl STATIC ${OMPL_SOURCE_CODE})
    else(MSVC)
        add_library(ompl SHARED ${OMPL_SOURCE_CODE})
    endif(MSVC)
    target_link_libraries(ompl
        ${Boost_THREAD_LIBRARY}
        ${Boost_DATE_TIME_LIBRARY}
        ${OMPL_LINK_LIBRARIES}
    )
endif(OMPL_IN_ROS)

# install the library
install(TARGETS ompl DESTINATION lib/)
get_target_property(OMPL_LIBRARY_LOC ompl LOCATION)
add_custom_command(TARGET ompl POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${OMPL_LIBRARY_LOC}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../py-bindings/ompl/util/libompl${CMAKE_SHARED_LIBRARY_SUFFIX}"
    WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH})
