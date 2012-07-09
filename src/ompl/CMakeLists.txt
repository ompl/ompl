#############################################
# Construct configuration file              #
#############################################

file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" VERSION_DIR)
configure_file("${VERSION_DIR}/config.h.in" "${VERSION_DIR}/config.h")


#############################################
# recursively find all *.cpp files that     #
# correspond to the basic ompl library      #
#############################################
file(GLOB_RECURSE OMPL_SOURCE_CODE datastructures/*.cpp util/*.cpp base/*.cpp geometric/*.cpp control/*.cpp tools/*.cpp)



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

if (OMPL_EXTENSION_OPENDE)
    file(GLOB_RECURSE OMPL_OPENDE_EXTENSION_SOURCE_CODE extensions/opende/*.cpp)
    set(OMPL_SOURCE_CODE ${OMPL_SOURCE_CODE} ${OMPL_OPENDE_EXTENSION_SOURCE_CODE})
    list(APPEND OMPL_LINK_LIBRARIES ${OPENDE_LIBRARY})
endif()

#############################################
# Add enabled contributed source code       #
#############################################
add_subdirectory(contrib/sample_contrib)
add_subdirectory(contrib/rrt_star)

#############################################
# Build and install the library             #
#############################################
if(MSVC OR IS_ICPC)
  add_library(ompl STATIC ${OMPL_SOURCE_CODE})
else(MSVC OR IS_ICPC)
  add_library(ompl SHARED ${OMPL_SOURCE_CODE})
endif(MSVC OR IS_ICPC)
target_link_libraries(ompl
  ${Boost_THREAD_LIBRARY}
  ${Boost_DATE_TIME_LIBRARY}
  ${Boost_SERIALIZATION_LIBRARY}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${OMPL_LINK_LIBRARIES}
  )

if(MSVC)
  set_target_properties(ompl PROPERTIES VERSION "${OMPL_VERSION}" STATIC_LIBRARY_FLAGS "psapi.lib ws2_32.lib")
else(MSVC)
  if (MINGW)
    target_link_libraries(ompl psapi ws2_32)
    set_target_properties(ompl PROPERTIES LINK_FLAGS "-Wl,--export-all-symbols")
  endif(MINGW)
  set_target_properties(ompl PROPERTIES VERSION "${OMPL_VERSION}" SOVERSION "${OMPL_ABI_VERSION}")
endif(MSVC)

# install the library
install(TARGETS ompl DESTINATION lib/)
if(NOT MSVC)
    get_target_property(OMPL_LIBRARY_LOC ompl LOCATION)
    add_custom_command(TARGET ompl POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${OMPL_LIBRARY_LOC}"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../py-bindings/ompl/util/libompl${CMAKE_SHARED_LIBRARY_SUFFIX}"
        WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH})
endif(NOT MSVC)
