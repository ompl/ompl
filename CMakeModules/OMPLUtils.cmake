macro(add_ompl_test test_name)
  add_executable(${ARGV})
  target_link_libraries(${test_name}
    ompl::ompl
    Boost::program_options
    Boost::serialization
    Boost::filesystem
    Boost::system
    Boost::unit_test_framework)
  add_test(NAME ${test_name} COMMAND $<TARGET_FILE:${test_name}>)

  if (TARGET FLANN::flann)
    target_link_libraries(${test_name} FLANN::flann)
  endif()
endmacro(add_ompl_test)

macro(add_ompl_python_test test_file)
  get_filename_component(test_name "${test_file}" NAME)
  add_test(${test_name} "${PYTHON_EXEC}" "${CMAKE_CURRENT_SOURCE_DIR}/${test_file}" "-v")
endmacro(add_ompl_python_test)

option(OMPL_VERSIONED_INSTALL "Install header files in include/ompl-X.Y/ompl, where X and Y are the major and minor version numbers" ON)
add_feature_info(OMPL_VERSIONED_INSTALL "${OMPL_VERSIONED_INSTALL}" "Whether to install header files in\n   <prefix>/include/ompl-X.Y/ompl, where X and Y are the major and minor\n   version numbers")
if (OMPL_VERSIONED_INSTALL)
    set(CMAKE_INSTALL_INCLUDEDIR "include/ompl-${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
endif()

find_program(DOCKER docker NO_CMAKE_SYSTEM_PATH)
find_path(DOCKERFILE_PATH ompl.Dockerfile
    PATHS "${CMAKE_SOURCE_DIR}/scripts/docker" "${CMAKE_SOURCE_DIR}/ompl/scripts/docker"
    NO_DEFAULT_PATH)
if (DOCKER AND UNIX)
    add_custom_target(docker)
    macro(add_docker_target name)
        if(${ARGC} GREATER 1)
            get_filename_component(_path "${ARGV1}" ABSOLUTE)
        else()
            set(_path "${CMAKE_CURRENT_SOURCE_DIR}")
        endif()
        add_custom_target(docker-${name}
            COMMAND ${DOCKER} build -t "${name}:${PROJECT_VERSION}" -f ${DOCKERFILE_PATH}/${name}.Dockerfile "${_path}")
        add_dependencies(docker docker-${name})
    endmacro()
else()
    macro(add_docker_target name)
        # do nothing
    endmacro()
endif()
