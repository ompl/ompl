option(OMPL_BUILD_DOC "Build OMPL documentation" ON)

if (OMPL_BUILD_DOC)

    find_package(Doxygen QUIET)

    if(DOXYGEN)
        if(UNIX)
            execute_process(COMMAND date "+%b %d, %Y" 
                OUTPUT_VARIABLE OMPL_RELEASE_DATE
                OUTPUT_STRIP_TRAILING_WHITESPACE)
        endif()
        configure_file("${CMAKE_CURRENT_SOURCE_DIR}/dox/download.dox.in"
            "${CMAKE_CURRENT_SOURCE_DIR}/dox/download.dox")
        configure_file("${CMAKE_CURRENT_SOURCE_DIR}/dox/mainpage.dox.in"
            "${CMAKE_CURRENT_SOURCE_DIR}/dox/mainpage.dox")

        # add "doc" target if Doxygen is detected
        add_custom_target(doc
            COMMAND ${CMAKE_COMMAND} -E remove_directory html
            COMMAND ${DOXYGEN} "Doxyfile"
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
        add_custom_command(TARGET doc POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy js/search.js html/search
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

        # add "website" target
        add_custom_target(website
            COMMAND ./mkwebdocs.sh
            DEPENDS doc
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    endif(DOXYGEN)

endif(OMPL_BUILD_DOC)
