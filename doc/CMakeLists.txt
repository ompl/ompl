option(OMPL_BUILD_DOC "Build OMPL documentation" ON)

if (OMPL_BUILD_DOC)

    find_package(Doxygen QUIET)

    if(DOXYGEN)
        if(UNIX)
            execute_process(COMMAND date "+%b %d, %Y" 
                OUTPUT_VARIABLE OMPL_RELEASE_DATE
                OUTPUT_STRIP_TRAILING_WHITESPACE)
        endif()
        configure_file("${CMAKE_CURRENT_SOURCE_DIR}/dox/download.md.in"
            "${CMAKE_CURRENT_SOURCE_DIR}/dox/download.md")
        configure_file("${CMAKE_CURRENT_SOURCE_DIR}/dox/mainpage.md.in"
            "${CMAKE_CURRENT_SOURCE_DIR}/dox/mainpage.md")

        # add "doc" target if Doxygen is detected
        add_custom_target(doc
            COMMAND ./mkdocs.sh
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

        # add "website" target
        add_custom_target(website
            COMMAND ./mkwebdocs.sh
            DEPENDS doc
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    endif(DOXYGEN)

endif(OMPL_BUILD_DOC)
