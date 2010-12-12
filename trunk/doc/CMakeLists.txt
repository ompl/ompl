option(OMPL_BUILD_DOC "Build OMPL documentation" ON)

if (OMPL_BUILD_DOC)

    find_package(Doxygen QUIET)

    if(DOXYGEN)
        # add "doc" target if Doxygen is detected
        add_custom_target(doc
            COMMAND ${CMAKE_COMMAND} -E remove_directory html
            COMMAND ${DOXYGEN} "Doxyfile"
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
        add_custom_command(TARGET doc POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy js/search.js html/search
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

        # Ruby is only needed if you want the regenerate the CSS files for 
        # the documentation
        find_package(Ruby QUIET)

        # if ruby is installed, we can also regenerate the CSS file from 
        # individual CSS files using the Blueprint framework
        if(RUBY_EXECUTABLE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/blueprint/lib/compress.rb")
            add_custom_target(update_css
                COMMAND ${RUBY_EXECUTABLE} ./blueprint/lib/compress.rb -p ompl
                WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
            # automatically update CSS when regenerating HTML
            add_dependencies(doc update_css)
        endif()

        # add "website" target
        add_custom_target(website
            COMMAND ./mkwebdocs.sh
            DEPENDS doc
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

        # install target
        install(DIRECTORY css html images js DESTINATION "${OMPL_DOC_INSTALL_DIR}"
            PATTERN ".svn" EXCLUDE)
        install(FILES index.html DESTINATION "${OMPL_DOC_INSTALL_DIR}")
    endif(DOXYGEN)

endif(OMPL_BUILD_DOC)
