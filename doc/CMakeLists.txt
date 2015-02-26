find_package(Doxygen QUIET)

if(DOXYGEN)
    if(UNIX)
        execute_process(COMMAND date "+%b %d, %Y"
            OUTPUT_VARIABLE OMPL_RELEASE_DATE
            OUTPUT_STRIP_TRAILING_WHITESPACE)
        set(OMPL_RELEASE_DATE "${OMPL_RELEASE_DATE}" PARENT_SCOPE)
    endif()
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/header.html.in"
        "${CMAKE_CURRENT_SOURCE_DIR}/header.html")
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/markdown/download.md.in"
        "${CMAKE_CURRENT_SOURCE_DIR}/markdown/download.md")
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/markdown/mainpage.md.in"
        "${CMAKE_CURRENT_SOURCE_DIR}/markdown/mainpage.md")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/images/ompl.svg" OMPLSVG)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/images/cforest.svg" CFORESTSVG)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/images/cforest_sampler.svg" CFOREST_SAMPLERSVG)
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/markdown/api_overview.md.in"
        "${CMAKE_CURRENT_SOURCE_DIR}/markdown/api_overview.md")
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/markdown/CForest.md.in"
        "${CMAKE_CURRENT_SOURCE_DIR}/markdown/CForest.md")

    # add "doc" target if Doxygen is detected
    add_custom_target(doc
        COMMAND ./mkdocs.sh
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/doc")

    # add "website" target
    add_custom_target(website
        COMMAND ./mkwebdocs.sh
        DEPENDS doc
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/doc")
endif(DOXYGEN)

install(FILES man/ompl_benchmark_statistics.1
    DESTINATION ${CMAKE_INSTALL_MANDIR}/man1 COMPONENT ompl)
