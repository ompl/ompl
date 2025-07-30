if(DOXYGEN_FOUND AND NOT CMAKE_VERSION VERSION_LESS 3.9)
    if(UNIX)
        execute_process(COMMAND date "+%b %d, %Y"
            OUTPUT_VARIABLE OMPL_RELEASE_DATE
            OUTPUT_STRIP_TRAILING_WHITESPACE)
        set(OMPL_RELEASE_DATE "${OMPL_RELEASE_DATE}" PARENT_SCOPE)
    endif()
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/markdown/download.md.in"
        "${CMAKE_BINARY_DIR}/doc/ompl_download.md")
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/markdown/mainpage.md.in"
        "${CMAKE_BINARY_DIR}/doc/mainpage.md")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/images/ompl.svg" OMPLSVG)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/images/cforest.svg" CFORESTSVG)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/images/cforest_sampler.svg" CFOREST_SAMPLERSVG)
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/markdown/api_overview.md.in"
        "${CMAKE_BINARY_DIR}/doc/api_overview.md")
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/markdown/CForest.md.in"
        "${CMAKE_BINARY_DIR}/doc/CForest.md")

    include(DoxygenSettings)
    set(DOXYGEN_PROJECT_NAME "ompl")
    set(DOXYGEN_EXAMPLE_PATH
        "${CMAKE_CURRENT_SOURCE_DIR}/../demos"
        "${CMAKE_CURRENT_SOURCE_DIR}/markdown/code")
    set(DOXYGEN_HTML_COLORSTYLE "light")
    set(DOXYGEN_HTML_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/header.html")
    set(DOXYGEN_HTML_FOOTER "${CMAKE_CURRENT_SOURCE_DIR}/footer.html")
    set(DOXYGEN_HTML_EXTRA_STYLESHEET "{CMAKE_CURRENT_SOURCE_DIR}/ompl.css")
    set(DOXYGEN_HTML_OUTPUT "${CMAKE_BINARY_DIR}/ompl_doc")
    set(DOXYGEN_GENERATE_TREEVIEW NO)
    set(DOXYGEN_GENERATE_TAGFILE "${DOXYGEN_HTML_OUTPUT}/ompl.tag")
    doxygen_add_docs(ompl_doc
        "${CMAKE_CURRENT_SOURCE_DIR}/markdown"
        "${CMAKE_CURRENT_SOURCE_DIR}/../demos"
        "${CMAKE_CURRENT_SOURCE_DIR}/../src/ompl"
        "${CMAKE_BINARY_DIR}/doc"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}")
    add_custom_command(TARGET ompl_doc POST_BUILD
        COMMAND cp -r images ieee-ram-2012-ompl.pdf "${CMAKE_BINARY_DIR}/install-ompl-ubuntu.sh" "${DOXYGEN_HTML_OUTPUT}"
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

    # use latexmk to create the OMPL primer PDF
    find_program(LATEXMK latexmk)
    if (LATEXMK)
        add_custom_command(TARGET ompl_doc POST_BUILD
            COMMAND "BIBINPUTS=${CMAKE_CURRENT_SOURCE_DIR}/latex/bib" openout_any=a latexmk -gg -pdf "-outdir=${DOXYGEN_HTML_OUTPUT}" OMPL_Primer.tex
            COMMAND latexmk -c -pdf "-outdir=${DOXYGEN_HTML_OUTPUT}" OMPL_Primer.tex
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/latex")
    endif()

    add_custom_target(website
        COMMAND rsync -a ompl_doc/ "$ENV{HOME}/src/ompl.github.io/"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}")
    add_dependencies(website ompl_doc)
endif()

install(FILES man/ompl_benchmark_statistics.1 man/plannerarena.1
    DESTINATION ${CMAKE_INSTALL_MANDIR}/man1 COMPONENT ompl)
