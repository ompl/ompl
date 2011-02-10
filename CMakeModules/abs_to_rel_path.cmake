# function to change absolute to relative paths, which is handy if we want to
# redistribute the generated python binding code

file(GLOB fnames "${PATH}/*.cpp")
foreach(fname ${fnames})
    file(READ ${fname} _text)
    string(REPLACE "#include \"${CMAKE_CURRENT_SOURCE_DIR}/bindings/"
        "#include \"../" _new_text "${_text}")
    file(WRITE ${fname} "${_new_text}")
endforeach(fname)
