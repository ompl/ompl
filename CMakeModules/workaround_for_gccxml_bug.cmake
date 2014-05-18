# Function to work around a gccxml bug that results in a code that cannot be compiled
#
# For some boost::function's it generates code like this: boost::function<int()(int)>
# This script fixes that, albeit in a very fragile way. This could probably be improved
# by someone with better regexp writing skills.
file(GLOB fnames "${PATH}/*.pypp.*")
foreach(fname ${fnames})
    file(READ ${fname} _text)
    string(REPLACE " ()(" " (" _new_text "${_text}")
    file(WRITE "${fname}" "${_new_text}")
endforeach(fname)
