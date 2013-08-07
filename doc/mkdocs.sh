#!/bin/sh
rm -rf html
doxygen Doxyfile
cd html
# for doxygen >= 1.8.4
for f in md_doc_dox_*; do mv $f `echo $f | cut -c12-1000`; done
# for doxygen >= 1.8.0 and < 1.8.4
# for f in md_*; do mv $f `echo $f | cut -c4-1000`; done
# mv mainpage.html index.html
rm *8md_source.html
