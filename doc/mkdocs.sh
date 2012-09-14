#!/bin/sh
rm -rf html
doxygen Doxyfile
cd html
for f in md_*; do mv $f `echo $f | cut -c4-1000`; done
mv mainpage.html index.html
rm *8md_source.html
