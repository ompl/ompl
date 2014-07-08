#!/bin/sh
rm -rf html
doxygen Doxyfile
cd html
for f in md_doc_markdown_*; do mv $f `echo $f | cut -c17-1000`; done
rm *8md_source.html
