#!/bin/sh
rm -rf html
doxygen Doxyfile
cp -r css fonts images js php ieee-ram-2012-ompl.pdf html

cd html
for f in md_doc_markdown_*; do mv $f `echo $f | cut -c17-1000`; done
rm *8md_source.html

for f in *.html; do
    sed -i "" 's/href="md_ompl_doc_markdown_/href="/g;s/href="md_doc_markdown_/href="/g' $f
done

