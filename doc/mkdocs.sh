#!/bin/sh
rm -rf html
doxygen Doxyfile
cp -r css fonts images js ieee-ram-2012-ompl.pdf ../install-ompl-ubuntu.sh html

cd html
for f in md_doc_markdown_*; do mv $f `echo $f | cut -c17-1000`; done
rm *8md_source.html

for f in *.html search/*.js; do
    sed 's/href="md_doc_markdown_/href="/g' "$f" > "$f.new"
    mv "$f.new" "$f"
done
