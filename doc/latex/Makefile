export BIBINPUTS=.:./bib:$BIBINPUTS::

all:    primer

primer:
	pdflatex OMPL_Primer.tex
	bibtex OMPL_Primer.aux
	pdflatex OMPL_Primer.tex
	pdflatex OMPL_Primer.tex

clean:
	rm -f OMPL_Primer.{aux,bbl,blg,log,toc}

