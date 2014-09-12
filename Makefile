paper:
	pdflatex paper.tex
	bibtex paper
	pdflatex paper.tex
	pdflatex paper.tex

clean:	
	rm paper.log paper.blg paper.aux paper.bbl paper.pdf
