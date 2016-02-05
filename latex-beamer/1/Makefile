all: build/umls.pdf

epss := $(patsubst %.uml,build/%.eps,$(wildcard *.uml))
build/%.eps: %.uml
	@echo 'Building target: $< $@'
	plantuml -o build -teps $<

odgs := $(patsubst %.odg,build/%.pdf,$(wildcard *.odg))
build/%.pdf: %.odg
	@echo 'Building target: $< $@'
	libreoffice --headless --convert-to --convert-to pdf --outdir build $<

build/umls.pdf: umls.tex $(epss) $(odgs)
	pdflatex --output-directory build umls.tex
	pdflatex --output-directory build umls.tex
