epss := $(patsubst %.uml,build/%.eps,$(wildcard *.uml))
odgs := $(patsubst %.odg,build/%.pdf,$(wildcard *.odg))

all: build/umls.pdf

build/umls.pdf: umls.tex $(epss) $(odgs)
	pdflatex --output-directory build umls.tex
	pdflatex --output-directory build umls.tex

build/%.eps: %.uml
	@echo 'Building target: $< $@'
	plantuml -o build -teps $<

build/%.pdf: %.odg
	@echo 'Building target: $< $@'
	libreoffice --headless --convert-to --convert-to pdf --outdir build $<
