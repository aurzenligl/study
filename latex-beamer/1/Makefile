epss := $(patsubst %.uml,build/%.eps,$(wildcard *.uml))
odgs := $(patsubst %.odg,build/%.pdf,$(wildcard *.odg))

all: $(epss) $(odgs)
	pdflatex --output-directory build umls.tex

build/%.eps: %.uml
	@echo 'Building target: $< $@'
	plantuml -o build -teps $<

build/%.pdf: %.odg
	@echo 'Building target: $< $@'
	libreoffice --headless --convert-to --convert-to pdf --outdir build $<
