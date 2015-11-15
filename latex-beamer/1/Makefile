epss := $(patsubst %.uml,build/%.eps,$(wildcard *.uml))

all: $(epss)
	pdflatex --output-directory build umls.tex

build/%.eps: %.uml
	@echo 'Building target: $< $@'
	plantuml -o build -teps $<

