
.ONESHELL:

SRCDIR=$(CURDIR)/src

all:

mach1:
	mkdir -p build/$@
	cd build/$@
	gcc -E -nostdinc -x assembler-with-cpp -o linux.dts $(SRCDIR)/mach1.dts
	dtc -O dtb -o linux.dtb linux.dts
