BLDDIR=$(CURDIR)/build
SRCDIR=$(CURDIR)/src
TARGETS=$(shell grep -Po "^[^\#[:space:]].*(?=:)" Makefile | grep -v "\(\.\|all\|list\)")

define build_dtb
	gcc $2 -E -nostdinc -x assembler-with-cpp -o $(BLDDIR)/$@/linux$1.dts $(SRCDIR)/$@.dts
	dtc -O dtb -o $(BLDDIR)/$@/linux$1.dtb $(BLDDIR)/$@/linux$1.dts
endef

define build_board
	mkdir -p $(BLDDIR)/$@
	$(call build_dtb,,-DMAIN)
	$(call build_dtb,_addon1)
	$(call build_dtb,_addon2)
	$(call build_dtb,_addon3)
endef

all: $(TARGETS)

list:
	@echo $(TARGETS)

mach1:
	$(build_board)

mach2:
	$(build_board)
