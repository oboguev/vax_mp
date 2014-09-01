#
# makefile for VAX MP for Linux, OS X and Unix
#
# To build VAX MP invoke as:
#
#    make CONFIG=... USE_NETWORK={1|0}
#
# where possible values for CONFIG are:
#
#    x86-dbg   for 32-bit i86 debug build
#    x86-rel       32-bit i86 release build
#    x64-dbg       64-bit x86 debug build
#    x64-rel       64-bit x86 release build
#
# To clean the target, invoke
#
#    make CONFIG=... clean
#
# To rebuild dependency list before building, invoke as
#
#    make CONFIG=... depend
#
# Suggested sequence for clean rebuild is:
#
#    make CONFIG=... clean
#    make CONFIG=... depend
#    make CONFIG=... USE_NETWORK={1|0}
#
# or
#
#    make CONFIG=... rebuild
#

SHELL = /bin/sh
MK2 = $(MAKE) $(MAKEFLAGS) -f makefile2 --no-print-directory
VAX = vax_mp

.PHONY : all clean rebuild depend $(VAX) check-config ;

all: $(VAX) ;

check-config:
	@chmod a+x ./check-config.sh
	@./check-config.sh CONFIG=$(CONFIG) USE_NETWORK=$(USE_NETWORK)

$(VAX): check-config
	@$(MK2) $(VAX)

depend: check-config
	@$(MK2) depend

rebuild: clean
	@$(MK2) depend
	@$(MK2) $(VAX)

clean: check-config
	-rm -rf $(CONFIG) $(CONFIG)-depend.mk
