###############################################################################
# Makefile script for CRC Test Application
#
# @par
# Copyright (c) 2009-2017, Intel Corporation
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Intel Corporation nor the names of its contributors
#       may be used to endorse or promote products derived from this software
#       without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
###############################################################################

APPNAME ?= crctest
LIBNAME ?= libcrc.a
DEPFILE ?= depend

CFLAGS = -Wall -Winline
CFLAGS_GCC = -msse4.2 -mpclmul -falign-loops=32
CFLAGS_ICC64 = -axSSE4.2 -xHost
CFLAGS_ICC32 = -axSSE4.2
LDFLAGS = -L. -lcrc

ifeq ($(CC),icc)
#
# ICC Intel C Compiler
#
ifeq ($(BUILD32),y)
CFLAGS += $(CFLAGS_ICC32)
else
CFLAGS += $(CFLAGS_ICC64)
endif
else
#
# GCC GNU C Compiler
#
CFLAGS += $(CFLAGS_GCC)
endif

ifeq ($(DEBUG),y)
CFLAGS += -g -O0 -DDEBUG
else
# Same as -O3 but no -ftree-vectorize to avoid vectorization of standard checksum code
# When -O3 was used then standard TCP/IP checksum code got vectorized and
# it was not fair comparison between standard IA code and SSE enabled one.
# SSE enabled code was still better than vectorized by compiler. Change to -O3 and check :)
# It doesnot affect CRC code which doesnot get vectorized anyway.
CFLAGS += -O2 -finline-functions -funswitch-loops -fpredictive-commoning -fgcse-after-reload -fipa-cp-clone
endif

ifeq ($(VERBOSE),y)
CFLAGS += -ftree-vectorizer-verbose=2
endif

ifeq ($(CROSSBUILD32),y)
CFLAGS+=-m32
endif

#
# Build rules
#


AFILES = crc.o
AFILES += crcr.o
AFILES += crc_rnc.o
AFILES += crc_wimax.o
AFILES += crc_sctp.o
AFILES += crc_tcpip.o
AFILES += crc_ether.o
AFILES += crc_cable.o

all: $(APPNAME)

$(APPNAME): $(DEPFILE) main.o $(LIBNAME)
	$(CC) $(CFLAGS) main.o $(LDFLAGS) -o $@

$(DEPFILE): $(subst .o,.c,main.o $(AFILES))
	$(CC) -MM $(CFLAGS) $^ > $@

$(LIBNAME): $(AFILES) crc32_refl_by8.o
	$(AR) crvs $@ $^

crc32_refl_by8.o:
	yasm -f x64 -f elf64 -D LINUX -X gnu -g dwarf2 crc32_refl_by8.asm

.PHONY: help
help:
	@echo " "
	@echo "Compilation options:"
	@echo "    make all      - compile crctest application"
	@echo "    make clean    - clean folder from compilation object files"
	@echo "    make style    - run checkpatch.pl on files"
	@echo "    make cppcheck - run cppcheck on files"
	@echo " "

# Include dependecies file
-include $(DEPFILE)

.PHONY: TAGS
TAGS:
	etags *.[ch]

.PHONY: clean
clean:
	-rm -f $(APPNAME) $(DEPFILE) *.o *.a

CHECKPATCH?=checkpatch.pl
.PHONY: style
style:
	$(CHECKPATCH) --no-tree --no-signoff --emacs \
	--ignore CODE_INDENT,INITIALISED_STATIC,LEADING_SPACE,SPLIT_STRING,\
	NEW_TYPEDEFS,UNSPECIFIED_INT,ARRAY_SIZE,BLOCK_COMMENT_STYLE,\
	CONSTANT_COMPARISON,PREFER_PACKED,GLOBAL_INITIALISERS \
	-f crc.c -f crc.h -f crcr.c -f crcr.h -f crc_rnc.c -f crc_rnc.h \
	-f crc_sctp.c -f crc_sctp.h -f crc_tcpip.c -f crc_tcpip.h \
	-f crc_wimax.c -f crc_wimax.h -f crc_ether.c -f crc_ether.h \
	-f crc_cable.c -f crc_cable.h -f main.c -f crcext.h

CPPCHECK?=cppcheck
.PHONY: cppcheck
cppcheck:
	$(CPPCHECK) --enable=warning,portability,performance,unusedFunction,\
	missingInclude --std=c99 --template=gcc --suppress=uninitvar \
	crc.c crc.h crcr.c crcr.h crc_rnc.c crc_rnc.h crc_sctp.c crc_sctp.h \
	crc_tcpip.c crc_tcpip.h crc_wimax.c crc_wimax.h crc_ether.c \
	crc_ether.h crc_cable.c crc_cable.h main.c crcext.h
