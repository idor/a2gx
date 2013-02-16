#
# Copyright (C) 2013 Vlad Lazarenko <vlad@lazarenko.me>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

MAKEFLAGS += --no-builtin-rules

PROJECT_DIR := $(PWD)

export PROJECT_NAME = a2gx

SETUP_SCRIPT = $(PROJECT_NAME)_create.tcl
SDC_FILE = $(PROJECT_NAME).sdc
PRJ_FILE = $(PROJECT_NAME).prj

BUILD_DIR = $(PROJECT_DIR)/build

QSYS_TOP = $(PROJECT_NAME).qsys
QSYS_QIP = $(BUILD_DIR)/$(PROJECT_NAME).qip

export RTL_FILES = $(wildcard *.v)
export QIP_FILES = $(QSYS_QIP)

# Quartus II Project Generation
$(PRJ_FILE): $(SETUP_SCRIPT) $(SDC_FILE)
	quartus_sh -t $(SETUP_SCRIPT)
	@touch $(PRJ_FILE)

# QSys System Generation
generate_qsys: $(QSYS_QIP)

$(QSYS_QIP): $(QSYS_TOP)
	ip-generate \
		--project-directory=$(PROJECT_DIR) \
		--output-directory=$(BUILD_DIR) \
		--file-set=QUARTUS_SYNTH \
		--system-info=DEVICE_FAMILY="Arria II GX" \
		--system-info=DEVICE="EP2AGX260FF35I3" \
		--system-info=DEVICE_SPEEDGRADE=3 \
		--system-info=GENERATION_ID=0 \
		--component-file=$(QSYS_TOP) \
		--output-name=$(PROJECT_NAME)_qsys \
		--report-file=qip:$(QSYS_QIP) \
		--report-file=html:$(BUILD_DIR)/$(PROJECT_NAME)_report.html
