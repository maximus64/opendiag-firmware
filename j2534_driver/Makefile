# SPDX-License-Identifier: GPL-3.0-only
.DEFAULT_GOAL := all
.DELETE_ON_ERROR:
SHELL := /bin/sh

CONFIG ?= release
PYTHON ?= python3
CROSS_COMPILE ?=

ifneq ($(CROSS_COMPILE),)
ifeq ($(origin CC),default)
CC := $(CROSS_COMPILE)gcc
endif
ifeq ($(origin CXX),default)
CXX := $(CROSS_COMPILE)g++
endif
endif

# The one place the target OS is decided; everything below keys off TARGET_OS.
TARGET_TRIPLE := $(shell $(CC) -dumpmachine)
ifneq ($(findstring -mingw32,$(TARGET_TRIPLE)),)
TARGET_OS := win32
else ifneq ($(findstring -darwin,$(TARGET_TRIPLE)),)
TARGET_OS := darwin
else ifneq ($(findstring -linux,$(TARGET_TRIPLE)),)
TARGET_OS := linux
else
$(error Unsupported target "$(TARGET_TRIPLE)" from $(CC); expected Linux, macOS or MinGW)
endif
BUILD_DIR ?= build/$(TARGET_OS)/$(CONFIG)

NANOPB_DIR := ../components/nanopb/upstream
PROTOCOL_DIR := ../protocol
GENERATED_DIR := $(BUILD_DIR)/generated
NANOPB_GENERATOR := $(NANOPB_DIR)/generator/nanopb_generator.py
NANOPB_GENERATOR_DEPS := $(NANOPB_GENERATOR) \
    $(NANOPB_DIR)/generator/proto/__init__.py \
    $(NANOPB_DIR)/generator/proto/_utils.py \
    $(NANOPB_DIR)/generator/proto/nanopb.proto

ifeq ($(filter $(CONFIG),release debug),)
$(error CONFIG must be release or debug)
endif

CPPFLAGS += -Iinclude -I$(NANOPB_DIR) -I$(GENERATED_DIR)
CFLAGS += -std=c11 -Wall -Wextra -Werror -MMD -MP
CXXFLAGS += -std=c++11 -Wall -Wextra -Werror -MMD -MP
ifeq ($(CONFIG),debug)
CFLAGS += -Og -g
CXXFLAGS += -Og -g
else
CFLAGS += -O2
CXXFLAGS += -O2
endif

PLATFORM_COMMON := platform/common.cpp
CORE := driver/api.cpp driver/exports.cpp driver/link.cpp common/api_trace.cpp
LEGACY := legacy/legacy.cpp legacy/exports.cpp legacy/backend.cpp legacy/dllmain.cpp common/api_trace.cpp
NANOPB_SOURCES := pb_common.c pb_encode.c pb_decode.c
GUI := config/main.cpp config/settings.cpp config/connection.cpp

ifeq ($(TARGET_OS),win32)
RC ?= $(CROSS_COMPILE)windres
CPPFLAGS += -DWIN32_LEAN_AND_MEAN -DNOMINMAX -DWINVER=0x0501 -D_WIN32_WINNT=0x0501
CPPFLAGS += -D__USE_MINGW_ANSI_STDIO=1
CFLAGS += -march=i686 -mtune=generic
CXXFLAGS += -march=i686 -mtune=generic
LDFLAGS += -static -static-libgcc -static-libstdc++ -Wl,--no-insert-timestamp
LDFLAGS += -Wl,--major-subsystem-version,5,--minor-subsystem-version,1
LDFLAGS += -Wl,--subsystem,windows
PLATFORM := platform/windows.cpp
LDLIBS += -lws2_32
NATIVE_LIB := $(BUILD_DIR)/opendiag32.dll
LEGACY_LIB := $(BUILD_DIR)/odg40432.dll
PRODUCTS := $(NATIVE_LIB) $(LEGACY_LIB) $(BUILD_DIR)/opendiag_config.exe
CORE_LINK := driver/opendiag32.def $(BUILD_DIR)/driver/version.res
LEGACY_LINK := legacy/odg40432.def $(BUILD_DIR)/legacy/version.res
# The linker's default image base depends on the output path.
CORE_LDFLAGS := -Wl,--image-base,0x65000000
LEGACY_LDFLAGS := -Wl,--image-base,0x66000000
else
ifeq ($(TARGET_OS),linux)
CPPFLAGS += -D_GNU_SOURCE
LDLIBS += -ldl
SHARED_EXT := so
else
SHARED_EXT := dylib
endif
PLATFORM := platform/posix.cpp platform/$(TARGET_OS).cpp
CFLAGS += -fPIC -fvisibility=hidden
CXXFLAGS += -fPIC -fvisibility=hidden -fvisibility-inlines-hidden
LDLIBS += -pthread
NATIVE_LIB := $(BUILD_DIR)/libopendiag.$(SHARED_EXT)
LEGACY_LIB := $(BUILD_DIR)/libopendiag0404.$(SHARED_EXT)
PRODUCTS := $(NATIVE_LIB) $(LEGACY_LIB)
endif

PLATFORM_SOURCES := $(PLATFORM_COMMON) $(PLATFORM)
objects = $(addprefix $(BUILD_DIR)/,$(patsubst %.c,%.o,$(patsubst %.cpp,%.o,$(1))))
NANOPB_OBJECTS := $(addprefix $(BUILD_DIR)/nanopb/,$(NANOPB_SOURCES:.c=.o))
CORE_OBJECTS := $(call objects,$(CORE) $(PLATFORM_SOURCES)) $(NANOPB_OBJECTS) $(GENERATED_DIR)/j2534.pb.o
LEGACY_OBJECTS := $(call objects,$(LEGACY) $(PLATFORM_SOURCES))
GUI_OBJECTS := $(call objects,$(GUI) $(PLATFORM_SOURCES))
CONFIG_STAMP := $(BUILD_DIR)/.build-config

.PHONY: FORCE
$(CONFIG_STAMP): FORCE
	@mkdir -p "$(@D)"
	@$(PYTHON) tools/build_config.py "$@" "$(CC)" "$(CXX)" "$(RC)" "$(CPPFLAGS)" "$(CFLAGS)" "$(CXXFLAGS)" "$(LDFLAGS)" "$(LDLIBS)"

$(sort $(CORE_OBJECTS) $(LEGACY_OBJECTS) $(GUI_OBJECTS)) $(BUILD_DIR)/tests/abi0404.o $(BUILD_DIR)/tests/abi0500.o: $(CONFIG_STAMP) Makefile
$(call objects,$(CORE)): $(GENERATED_DIR)/j2534.pb.h
ifeq ($(TARGET_OS),win32)
$(BUILD_DIR)/driver/version.res $(BUILD_DIR)/legacy/version.res $(BUILD_DIR)/config/config.res: $(CONFIG_STAMP) Makefile
$(BUILD_DIR)/config/config.res: config/resource.h
endif

.PHONY: all clean test repeat-test check-toolchain check-wincompat package
all: $(PRODUCTS) $(BUILD_DIR)/opendiag.ini

$(addprefix $(NANOPB_DIR)/,$(NANOPB_SOURCES) pb.h) $(NANOPB_GENERATOR_DEPS):
	@echo "Nanopb submodule missing. Run: git submodule update --init --recursive" >&2; exit 1

# Both outputs come from one generator invocation, including parallel builds.
$(GENERATED_DIR)/j2534.pb.c $(GENERATED_DIR)/j2534.pb.h &: $(PROTOCOL_DIR)/j2534.proto $(PROTOCOL_DIR)/j2534.options $(NANOPB_GENERATOR_DEPS) Makefile
	@mkdir -p "$(GENERATED_DIR)"
	NANOPB_PB2_TEMP_DIR="$(abspath $(GENERATED_DIR))" $(PYTHON) -B "$(NANOPB_GENERATOR)" -I "$(PROTOCOL_DIR)" -D "$(GENERATED_DIR)" "$<"

ifeq ($(TARGET_OS),win32)
$(CORE_OBJECTS) $(LEGACY_OBJECTS) $(GUI_OBJECTS): | check-toolchain
check-toolchain:
	@$(PYTHON) tools/check_toolchain.py --cc "$(CC)" --cxx "$(CXX)"
else
check-toolchain:
	@true
endif

$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p "$(@D)"
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c "$<" -o "$@"

$(BUILD_DIR)/%.o: %.c
	@mkdir -p "$(@D)"
	$(CC) $(CPPFLAGS) $(CFLAGS) -c "$<" -o "$@"

$(BUILD_DIR)/nanopb/%.o: $(NANOPB_DIR)/%.c $(NANOPB_DIR)/pb.h
	@mkdir -p "$(@D)"
	$(CC) $(CPPFLAGS) $(CFLAGS) -c "$<" -o "$@"

$(GENERATED_DIR)/%.o: $(GENERATED_DIR)/%.c $(GENERATED_DIR)/%.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -c "$<" -o "$@"

$(BUILD_DIR)/%.res: %.rc
	@mkdir -p "$(@D)"
	$(RC) $(CPPFLAGS) -i "$<" -o "$@" -O coff

$(NATIVE_LIB): $(CORE_OBJECTS) $(CORE_LINK)
	$(CXX) -shared $(LDFLAGS) $(CORE_LDFLAGS) -o "$@" $^ $(LDLIBS)

$(LEGACY_LIB): $(LEGACY_OBJECTS) $(LEGACY_LINK)
	$(CXX) -shared $(LDFLAGS) $(LEGACY_LDFLAGS) -o "$@" $^ $(LDLIBS)

$(BUILD_DIR)/opendiag_config.exe: $(GUI_OBJECTS) $(BUILD_DIR)/config/config.res
	$(CXX) -mwindows $(LDFLAGS) -o "$@" $^ $(LDLIBS) -luser32 -lgdi32 -lshell32 -ladvapi32 -lversion

$(BUILD_DIR)/opendiag.ini:
	@mkdir -p "$(@D)"
	cp opendiag.ini "$@"

ifneq ($(TARGET_OS),win32)
API_TESTS := $(BUILD_DIR)/tests/api0500 $(BUILD_DIR)/tests/api0404
HOST_TESTS := $(BUILD_DIR)/tests/platform_test $(API_TESTS) $(BUILD_DIR)/tests/abi0404.o $(BUILD_DIR)/tests/abi0500.o
TEST_OBJECTS := $(call objects,tests/platform_test.cpp tests/api0500.cpp tests/api0404.cpp)
$(TEST_OBJECTS): $(CONFIG_STAMP) Makefile
$(BUILD_DIR)/tests/platform_test: $(call objects,tests/platform_test.cpp common/api_trace.cpp $(PLATFORM_SOURCES))
	$(CXX) $(LDFLAGS) -o "$@" $^ $(LDLIBS)
$(API_TESTS): $(BUILD_DIR)/tests/%: $(BUILD_DIR)/tests/%.o $(call objects,$(PLATFORM_SOURCES))
	$(CXX) $(LDFLAGS) -o "$@" $^ $(LDLIBS)
LIBRARY_ARGS = --native "$(abspath $(NATIVE_LIB))" --legacy "$(abspath $(LEGACY_LIB))"

test: all repeat-test $(HOST_TESTS)
	$(PYTHON) tests/run_tests.py --build "$(abspath $(BUILD_DIR))" $(LIBRARY_ARGS)
HOST_TEST_BUILD ?= ../test/host/build
$(GENERATED_DIR)/j2534_pb2.py: $(PROTOCOL_DIR)/j2534.proto
	@mkdir -p "$(@D)"
	protoc -I "$(PROTOCOL_DIR)" --python_out="$(GENERATED_DIR)" "$<"

repeat-test: all $(GENERATED_DIR)/j2534_pb2.py
	cmake -S ../test/host -B "$(HOST_TEST_BUILD)"
	cmake --build "$(HOST_TEST_BUILD)" --target j2534_contract_firmware
	OPENDIAG_TEST_PROTO_DIR="$(abspath $(GENERATED_DIR))" \
		$(PYTHON) tests/test_repeat_messages.py $(LIBRARY_ARGS) \
		--firmware "$(abspath $(HOST_TEST_BUILD))/libj2534_contract_firmware.$(SHARED_EXT)"
check-wincompat package:
	@echo "$@ requires Windows artifacts; use: docker compose run --rm --build j2534-driver" >&2; exit 2
else
test repeat-test:
	@echo "Unit tests are native host only: make test" >&2; exit 2
check-wincompat: all $(BUILD_DIR)/tests/abi0404.o $(BUILD_DIR)/tests/abi0500.o
	$(PYTHON) tools/check_wincompat.py "$(BUILD_DIR)"
package: check-wincompat
	$(PYTHON) tools/package.py "$(BUILD_DIR)"
endif

clean:
	$(PYTHON) -c 'import shutil; shutil.rmtree("$(BUILD_DIR)", ignore_errors=True)'

-include $(sort $(CORE_OBJECTS:.o=.d) $(LEGACY_OBJECTS:.o=.d) $(GUI_OBJECTS:.o=.d) \
    $(TEST_OBJECTS:.o=.d) $(BUILD_DIR)/tests/abi0404.d $(BUILD_DIR)/tests/abi0500.d)
