# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(opendiag)

idf_build_get_property(firmware_version PROJECT_VER)
string(LENGTH "${firmware_version}" firmware_version_length)
if(firmware_version_length LESS 1 OR firmware_version_length GREATER 31)
    message(FATAL_ERROR "Firmware version must be 1-31 bytes: ${firmware_version}")
endif()
if(CONFIG_APP_EXCLUDE_PROJECT_VER_VAR)
    message(FATAL_ERROR "OpenDIAG requires the embedded project version")
endif()

idf_build_get_property(python PYTHON)
add_custom_target(firmware-package
    COMMAND "${python}" "${CMAKE_SOURCE_DIR}/tools/package_firmware.py"
            --build-dir "${CMAKE_BINARY_DIR}"
    DEPENDS app bootloader partition_table_bin
    COMMENT "Packaging firmware for distribution"
    USES_TERMINAL
    VERBATIM
)

# make format / idf.py format - reformat main/ and test/host/ in place.
# make format-check / idf.py format-check - same, but fails without touching
# anything (for CI).
add_custom_target(format
    COMMAND ${CMAKE_SOURCE_DIR}/tools/format.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Formatting main/ and test/host/ with clang-format"
    USES_TERMINAL
)
add_custom_target(format-check
    COMMAND ${CMAKE_SOURCE_DIR}/tools/format.sh --check
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Checking main/ and test/host/ formatting"
    USES_TERMINAL
)
