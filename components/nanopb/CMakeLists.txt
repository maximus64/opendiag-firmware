if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/upstream/pb.h")
    message(FATAL_ERROR "Nanopb submodule missing. Run: git submodule update --init --recursive")
endif()

idf_component_register(SRCS upstream/pb_common.c upstream/pb_encode.c upstream/pb_decode.c
                       INCLUDE_DIRS "upstream")
