cmake_minimum_required(VERSION 3.22)

project(i2w_mcap_example LANGUAGES CXX)

add_executable(mcap_example_writer src/writer.cpp)
add_executable(mcap_example_reader src/reader.cpp)
add_executable(mcap_image_writer src/imageWriter.cpp)
add_executable(mcap_proto_image_writer src/protoImageWriter.cpp)
add_executable(mcap_compressed_proto_image_writer src/compressedProtoImageWrite.cpp)

foreach(target mcap_example_writer mcap_example_reader mcap_image_writer mcap_proto_image_writer
               mcap_compressed_proto_image_writer)
  target_include_directories(${target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
  target_compile_definitions(${target} PRIVATE MCAP_COMPRESSION_NO_LZ4 MCAP_COMPRESSION_NO_ZSTD)
  target_compile_features(${target} PRIVATE cxx_std_17)
endforeach()

# The writer logs each channel from its own thread.
find_package(Threads REQUIRED)
target_link_libraries(mcap_example_writer PRIVATE Threads::Threads)

# Generated C++ for the Foxglove protobuf schemas in proto/ (e.g. foxglove/CompressedImage.pb.h).
find_package(Protobuf REQUIRED)
set(PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto")
set(PROTO_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/proto_gen")
file(MAKE_DIRECTORY "${PROTO_GEN_DIR}")
set(PROTO_SRCS)
foreach(name CompressedImage CompressedVideo)
  add_custom_command(
    OUTPUT "${PROTO_GEN_DIR}/foxglove/${name}.pb.cc" "${PROTO_GEN_DIR}/foxglove/${name}.pb.h"
    COMMAND protobuf::protoc --cpp_out "${PROTO_GEN_DIR}" -I "${PROTO_DIR}" "${PROTO_DIR}/foxglove/${name}.proto"
    DEPENDS "${PROTO_DIR}/foxglove/${name}.proto"
    COMMENT "Generating C++ from foxglove/${name}.proto")
  list(APPEND PROTO_SRCS "${PROTO_GEN_DIR}/foxglove/${name}.pb.cc")
endforeach()
add_library(foxglove_proto STATIC ${PROTO_SRCS})
target_include_directories(foxglove_proto PUBLIC "${PROTO_GEN_DIR}")
target_link_libraries(foxglove_proto PUBLIC protobuf::libprotobuf)
target_link_libraries(mcap_proto_image_writer PRIVATE foxglove_proto)

# H.264 encoding (libx264 through FFmpeg) for the CompressedVideo writer.
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavcodec libavutil libswscale)
target_link_libraries(mcap_compressed_proto_image_writer PRIVATE foxglove_proto PkgConfig::FFMPEG)
