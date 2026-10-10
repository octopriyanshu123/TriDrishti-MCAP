cmake_minimum_required(VERSION 3.22)

project(i2w_mcap_example LANGUAGES CXX)

add_executable(mcap_example_writer src/writer.cpp)
add_executable(mcap_example_reader src/reader.cpp)
add_executable(mcap_image_writer src/imageWriter.cpp)
add_executable(mcap_proto_image_writer src/protoImageWriter.cpp)
add_executable(mcap_compressed_proto_image_writer src/compressedProtoImageWrite.cpp)
add_executable(mcap_pointcloud_writer src/pointCloud.cpp)
add_executable(mcap_logger src/logger.cpp)

foreach(target mcap_example_writer mcap_example_reader mcap_image_writer mcap_proto_image_writer
               mcap_compressed_proto_image_writer mcap_pointcloud_writer mcap_logger)
  target_include_directories(${target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
  target_compile_features(${target} PRIVATE cxx_std_17)
endforeach()

# Compression is off everywhere except the point cloud writer, which uses zstd
# (point clouds are raw floats; JPEG/H.264 don't compress), and the reader, which
# must be able to open those files.
foreach(target mcap_example_writer mcap_image_writer mcap_proto_image_writer
               mcap_compressed_proto_image_writer)
  target_compile_definitions(${target} PRIVATE MCAP_COMPRESSION_NO_LZ4 MCAP_COMPRESSION_NO_ZSTD)
endforeach()
target_compile_definitions(mcap_pointcloud_writer PRIVATE MCAP_COMPRESSION_NO_LZ4)
target_compile_definitions(mcap_logger PRIVATE MCAP_COMPRESSION_NO_LZ4)
target_compile_definitions(mcap_example_reader PRIVATE MCAP_COMPRESSION_NO_LZ4)

# The writer logs each channel from its own thread.
find_package(Threads REQUIRED)
target_link_libraries(mcap_example_writer PRIVATE Threads::Threads)
target_link_libraries(mcap_pointcloud_writer PRIVATE Threads::Threads)
target_link_libraries(mcap_logger PRIVATE Threads::Threads)

# Generated C++ for the protobuf schemas in proto/ (e.g. foxglove/CompressedImage.pb.h).
# One protoc call for all files, so headers that import each other (PointCloud -> Pose
# -> Vector3) all exist before anything compiles.
find_package(Protobuf REQUIRED)
set(PROTO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto")
set(PROTO_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/proto_gen")
file(MAKE_DIRECTORY "${PROTO_GEN_DIR}")
set(PROTO_NAMES
  foxglove/CompressedImage foxglove/CompressedVideo
  foxglove/Vector3 foxglove/Quaternion foxglove/Pose foxglove/PackedElementField
  foxglove/PointCloud foxglove/LaserScan
  i2w_example/Range)
set(PROTO_FILES)
set(PROTO_SRCS)
set(PROTO_HDRS)
foreach(name ${PROTO_NAMES})
  list(APPEND PROTO_FILES "${PROTO_DIR}/${name}.proto")
  list(APPEND PROTO_SRCS "${PROTO_GEN_DIR}/${name}.pb.cc")
  list(APPEND PROTO_HDRS "${PROTO_GEN_DIR}/${name}.pb.h")
endforeach()
add_custom_command(
  OUTPUT ${PROTO_SRCS} ${PROTO_HDRS}
  COMMAND protobuf::protoc --cpp_out "${PROTO_GEN_DIR}" -I "${PROTO_DIR}" ${PROTO_FILES}
  DEPENDS ${PROTO_FILES}
  COMMENT "Generating C++ from proto/")
add_library(foxglove_proto STATIC ${PROTO_SRCS} ${PROTO_HDRS})
target_include_directories(foxglove_proto PUBLIC "${PROTO_GEN_DIR}")
target_link_libraries(foxglove_proto PUBLIC protobuf::libprotobuf)
target_link_libraries(mcap_proto_image_writer PRIVATE foxglove_proto)

# H.264 encoding (libx264 through FFmpeg) for the CompressedVideo writer.
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavcodec libavutil libswscale)
target_link_libraries(mcap_compressed_proto_image_writer PRIVATE foxglove_proto PkgConfig::FFMPEG)

# zstd chunk compression for the point cloud writer (and the reader, to open its files).
pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)
target_link_libraries(mcap_pointcloud_writer PRIVATE foxglove_proto PkgConfig::ZSTD)
target_link_libraries(mcap_example_reader PRIVATE PkgConfig::ZSTD)

# The combined logger: H.264 video, point clouds, plot and gauge, zstd-compressed.
target_link_libraries(mcap_logger PRIVATE foxglove_proto PkgConfig::FFMPEG PkgConfig::ZSTD)
