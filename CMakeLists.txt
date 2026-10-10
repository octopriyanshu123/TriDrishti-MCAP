cmake_minimum_required(VERSION 3.22)

project(i2w_mcap_example LANGUAGES CXX)

add_executable(mcap_example_writer src/writer.cpp)
add_executable(mcap_example_reader src/reader.cpp)

foreach(target mcap_example_writer mcap_example_reader)
  target_include_directories(${target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
  target_compile_definitions(${target} PRIVATE MCAP_COMPRESSION_NO_LZ4 MCAP_COMPRESSION_NO_ZSTD)
  target_compile_features(${target} PRIVATE cxx_std_17)
endforeach()

# The writer logs each channel from its own thread.
find_package(Threads REQUIRED)
target_link_libraries(mcap_example_writer PRIVATE Threads::Threads)
