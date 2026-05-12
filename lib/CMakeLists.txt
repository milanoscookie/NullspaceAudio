
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFTW3 REQUIRED IMPORTED_TARGET fftw3f fftw3)

# DSP Interface Library - builds independently
add_library(DSPInterface STATIC
  dsp_interface.cpp
  audio_source.cpp
  wav_file_source.cpp
  wav_writer.cpp
)


# Include dirs for consumers
target_include_directories(DSPInterface
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/_deps/eigen
)

target_compile_definitions(DSPInterface PUBLIC EIGEN_FFTW_DEFAULT)

target_link_libraries(DSPInterface
  PUBLIC
    PkgConfig::FFTW3
)

set_target_properties(DSPInterface PROPERTIES
  VERSION 1.0.0
  SOVERSION 1
)
