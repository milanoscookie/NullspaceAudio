
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFTW3 REQUIRED IMPORTED_TARGET fftw3f fftw3)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    "$<$<AND:$<COMPILE_LANGUAGE:CXX>,$<NOT:$<CONFIG:Debug>>>:-O3>"
    "$<$<AND:$<COMPILE_LANGUAGE:CXX>,$<NOT:$<CONFIG:Debug>>>:-march=native>"
  )
elseif(MSVC)
  add_compile_options(
    "$<$<AND:$<COMPILE_LANGUAGE:CXX>,$<NOT:$<CONFIG:Debug>>>:/O2>"
  )
endif()

# DSP Interface Library - builds independently
add_library(DSPInterface STATIC
  dsp_interface.cpp
  audio_source.cpp
  simulator_paths.cpp
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
