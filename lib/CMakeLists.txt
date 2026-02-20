include(FetchContent)

FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)
FetchContent_MakeAvailable(eigen)

# DSP Interface Library - builds independently
add_library(DSPInterface STATIC
  dsp_interface.cpp
  audio_source.cpp
  wav_file_source.cpp
  wav_writer.cpp
)

# Include dirs for consumers (your own headers)
target_include_directories(DSPInterface
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link Eigen
target_link_libraries(DSPInterface
  PUBLIC Eigen3::Eigen
)

set_target_properties(DSPInterface PROPERTIES
  VERSION 1.0.0
  SOVERSION 1
)

