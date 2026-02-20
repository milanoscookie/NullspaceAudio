
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

set_target_properties(DSPInterface PROPERTIES
  VERSION 1.0.0
  SOVERSION 1
)

