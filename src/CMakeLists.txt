# Main executable
add_executable(DSPApp main.cpp)

# Find ALSA for suppressing error messages
find_package(PkgConfig REQUIRED)
pkg_check_modules(ALSA REQUIRED alsa)

# Link against the DSP interface library and ALSA
target_link_libraries(DSPApp PRIVATE DSPInterface ${ALSA_LIBRARIES})
target_include_directories(DSPApp PRIVATE ${ALSA_INCLUDE_DIRS})
