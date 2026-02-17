# Tests
enable_testing()

# All test source files
set(TEST_SOURCES
  test_ring_buffer.cpp
  test_double_buffer_spsc.cpp
  test_iir_filter.cpp
  test_lp_butterworth.cpp
  test_linear_system.cpp
  test_wav_writer.cpp
  test_dsp_interface.cpp
)

# Find ALSA (needed by DSPInterface -> PortAudio)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ALSA REQUIRED alsa)

# One executable per test file
foreach(src ${TEST_SOURCES})
  get_filename_component(test_name ${src} NAME_WE)
  add_executable(${test_name} ${src})
  target_link_libraries(${test_name} PRIVATE DSPInterface ${ALSA_LIBRARIES})
  target_include_directories(${test_name} PRIVATE ${ALSA_INCLUDE_DIRS})
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
