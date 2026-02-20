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

# One executable per test file
foreach(src ${TEST_SOURCES})
  get_filename_component(test_name ${src} NAME_WE)
  add_executable(${test_name} ${src})
  target_link_libraries(${test_name} PRIVATE DSPInterface)
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
