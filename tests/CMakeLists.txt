# Tests
enable_testing()

file(GLOB TEST_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp"
)

# One executable per test file
foreach(src ${TEST_SOURCES})
  get_filename_component(test_name ${src} NAME_WE)
  add_executable(${test_name} ${src})
  target_link_libraries(${test_name} PRIVATE DSPInterface)
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
