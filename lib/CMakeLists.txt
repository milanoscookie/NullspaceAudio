
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFTW3 REQUIRED IMPORTED_TARGET fftw3f fftw3)

add_compile_options(
  "$<$<AND:$<COMPILE_LANGUAGE:CXX>,$<NOT:$<CONFIG:Debug>>>:-O3>"
  "$<$<AND:$<COMPILE_LANGUAGE:CXX>,$<NOT:$<CONFIG:Debug>>>:-march=native>"
)

file(GLOB DSPINTERFACE_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

file(GLOB_RECURSE DSPINTERFACE_HEADERS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
)

# DSP Interface Library - builds independently
add_library(DSPInterface STATIC ${DSPINTERFACE_SOURCES} ${DSPINTERFACE_HEADERS})


# Include dirs for consumers
target_include_directories(DSPInterface
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
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
