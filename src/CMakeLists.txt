# Main executable
add_executable(DSPApp main.cpp anc.cpp)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    # "$<$<AND:$<COMPILE_LANGUAGE:CXX>,$<NOT:$<CONFIG:Debug>>>:-O3>"
    # "$<$<AND:$<COMPILE_LANGUAGE:CXX>,$<NOT:$<CONFIG:Debug>>>:-march=native>"
  )
elseif(MSVC)
  add_compile_options(
    "$<$<AND:$<COMPILE_LANGUAGE:CXX>,$<NOT:$<CONFIG:Debug>>>:/O2>"
  )
endif()

# Link against the DSP interface library only
target_link_libraries(DSPApp PRIVATE DSPInterface)

# Ensure anc.h is found
target_include_directories(DSPApp PRIVATE ../src)

target_sources(DSPApp PRIVATE anc.h)
