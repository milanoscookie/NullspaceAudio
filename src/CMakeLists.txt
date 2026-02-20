# Main executable
add_executable(DSPApp main.cpp anc.cpp)


# Link against the DSP interface library only
target_link_libraries(DSPApp PRIVATE DSPInterface)

# Ensure anc.h is found
target_include_directories(DSPApp PRIVATE ../src)

target_sources(DSPApp PRIVATE anc.h)
