file(GLOB DSPAPP_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

file(GLOB DSPAPP_HEADERS CONFIGURE_DEPENDS
  "${CMAKE_SOURCE_DIR}/include/*.h"
)

# Main executable
add_executable(DSPApp ${DSPAPP_SOURCES} ${DSPAPP_HEADERS})

# Link against the DSP interface library only
target_link_libraries(DSPApp PRIVATE DSPInterface)

# Ensure anc.h is found
target_include_directories(DSPApp PRIVATE ${CMAKE_SOURCE_DIR}/include)
