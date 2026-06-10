file(GLOB_RECURSE DIR_SRCS *.cpp *.h)
add_library(HXD3530 SHARED ${DIR_SRCS})
target_compile_definitions(HXD3530 PRIVATE SharedHXD3530_Build)
target_link_libraries(HXD3530 Utils)

