file(GLOB_RECURSE DIR_SRCS *.cpp *.h)
add_library(GPI SHARED ${DIR_SRCS})
target_compile_definitions(GPI PRIVATE SharedGPI_Build)
target_link_libraries(GPI Utils)

