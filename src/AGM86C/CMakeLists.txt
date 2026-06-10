file(GLOB_RECURSE DIR_SRCS *.cpp *.h)
add_library(AGM86C SHARED ${DIR_SRCS})
target_compile_definitions(AGM86C PRIVATE SharedAGM86C_Build)
target_link_libraries(AGM86C Utils)

