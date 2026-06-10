file(GLOB_RECURSE DIR_SRCS *.cpp *.h)
add_library(HACM SHARED ${DIR_SRCS})
target_compile_definitions(HACM PRIVATE SharedAGM86_Build)
target_link_libraries(HACM Utils)

