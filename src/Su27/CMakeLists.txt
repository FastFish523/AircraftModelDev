file(GLOB_RECURSE DIR_SRCS *.cpp *.h)
add_library(Su27 SHARED ${DIR_SRCS})
target_compile_definitions(Su27 PRIVATE SharedSu27_Build)
target_link_libraries(Su27 Utils)

