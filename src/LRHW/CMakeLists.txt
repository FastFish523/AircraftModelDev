file(GLOB_RECURSE DIR_SRCS *.cpp *.h)
add_library(LRHW SHARED ${DIR_SRCS})
target_compile_definitions(LRHW PRIVATE SharedLRHW_Build)
target_link_libraries(LRHW Utils)

