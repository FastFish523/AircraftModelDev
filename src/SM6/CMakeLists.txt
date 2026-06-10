file(GLOB_RECURSE DIR_SRCS *.cpp *.h)
add_library(SM6 SHARED ${DIR_SRCS})
target_compile_definitions(SM6 PRIVATE SharedSM6_Build)
target_link_libraries(SM6 Utils)

