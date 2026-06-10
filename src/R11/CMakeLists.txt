file(GLOB_RECURSE DIR_SRCS *.cpp *.h)
add_library(R11 SHARED ${DIR_SRCS})
target_compile_definitions(R11 PRIVATE SharedR11_Build)
target_link_libraries(R11 Utils)

