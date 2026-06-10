file(GLOB_RECURSE DIR_SRCS *.cpp *.h)
add_library(AIM9 SHARED ${DIR_SRCS})
target_compile_definitions(AIM9 PRIVATE SharedAIM9_Build)
target_link_libraries(AIM9 Utils)

