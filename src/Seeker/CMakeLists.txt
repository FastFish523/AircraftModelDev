file(GLOB_RECURSE DIR_SRCS *.cpp *.h)
add_library(CommonSeeker SHARED ${DIR_SRCS})
target_compile_definitions(CommonSeeker PRIVATE SharedCommonSeeker_Build)
target_link_libraries(CommonSeeker Utils)

