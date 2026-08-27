file(GLOB_RECURSE DIR_SRCS *.cpp *.h)
add_library(HTV2 SHARED ${DIR_SRCS})
target_compile_definitions(HTV2 PRIVATE SharedHTV2_Build)
target_link_libraries(HTV2 Utils)

