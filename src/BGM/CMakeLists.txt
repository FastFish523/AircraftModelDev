file(GLOB_RECURSE DIR_SRCS *.cpp *.h)
add_library(BGM SHARED ${DIR_SRCS})
target_compile_definitions(BGM PRIVATE SharedBGM_Build)
target_link_libraries(BGM Utils)

