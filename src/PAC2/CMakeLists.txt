file(GLOB_RECURSE DIR_SRCS *.cpp *.h)
add_library(PAC2 SHARED ${DIR_SRCS})
target_link_libraries(PAC2 Utils)
target_compile_definitions(PAC2 PRIVATE SharedModel_Build)