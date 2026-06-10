file(GLOB_RECURSE DIR_SRCS *.cpp *.h)
add_library(PAC500 SHARED ${DIR_SRCS})
target_compile_definitions(PAC500 PRIVATE SharedPAC500_Build)
target_link_libraries(PAC500 Utils)

