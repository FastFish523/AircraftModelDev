file(GLOB_RECURSE DIR_SRCS *.cpp *.h)
add_library(AIM120D SHARED ${DIR_SRCS})
target_compile_definitions(AIM120D PRIVATE SharedAIM120D_Build)
target_link_libraries(AIM120D Utils)

