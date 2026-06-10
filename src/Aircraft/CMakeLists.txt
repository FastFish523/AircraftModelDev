file(GLOB_RECURSE DIR_SRCS *.cpp *.h)
add_library(Aircraft SHARED ${DIR_SRCS})
target_compile_definitions(Aircraft PRIVATE SharedAircraft_Build)
target_link_libraries(Aircraft Utils)

