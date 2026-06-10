file(GLOB_RECURSE DIR_SRCS *.cpp *.h)
add_library(TheoreticalModel SHARED ${DIR_SRCS})
target_compile_definitions(TheoreticalModel PRIVATE SharedTheoreticalModel_Build)
target_link_libraries(TheoreticalModel Utils)

