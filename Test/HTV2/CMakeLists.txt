add_executable(HTV2_Test main.cpp)
target_link_libraries(HTV2_Test PRIVATE HTV2 Utils)

add_executable(HTV2_Dive_Test dive_main.cpp)
target_link_libraries(HTV2_Dive_Test PRIVATE HTV2 Utils)

add_executable(HTV2_Boost_Test Boosttest.cpp)
target_link_libraries(HTV2_Boost_Test PRIVATE HTV2 Utils)

add_executable(HTV2_Midcourse_Test midcourse_main.cpp)
target_link_libraries(HTV2_Midcourse_Test PRIVATE HTV2 Utils)
