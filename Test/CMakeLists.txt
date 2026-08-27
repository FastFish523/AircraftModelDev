add_executable(AGM86CTest AGM86C/main.cpp)
target_link_libraries(AGM86CTest AGM86C)

add_executable(AIM120DTest AIM120D/main.cpp)
target_link_libraries(AIM120DTest AIM120D )

add_executable(AIM9Test AIM9/main.cpp)
target_link_libraries(AIM9Test AIM9 )

add_executable(BGMTest BGM/main.cpp)
target_link_libraries(BGMTest BGM CommonSeeker)

add_executable(GPITest GPI/main.cpp)
target_link_libraries(GPITest GPI HXD3530)

add_executable(HACMTest HACM/main.cpp)
target_link_libraries(HACMTest HACM)


add_executable(HXDTest HXD/main.cpp)
target_link_libraries(HXDTest HXD3530)

add_subdirectory(HTV2)

add_executable(LRHWTest LRHW/main.cpp)
target_link_libraries(LRHWTest LRHW)

add_executable(PAC2Test PAC2/main.cpp)
target_link_libraries(PAC2Test PAC2)

add_executable(PAC500Test PAC500/main.cpp)
target_link_libraries(PAC500Test PAC500 HXD3530)

add_executable(R11Test R11/main.cpp)
target_link_libraries(R11Test R11)

add_executable(SM6Test SM6/main.cpp)
target_link_libraries(SM6Test SM6)

add_executable(Su27Test Su27/main.cpp)
target_link_libraries(Su27Test Su27)

add_executable(TheoreticalModelTest TheoreticalModel/main.cpp)
target_link_libraries(TheoreticalModelTest TheoreticalModel)

add_executable(AircrafTest Aircraft/main.cpp)
target_link_libraries(AircrafTest Aircraft )

add_executable(OrbitModelTest OrbitModel/main.cpp)
target_link_libraries(OrbitModelTest OrbitModel)

add_executable(link16Test Link16/main.cpp)

add_executable(seekerTest seekerTest/seekerTest.cpp)
target_link_libraries(seekerTest Utils)


