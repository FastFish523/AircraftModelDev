
add_compile_definitions(Model_Build)
file(GLOB_RECURSE DIR_SRCS *.cpp)

add_library(OrbitModel SHARED ${DIR_SRCS})
target_compile_definitions(OrbitModel PRIVATE SharedOrbitModel_Build)
target_link_libraries(OrbitModel Utils)

find_package(Python REQUIRED COMPONENTS Development)

find_program(SWIG_EXECUTABLE swig REQUIRED)

set(SWIG_LIB_DIR "/usr/share/swig4.0")
set(BUILD_DIR ${PROJECT_SOURCE_DIR}/build)

# ---------- 手动生成 SWIG wrapper ----------
set(SWIG_WRAP_CXX ${BUILD_DIR}/orbitModel_wrap.cxx)
set(SWIG_PY      ${BUILD_DIR}/orbitModel.py)
set(SWIG_SRC "swig/orbitModel.i")

add_custom_command(
        OUTPUT ${SWIG_WRAP_CXX} ${SWIG_PY}
        COMMAND ${SWIG_EXECUTABLE}
        -python
        -c++
        -I${PROJECT_SOURCE_DIR}/include
        -I${SWIG_LIB_DIR}/python
        -outdir ${BUILD_DIR}
        -o ${SWIG_WRAP_CXX}
        ${PROJECT_SOURCE_DIR}/${SWIG_SRC}
        DEPENDS ${PROJECT_SOURCE_DIR}/${SWIG_SRC}
        COMMENT "Running SWIG"
)

# ---------- 生成 Python 模块 ----------
add_library(orbitModelPy MODULE
        ${DIR_SRCS}
        ${SWIG_WRAP_CXX}
)

target_include_directories(orbitModelPy PRIVATE
        ${Python_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
        ${SWIG_LIB_DIR}/python
)

target_link_libraries(orbitModelPy PRIVATE
        ${Python_LIBRARIES}
        Utils
)

set_target_properties(orbitModelPy PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${BUILD_DIR}
        PREFIX "_"
        SUFFIX ".so"
)


