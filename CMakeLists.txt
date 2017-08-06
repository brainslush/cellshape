cmake_minimum_required(VERSION 3.7)
project(Cellformation)
set(APP Cellformation)

set(OF_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../of/" CACHE PATH "The root directory of ofnode/of project.")

include(${OF_ROOT}/openFrameworks.cmake)

## Compiler options
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")  ## Optimize disabled
set(CMAKE_CXX_FLAGS_RELEASE "-O3") ## Maximum opt level
##set(CMAKE_EXE_LINKER_FLAGS "-s")  ## Strip binary

ofxaddon(ofxDatGui)

set(SOURCE_FILES
        src/base.cpp
        src/base.h
        src/cActin.cpp
        src/cActin.h
        src/cBaseclasses.cpp
        src/cBaseclasses.h
        src/cCell.cpp
        src/cCell.h
        src/cFac.cpp
        src/cFac.h
        src/cMembrane.cpp
        src/cMembrane.h
        src/cSurface.cpp
        src/cSurface.h
        src/extIncludes.h
        src/globalVars.h
        src/grid.cpp
        src/grid.h
        src/gui.cpp
        src/gui.h
        src/main.cpp
        src/ofApp.cpp
        src/ofApp.h
        src/random.cpp
        src/random.h
        src/RigidBody.cpp
        src/RigidBody.h
        src/settings.h
        src/variable_type.h
        src/cFunctors.cpp
        src/cFunctors.h
        src/std.cpp
        src/std.h
        src/bmath.h
        src/typeRegistrar.h src/ignoreList.h src/typeRegistrar.cpp src/ignoreList.cpp)

add_executable(
        ${APP}
        ${SOURCE_FILES}
        ${OFXADDONS_SOURCES}
)

target_link_libraries(
        ${APP}
        ${OPENFRAMEWORKS_LIBRARIES}
)

add_custom_command(
        TARGET ${APP} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${OF_ROOT}/addons/ofxDatGui/ofxbraitsch $<TARGET_FILE_DIR:${APP}>/data/ofxbraitsch
)