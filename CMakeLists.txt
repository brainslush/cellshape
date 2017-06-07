cmake_minimum_required(VERSION 3.7)
project(Cellformation)
set(APP Cellformation)

set(CMAKE_CXX_STANDARD 11)

set(OF_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../of/" CACHE PATH "The root directory of ofnode/of project.")

include(${OF_ROOT}/openFrameworks.cmake)

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
        src/variable_type.h)

add_executable(
        ${APP}
        ${SOURCE_FILES}
        ${OFXADDONS_SOURCES}
)

target_link_libraries(
        ${APP}
        ${OPENFRAMEWORKS_LIBRARIES}
)