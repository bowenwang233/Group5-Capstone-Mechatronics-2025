cmake_minimum_required(VERSION 3.20)
project(RoverDashboard LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt
find_package(Qt6 REQUIRED COMPONENTS Widgets Network)

# Source files
set(SOURCES
    src/main.cpp
    src/dashboard_window.cpp
    src/camera_canvas.cpp
    src/http_bridge.cpp
    src/gamepad_widget.cpp
    src/udp_receiver.cpp        # <- included
)

# Header files (optional, for IDE organization)
set(HEADERS
    src/dashboard_window.h
    src/ui_types.h
    src/camera_canvas.h
    src/http_bridge.h
    src/gamepad.h
    src/gamepad_widget.h
    src/udp_receiver.h          # <- included
)

add_executable(RoverDashboard ${SOURCES} ${HEADERS})

target_link_libraries(RoverDashboard PRIVATE Qt6::Widgets Qt6::Network)

if (WIN32)
    target_link_libraries(RoverDashboard PRIVATE Xinput)
endif()
