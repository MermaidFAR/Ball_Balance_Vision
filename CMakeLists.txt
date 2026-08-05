cmake_minimum_required(VERSION 3.16)

project(ti_rdk_vision LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(ti_vision
    src/main.cpp
    src/v4l2_camera.cpp
    src/mjpeg_decoder.cpp
    src/bpu_detector.cpp
    src/roi_extractor.cpp
    src/uart_publisher.cpp
    src/recording_store.cpp
    src/web_server.cpp
)

target_include_directories(ti_vision PRIVATE include)
target_compile_options(ti_vision PRIVATE -O3 -Wall -Wextra -Wpedantic)
target_link_directories(ti_vision PRIVATE /usr/lib /usr/lib/hbbpu /usr/hobot/lib)
target_link_libraries(ti_vision PRIVATE dnn multimedia hbmem pthread)

install(TARGETS ti_vision RUNTIME DESTINATION bin)

add_executable(ti_calibrate_jpeg
    src/calibrate_jpeg.cpp
    src/mjpeg_decoder.cpp
    src/bpu_detector.cpp
    src/roi_extractor.cpp
)
target_include_directories(ti_calibrate_jpeg PRIVATE include)
target_compile_options(ti_calibrate_jpeg PRIVATE -O3 -Wall -Wextra -Wpedantic)
target_link_directories(ti_calibrate_jpeg PRIVATE /usr/lib /usr/lib/hbbpu /usr/hobot/lib)
target_link_libraries(ti_calibrate_jpeg PRIVATE dnn multimedia hbmem pthread)
