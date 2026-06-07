cmake_minimum_required(VERSION 3.10)

project(ai_inference)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

set(LIBS
    ${OpenCV_LIBS}
    hailort
)

set(INCLUDES
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(depth_infer depth_estim_test.cpp)
target_link_libraries(depth_infer PRIVATE ${LIBS})
target_include_directories(depth_infer PRIVATE ${INCLUDES})

add_executable(object_detector yolo_infer_test.cpp)
target_link_libraries(object_detector PRIVATE ${LIBS})
target_include_directories(object_detector PRIVATE ${INCLUDES})

# Main inference pipeline (T1 → T2 → T3 → T4)
add_executable(pipeline
    main.cpp
    camera_capture.cpp
)
target_link_libraries(pipeline PRIVATE ${OpenCV_LIBS} Threads::Threads)
target_include_directories(pipeline PRIVATE ${INCLUDES})