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
    src
)

add_executable(depth_infer module_test/depth_estim_test.cpp)
target_link_libraries(depth_infer PRIVATE ${LIBS})
target_include_directories(depth_infer PRIVATE ${INCLUDES})

add_executable(object_detector module_test/yolo_infer_test.cpp)
target_link_libraries(object_detector PRIVATE ${LIBS})
target_include_directories(object_detector PRIVATE ${INCLUDES})

add_executable(pipeline_visual_test
    module_test/pipeline_visual_test.cpp
    src/camera_capture.cpp
    src/inference_engine.cpp
)
target_link_libraries(pipeline_visual_test PRIVATE ${OpenCV_LIBS} hailort Threads::Threads)
target_include_directories(pipeline_visual_test PRIVATE ${INCLUDES})

# Main inference pipeline (T1 → T2 → T3 → T4)
add_executable(pipeline
    src/main.cpp
    src/camera_capture.cpp
    src/inference_engine.cpp
)
target_link_libraries(pipeline PRIVATE ${OpenCV_LIBS} hailort Threads::Threads)
target_include_directories(pipeline PRIVATE ${INCLUDES})
