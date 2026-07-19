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

add_executable(camera_test module_test/camera_capture_test.cpp src/camera_capture.cpp)
target_link_libraries(camera_test PRIVATE ${OpenCV_LIBS} Threads::Threads)
target_include_directories(camera_test PRIVATE ${INCLUDES})

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

add_executable(servo_test module_test/servo_test.cpp src/servo_controller.cpp)
target_link_libraries(servo_test PRIVATE lgpio Threads::Threads)
target_include_directories(servo_test PRIVATE ${INCLUDES})

add_executable(motor_test module_test/motor_driver_test.cpp)
target_link_libraries(motor_test PRIVATE lgpio)
target_include_directories(motor_test PRIVATE ${INCLUDES})

# Main inference pipeline (T1 → T2 → T3 → T4)
add_executable(pipeline
    src/main.cpp
    src/camera_capture.cpp
    src/inference_engine.cpp
    src/decision_engine.cpp
    src/pid_controller.cpp
    src/vfh_controller.cpp
    src/motor_controller.cpp
    src/servo_controller.cpp
    src/rc_receiver.cpp
)
target_link_libraries(pipeline PRIVATE ${OpenCV_LIBS} hailort lgpio Threads::Threads)
target_include_directories(pipeline PRIVATE ${INCLUDES})

add_executable(rc_receiver_test module_test/rc_receiver_test.cpp src/rc_receiver.cpp)
target_link_libraries(rc_receiver_test PRIVATE Threads::Threads)
target_include_directories(rc_receiver_test PRIVATE ${INCLUDES})

add_executable(vfh_test module_test/vfh_controller_test.cpp src/vfh_controller.cpp)
target_link_libraries(vfh_test PRIVATE ${OpenCV_LIBS})
target_include_directories(vfh_test PRIVATE ${INCLUDES})

add_executable(follow_test module_test/follow_controller_test.cpp src/pid_controller.cpp)
target_link_libraries(follow_test PRIVATE ${OpenCV_LIBS} Threads::Threads)
target_include_directories(follow_test PRIVATE ${INCLUDES})
