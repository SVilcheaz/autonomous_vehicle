#include "pid_controller.hpp"

PIDController::PIDController(const PipelineConfig& cfg, float kp, float ki, float kd)
        : kp_(kp), ki_(ki), kd_(kd)
        {
                camera_width_ = cfg.camera_width;
        }

PIDController::~PIDController() {}

<<<<<<< HEAD
DriveCommand PIDController::compute_follow(const PerceptionResult& result){
        for(auto& i : result.detections){
               if(i.label == "person"){
                break;
               }
        }
        return {0.0, 0.0};
=======
float PIDController::compute_control(const PerceptionResult& result){
        // for(auto& i : result.detections){
        //        if(i.label == "person"){
        //         break;
        //        }
        // }
        return 0.0f;
>>>>>>> ae5bf17 (add pid controller)
}