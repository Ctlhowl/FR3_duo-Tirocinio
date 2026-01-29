#pragma once
#include <franka/gripper_state.h>
#include <vector>
#include <string>

namespace franka_gripper {
class IGripper {
public:
    virtual ~IGripper() = default;
    virtual bool homing() = 0;
    virtual bool move(double width, double speed) = 0;
    virtual bool grasp(double width, double speed, double force, double epsilon_inner, double epsilon_outer) = 0;
    virtual bool stop() = 0;
    virtual franka::GripperState readOnce() = 0;
};
}