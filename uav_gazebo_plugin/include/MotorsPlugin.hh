#pragma once

#include <stdio.h>

#include <boost/bind.hpp>
#include <functional>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// #include "Float32.pb.h"

#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"

#include <ignition/math/Vector3.hh>

#include "common.h"

namespace turning_direction {
const static int CCW = 1;
const static int CW = -1;
} // namespace turning_direction

enum class MotorType { kVelocity, kPosition, kForce };

namespace gazebo {

class MotorsPlugin : public ModelPlugin {

public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

public:
    void OnUpdate();

private:
    void CreatePubsAndSubs();

private:
    event::ConnectionPtr updateConnection;

    bool pubs_and_subs_created_;

    std::string joint_name_;
    std::string link_name_;

    physics::ModelPtr model_;
    physics::JointPtr joint_;
    physics::LinkPtr link_;

    int motor_number_;
    int turning_direction_;
    MotorType motor_type_;

    common::PID pids_;

    bool publish_speed_;
    bool publish_position_;
    bool publish_force_;

    std::string command_sub_topic_;
    std::string wind_speed_sub_topic_;
    std::string motor_speed_pub_topic_;
    std::string motor_position_pub_topic_;
    std::string motor_force_pub_topic_;

    double max_force_;
    double max_rot_velocity_;
    double moment_constant_;
    double motor_constant_;
    double ref_motor_input_;
    double rolling_moment_coefficient_;
    double rotor_drag_coefficient_;
    double rotor_velocity_slowdown_sim_;
    double time_constant_down_;
    double time_constant_up_;

    std::unique_ptr<FirstOrderFilter<double>> rotor_velocity_filter_;
    ignition::math::Vector3d wind_speed_W_;
};

}