#include "MotorsPlugin.hh"

#include "std_msgs/Float32.h"

namespace gazebo {

void MotorsPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    gzdbg << __FUNCTION__ << "() called." << std::endl;

    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    ROS_INFO("Hello World!");

    // Store the pointer to the model
    // Store the pointer to the model
    this->model_ = _parent;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MotorsPlugin::OnUpdate, this));

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (_sdf->HasElement("jointName")) {
        joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
        gzdbg << "jointName:" << joint_name_ << std::endl;
    } else {
        gzerr << "[MotorsPlugin] Please specify a jointName, where the rotor "
                 "is attached.\n";
    }

    joint_ = model_->GetJoint(joint_name_);
    if (joint_ == NULL)
        gzthrow("[MotorsPlugin] Couldn't find specified joint \"" << joint_name_ << "\".");

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (_sdf->HasElement("linkName")) {
        link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
        gzdbg << "linkName:" << link_name_ << std::endl;
    } else
        gzerr << "[MotorsPlugin] Please specify a linkName of the rotor.\n";

    link_ = model_->GetLink(link_name_);
    if (link_ == NULL)
        gzthrow("[MotorsPlugin] Couldn't find specified link \"" << link_name_ << "\".");

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (_sdf->HasElement("motorNumber"))
        motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
    else
        gzerr << "[MotorsPlugin] Please specify a motorNumber.\n";

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (_sdf->HasElement("turningDirection")) {
        std::string turning_direction = _sdf->GetElement("turningDirection")->Get<std::string>();
        if (turning_direction == "cw")
            turning_direction_ = turning_direction::CW;
        else if (turning_direction == "ccw")
            turning_direction_ = turning_direction::CCW;
        else
            gzerr << "[MotorsPlugin] Please only use 'cw' or 'ccw' as "
                     "turningDirection.\n";
    } else
        gzerr << "[MotorsPlugin] Please specify a turning direction ('cw' or "
                 "'ccw').\n";

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (_sdf->HasElement("motorType")) {
        std::string motor_type = _sdf->GetElement("motorType")->Get<std::string>();
        if (motor_type == "velocity")
            motor_type_ = MotorType::kVelocity;
        else if (motor_type == "position")
            motor_type_ = MotorType::kPosition;
        else if (motor_type == "force") {
            motor_type_ = MotorType::kForce;
        } else
            gzerr << "[gazebo_motor_model] Please only use 'velocity', 'position' or "
                     "'force' as motorType.\n";
    } else {
        gzwarn << "[gazebo_motor_model] motorType not specified, using velocity.\n";
        motor_type_ = MotorType::kVelocity;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (motor_type_ == MotorType::kPosition) {
        if (_sdf->HasElement("joint_control_pid")) {
            sdf::ElementPtr pid = _sdf->GetElement("joint_control_pid");
            double p = 0;
            if (pid->HasElement("p"))
                p = pid->Get<double>("p");
            double i = 0;
            if (pid->HasElement("i"))
                i = pid->Get<double>("i");
            double d = 0;
            if (pid->HasElement("d"))
                d = pid->Get<double>("d");
            double iMax = 0;
            if (pid->HasElement("iMax"))
                iMax = pid->Get<double>("iMax");
            double iMin = 0;
            if (pid->HasElement("iMin"))
                iMin = pid->Get<double>("iMin");
            double cmdMax = 0;
            if (pid->HasElement("cmdMax"))
                cmdMax = pid->Get<double>("cmdMax");
            double cmdMin = 0;
            if (pid->HasElement("cmdMin"))
                cmdMin = pid->Get<double>("cmdMin");
            pids_.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
        } else {
            pids_.Init(0, 0, 0, 0, 0, 0, 0);
            gzerr << "[gazebo_motor_model] PID values not found, Setting all values "
                     "to zero!\n";
        }
    } else {
        pids_.Init(0, 0, 0, 0, 0, 0, 0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
    getSdfParam<std::string>(_sdf, "windSpeedSubTopic", wind_speed_sub_topic_, wind_speed_sub_topic_);
    getSdfParam<std::string>(_sdf, "motorSpeedPubTopic", motor_speed_pub_topic_, motor_speed_pub_topic_);

    if (_sdf->HasElement("motorPositionPubTopic")) {
        publish_position_ = true;
        motor_position_pub_topic_ = _sdf->GetElement("motorPositionPubTopic")->Get<std::string>();
    }
    if (_sdf->HasElement("motorForcePubTopic")) {
        publish_force_ = true;
        motor_force_pub_topic_ = _sdf->GetElement("motorForcePubTopic")->Get<std::string>();
    }

    getSdfParam<double>(_sdf, "rotorDragCoefficient", rotor_drag_coefficient_, rotor_drag_coefficient_);
    getSdfParam<double>(_sdf, "rollingMomentCoefficient", rolling_moment_coefficient_, rolling_moment_coefficient_);
    getSdfParam<double>(_sdf, "maxRotVelocity", max_rot_velocity_, max_rot_velocity_);
    getSdfParam<double>(_sdf, "motorConstant", motor_constant_, motor_constant_);
    getSdfParam<double>(_sdf, "momentConstant", moment_constant_, moment_constant_);

    getSdfParam<double>(_sdf, "timeConstantUp", time_constant_up_, time_constant_up_);
    getSdfParam<double>(_sdf, "timeConstantDown", time_constant_down_, time_constant_down_);
    getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // // Create the first order filter.
    rotor_velocity_filter_.reset(
        new FirstOrderFilter<double>(time_constant_up_, time_constant_down_, ref_motor_input_));
}

void MotorsPlugin::OnUpdate()
{
    // Apply a small linear velocity to the model.
    this->model_->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));

    if (!pubs_and_subs_created_) {
        CreatePubsAndSubs();
        pubs_and_subs_created_ = true;
    }
}

void MotorsPlugin::CreatePubsAndSubs()
{
    // gzdbg << __PRETTY_FUNCTION__ << " called." << std::endl;

    // // Create temporary "ConnectGazeboToRosTopic" publisher and message
    // gazebo::transport::PublisherPtr gz_connect_gazebo_to_ros_topic_pub
    //     = node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>("~/" + kConnectGazeboToRosSubtopic, 1);
    // gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

    // // Create temporary "ConnectRosToGazeboTopic" publisher and message
    // gazebo::transport::PublisherPtr gz_connect_ros_to_gazebo_topic_pub
    //     = node_handle_->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>("~/" + kConnectRosToGazeboSubtopic, 1);
    // gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;

    // // ============================================ //
    // //  ACTUAL MOTOR SPEED MSG SETUP (GAZEBO->ROS)  //
    // // ============================================ //
    // if (publish_speed_) {
    //     motor_velocity_pub_
    //         = node_handle_->Advertise<gz_std_msgs::Float32>("~/" + namespace_ + "/" + motor_speed_pub_topic_, 1);

    //     connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" + motor_speed_pub_topic_);
    //     connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" + motor_speed_pub_topic_);
    //     connect_gazebo_to_ros_topic_msg.set_msgtype(gz_std_msgs::ConnectGazeboToRosTopic::FLOAT_32);
    //     gz_connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg, true);
    // }

    // // =============================================== //
    // //  ACTUAL MOTOR POSITION MSG SETUP (GAZEBO->ROS)  //
    // // =============================================== //

    // if (publish_position_) {
    //     motor_position_pub_
    //         = node_handle_->Advertise<gz_std_msgs::Float32>("~/" + namespace_ + "/" + motor_position_pub_topic_, 1);

    //     connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" + motor_position_pub_topic_);
    //     connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" + motor_position_pub_topic_);
    //     connect_gazebo_to_ros_topic_msg.set_msgtype(gz_std_msgs::ConnectGazeboToRosTopic::FLOAT_32);
    //     gz_connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg, true);
    // }

    // // ============================================ //
    // //  ACTUAL MOTOR FORCE MSG SETUP (GAZEBO->ROS)  //
    // // ============================================ //

    // if (publish_force_) {
    //     motor_force_pub_
    //         = node_handle_->Advertise<gz_std_msgs::Float32>("~/" + namespace_ + "/" + motor_force_pub_topic_, 1);

    //     connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" + motor_force_pub_topic_);
    //     connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" + motor_force_pub_topic_);
    //     connect_gazebo_to_ros_topic_msg.set_msgtype(gz_std_msgs::ConnectGazeboToRosTopic::FLOAT_32);
    //     gz_connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg, true);
    // }

    // ============================================ //
    // = CONTROL COMMAND MSG SETUP (ROS->GAZEBO)  = //
    // ============================================ //

    // command_sub_ = node_handle_->Subscribe(
    //     "~/" + namespace_ + "/" + command_sub_topic_, &GazeboMotorModel::ControlCommandCallback, this);

    // connect_ros_to_gazebo_topic_msg.set_ros_topic(namespace_ + "/" + command_sub_topic_);
    // connect_ros_to_gazebo_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" + command_sub_topic_);
    // connect_ros_to_gazebo_topic_msg.set_msgtype(gz_std_msgs::ConnectRosToGazeboTopic::COMMAND_MOTOR_SPEED);
    // gz_connect_ros_to_gazebo_topic_pub->Publish(connect_ros_to_gazebo_topic_msg, true);
}

GZ_REGISTER_MODEL_PLUGIN(MotorsPlugin);
}