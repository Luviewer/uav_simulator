#pragma once

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include <ros/console.h>

#include <gazebo/msgs/msgs.hh>

#include <mav_msgs/Actuators.h>

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

namespace gazebo {

class ArduPilotTcpSocketPrivate;
class ArduUWBPluginPrivate;

class GAZEBO_VISIBLE ArduUWBPlugin : public ModelPlugin {
    /// \brief Constructor.
public:
    ArduUWBPlugin();

    /// \brief Destructor.
public:
    ~ArduUWBPlugin();

private:
    std::unique_ptr<ArduUWBPluginPrivate> dataPtr;

private:
    ignition::math::Pose3d modelXYZToAirplaneXForwardZDown;

    /// \brief transform from world frame to NED frame
private:
    ignition::math::Pose3d gazeboXYZToNED;

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
public:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Init ardupilot socket
private:
    bool InitArduUWBSockets() const;

private:
    void SetState() const;

private:
    void setProtocal(int16_t* Dist, int16_t* Rtls);

private:
    void OnUpdate();
};
}