#ifndef DAVE_ROS_GZ_PLUGINS__OCEAN_CURRENT_PLUGIN_HH_
#define DAVE_ROS_GZ_PLUGINS__OCEAN_CURRENT_PLUGIN_HH_

// Gazebo Simulation Related Headers
#include <gz/sim/System.hh>

// ROS 2 Headers
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

// Standard Library Headers
#include <map>
#include <memory>
#include <string>

// Plugin-specific Headers
#include "dave_gz_world_plugins/OceanCurrentWorldPlugin.hh"

// Dave Interfaces: Message Types
#include "dave_interfaces/msg/stratified_current_database.hpp"
#include "dave_interfaces/msg/stratified_current_velocity.hpp"

// Dave Interfaces: Service Types
#include "dave_interfaces/srv/get_current_model.hpp"
#include "dave_interfaces/srv/get_origin_spherical_coord.hpp"
#include "dave_interfaces/srv/set_current_direction.hpp"
#include "dave_interfaces/srv/set_current_model.hpp"
#include "dave_interfaces/srv/set_current_velocity.hpp"
#include "dave_interfaces/srv/set_origin_spherical_coord.hpp"
#include "dave_interfaces/srv/set_stratified_current_direction.hpp"
#include "dave_interfaces/srv/set_stratified_current_velocity.hpp"

// #include "dave_interfaces/srv/Stratified_Current_Database.hpp"
// #include "dave_interfaces/srv/Stratified_Current_Velocity.hpp"

namespace dave_ros_gz_plugins
{
class OceanCurrentPlugin : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPostUpdate
{
public:
  OceanCurrentPlugin();
  ~OceanCurrentPlugin() override;

  // ----------------------------------------------

  void Configure(
    const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr);

  // ----------------------------------------------

  void PostUpdate(const gz::sim::UpdateInfo & _info, const gz::sim::EntityComponentManager & _ecm);

  // ----------------------------------------------

  bool UpdateHorzAngle(
    const std::shared_ptr<dave_interfaces::srv::SetCurrentDirection::Request> _req,
    std::shared_ptr<dave_interfaces::srv::SetCurrentDirection::Response> _res);

  bool UpdateStratHorzAngle(
    const std::shared_ptr<dave_interfaces::srv::SetStratifiedCurrentDirection::Request> _req,
    std::shared_ptr<dave_interfaces::srv::SetStratifiedCurrentDirection::Response> _res);

  bool UpdateVertAngle(
    const std::shared_ptr<dave_interfaces::srv::SetCurrentDirection::Request> _req,
    std::shared_ptr<dave_interfaces::srv::SetCurrentDirection::Response> _res);

  bool UpdateStratVertAngle(
    const std::shared_ptr<dave_interfaces::srv::SetStratifiedCurrentDirection::Request> _req,
    std::shared_ptr<dave_interfaces::srv::SetStratifiedCurrentDirection::Response> _res);

  bool UpdateCurrentVelocity(
    const std::shared_ptr<dave_interfaces::srv::SetCurrentVelocity::Request> _req,
    std::shared_ptr<dave_interfaces::srv::SetCurrentVelocity::Response> _res);

  bool UpdateStratCurrentVelocity(
    const std::shared_ptr<dave_interfaces::srv::SetStratifiedCurrentVelocity::Request> _req,
    std::shared_ptr<dave_interfaces::srv::SetStratifiedCurrentVelocity::Response> _res);

  bool GetCurrentVelocityModel(
    const std::shared_ptr<dave_interfaces::srv::GetCurrentModel::Request> _req,
    std::shared_ptr<dave_interfaces::srv::GetCurrentModel::Response> _res);

  bool GetCurrentHorzAngleModel(
    const std::shared_ptr<dave_interfaces::srv::GetCurrentModel::Request> _req,
    std::shared_ptr<dave_interfaces::srv::GetCurrentModel::Response> _res);

  bool GetCurrentVertAngleModel(
    const std::shared_ptr<dave_interfaces::srv::GetCurrentModel::Request> _req,
    std::shared_ptr<dave_interfaces::srv::GetCurrentModel::Response> _res);

  bool UpdateCurrentVelocityModel(
    const std::shared_ptr<dave_interfaces::srv::SetCurrentModel::Request> _req,
    std::shared_ptr<dave_interfaces::srv::SetCurrentModel::Response> _res);

  bool UpdateCurrentHorzAngleModel(
    const std::shared_ptr<dave_interfaces::srv::SetCurrentModel::Request> _req,
    std::shared_ptr<dave_interfaces::srv::SetCurrentModel::Response> _res);

  bool UpdateCurrentVertAngleModel(
    const std::shared_ptr<dave_interfaces::srv::SetCurrentModel::Request> _req,
    std::shared_ptr<dave_interfaces::srv::SetCurrentModel::Response> _res);

private:
  // std::shared_ptr<rclcpp::Node> rosNode;

  struct PrivateData;
  std::unique_ptr<PrivateData> dataPtr;
};
}  // namespace dave_ros_gz_plugins

#endif  // DAVE_ROS_GZ_PLUGINS__OCEAN_CURRENT_PLUGIN_HH_
