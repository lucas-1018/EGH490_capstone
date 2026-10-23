// Standard Library Headers
#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// Boost Library Header
#include <boost/shared_ptr.hpp>

// ROS 2 Headers
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <std_msgs/msg/string.hpp>

// Gazebo Physics and Simulation Headers
#include <gz/physics/World.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/World.hh>

// Dave Gazebo World Plugins
#include "dave_gz_world_plugins/OceanCurrentWorldPlugin.hh"
#include "dave_gz_world_plugins/gauss_markov_process.hh"

// Dave ROS-Gazebo Plugin
#include "dave_ros_gz_plugins/OceanCurrentPlugin.hh"

GZ_ADD_PLUGIN(
  dave_ros_gz_plugins::OceanCurrentPlugin, gz::sim::System,
  dave_ros_gz_plugins::OceanCurrentPlugin::ISystemConfigure,
  dave_ros_gz_plugins::OceanCurrentPlugin::ISystemPostUpdate)

namespace dave_ros_gz_plugins
{
struct OceanCurrentPlugin::PrivateData
{
  // Time management
  std::chrono::steady_clock::duration lastUpdate{0};

  // ROS 2 Services for Current Models
  rclcpp::Service<dave_interfaces::srv::GetCurrentModel>::SharedPtr get_current_velocity_model;
  rclcpp::Service<dave_interfaces::srv::GetCurrentModel>::SharedPtr get_current_horz_angle_model;
  rclcpp::Service<dave_interfaces::srv::GetCurrentModel>::SharedPtr get_current_vert_angle_model;
  rclcpp::Service<dave_interfaces::srv::SetCurrentModel>::SharedPtr set_current_velocity_model;
  rclcpp::Service<dave_interfaces::srv::SetCurrentModel>::SharedPtr set_current_horz_angle_model;
  rclcpp::Service<dave_interfaces::srv::SetCurrentModel>::SharedPtr set_current_vert_angle_model;

  // ROS 2 Services for Current Velocity and Directions
  rclcpp::Service<dave_interfaces::srv::SetCurrentVelocity>::SharedPtr set_current_velocity;
  rclcpp::Service<dave_interfaces::srv::SetCurrentDirection>::SharedPtr set_current_horz_angle;
  rclcpp::Service<dave_interfaces::srv::SetCurrentDirection>::SharedPtr set_current_vert_angle;

  // ROS 2 Services for Stratified Current Models
  rclcpp::Service<dave_interfaces::srv::SetStratifiedCurrentVelocity>::SharedPtr
    set_stratified_current_velocity;
  rclcpp::Service<dave_interfaces::srv::SetStratifiedCurrentDirection>::SharedPtr
    set_stratified_current_horz_angle;
  rclcpp::Service<dave_interfaces::srv::SetStratifiedCurrentDirection>::SharedPtr
    set_stratified_current_vert_angle;

  // ROS Node
  std::shared_ptr<rclcpp::Node> rosNode;

  // Gazebo Simulation Objects
  gz::sim::World world;
  gz::sim::Model model;
  gz::sim::Entity entity;  // Added entity member

  // Model and SDF Parameters
  std::string modelName;
  std::shared_ptr<const sdf::Element> sdf;

  // Topic Names for Publishing Data
  // std::string stratifiedCurrentVelocityTopic;
  std::string stratifiedCurrentVelocityDatabaseTopic;
  // std::string currentVelocityTopic;
  std::string model_namespace;

  // ROS 2 Publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr flowVelocityPub;
  rclcpp::Publisher<dave_interfaces::msg::StratifiedCurrentVelocity>::SharedPtr
    stratifiedCurrentVelocityPub;
  rclcpp::Publisher<dave_interfaces::msg::StratifiedCurrentDatabase>::SharedPtr
    stratifiedCurrentDatabasePub;
};

/////////////////////////////////////////////////
OceanCurrentPlugin::OceanCurrentPlugin() : dataPtr(std::make_unique<PrivateData>()) {}
OceanCurrentPlugin::~OceanCurrentPlugin() = default;

/////////////////////////////////////////////////
void OceanCurrentPlugin::Configure(
  const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr)
{
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
    // gzerr << "ROS 2 has not been properly initialized. Please make sure you have initialized your
    // ROS 2 environment.";
  }

  // Initialize the ROS 2 node
  this->dataPtr->rosNode = std::make_shared<rclcpp::Node>("underwater_current_ros_plugin");

  // Retrieve the Gazebo world entity
  auto worldEntity = _ecm.EntityByComponents(gz::sim::components::World());
  this->dataPtr->world = gz::sim::World(worldEntity);

  // Set model entity
  this->dataPtr->entity = _entity;
  this->dataPtr->model = gz::sim::Model(_entity);
  this->dataPtr->modelName = this->dataPtr->model.Name(_ecm);

  // Save the SDF pointer
  this->dataPtr->sdf = _sdf;

  auto * worldPlugin = dave_gz_world_plugins::OceanCurrentWorldPlugin::Instance();
  if (!worldPlugin)
  {
    gzerr
      << "The world plugin hasn't been configured or isn't loaded so we can't publish real data yet"
      << std::endl;
    return;
  }

  auto sharedPtr = worldPlugin->sharedDataPtr;
  if (!sharedPtr)
  {
    gzerr
      << "The world plugin hasn't been configured or isn't loaded so we can't publish real data yet"
      << std::endl;
    return;
  }

  // Access the "real" currentHorzAngleModel
  auto & horzModel = worldPlugin->sharedDataPtr->currentHorzAngleModel;
  // Set the topic for the stratified current velocity database
  this->dataPtr->stratifiedCurrentVelocityDatabaseTopic =
    "stratified_current_velocity_topic_database";

  if (_sdf->HasElement("namespace"))
  {
    this->dataPtr->model_namespace = _sdf->Get<std::string>("namespace");
  }
  else
  {
    this->dataPtr->model_namespace = "hydrodynamics";
  }

  // Reinitialize the ROS 2 node with the model namespace // TODO: Do we need this confirm with
  // woen-sug : )

  this->dataPtr->rosNode =
    std::make_shared<rclcpp::Node>("underwater_current_ros_plugin", this->dataPtr->model_namespace);

  if (sharedPtr->use_constant_current)
  {  // Create and advertise Messages
    // Advertise the flow velocity as a stamped twist message
    this->dataPtr->flowVelocityPub =
      this->dataPtr->rosNode->create_publisher<geometry_msgs::msg::TwistStamped>(
        "currentVelocityTopic", 1);
  }

  // Advertise the stratified ocean current message
  this->dataPtr->stratifiedCurrentVelocityPub =
    this->dataPtr->rosNode->create_publisher<dave_interfaces::msg::StratifiedCurrentVelocity>(
      "stratifiedCurrentVelocityTopic", 1);

  // Advertise the stratified ocean current database message
  this->dataPtr->stratifiedCurrentDatabasePub =
    this->dataPtr->rosNode->create_publisher<dave_interfaces::msg::StratifiedCurrentDatabase>(
      this->dataPtr->stratifiedCurrentVelocityDatabaseTopic, 1);

  // Advertise the service to get the current velocity model
  this->dataPtr->get_current_velocity_model =
    this->dataPtr->rosNode->create_service<dave_interfaces::srv::GetCurrentModel>(
      "get_current_velocity_model", std::bind(
                                      &OceanCurrentPlugin::GetCurrentVelocityModel, this,
                                      std::placeholders::_1, std::placeholders::_2));

  // Advertise the service to get the current horizontal angle model
  this->dataPtr->get_current_horz_angle_model =
    this->dataPtr->rosNode->create_service<dave_interfaces::srv::GetCurrentModel>(
      "get_current_horz_angle_model", std::bind(
                                        &OceanCurrentPlugin::GetCurrentHorzAngleModel, this,
                                        std::placeholders::_1, std::placeholders::_2));

  // Advertise the service to get the current vertical angle model
  this->dataPtr->get_current_vert_angle_model =
    this->dataPtr->rosNode->create_service<dave_interfaces::srv::GetCurrentModel>(
      "get_current_vert_angle_model", std::bind(
                                        &OceanCurrentPlugin::GetCurrentVertAngleModel, this,
                                        std::placeholders::_1, std::placeholders::_2));

  // Advertise the service to update the current velocity model
  this->dataPtr->set_current_velocity_model =
    this->dataPtr->rosNode->create_service<dave_interfaces::srv::SetCurrentModel>(
      "set_current_velocity_model", std::bind(
                                      &OceanCurrentPlugin::UpdateCurrentVelocityModel, this,
                                      std::placeholders::_1, std::placeholders::_2));

  // Advertise the service to update the current horizontal angle model
  this->dataPtr->set_current_horz_angle_model =
    this->dataPtr->rosNode->create_service<dave_interfaces::srv::SetCurrentModel>(
      "set_current_horz_angle_model", std::bind(
                                        &OceanCurrentPlugin::UpdateCurrentHorzAngleModel, this,
                                        std::placeholders::_1, std::placeholders::_2));

  // Advertise the service to update the current vertical angle model
  this->dataPtr->set_current_vert_angle_model =
    this->dataPtr->rosNode->create_service<dave_interfaces::srv::SetCurrentModel>(
      "set_current_vert_angle_model", std::bind(
                                        &OceanCurrentPlugin::UpdateCurrentVertAngleModel, this,
                                        std::placeholders::_1, std::placeholders::_2));

  // Advertise the service to update the current velocity mean value
  this->dataPtr->set_current_velocity =
    this->dataPtr->rosNode->create_service<dave_interfaces::srv::SetCurrentVelocity>(
      "set_current_velocity", std::bind(
                                &OceanCurrentPlugin::UpdateCurrentVelocity, this,
                                std::placeholders::_1, std::placeholders::_2));

  // Advertise the service to update the stratified current velocity
  this->dataPtr->set_stratified_current_velocity =
    this->dataPtr->rosNode->create_service<dave_interfaces::srv::SetStratifiedCurrentVelocity>(
      "set_stratified_current_velocity", std::bind(
                                           &OceanCurrentPlugin::UpdateStratCurrentVelocity, this,
                                           std::placeholders::_1, std::placeholders::_2));

  // Advertise the service to update the current horizontal angle
  this->dataPtr->set_current_horz_angle =
    this->dataPtr->rosNode->create_service<dave_interfaces::srv::SetCurrentDirection>(
      "set_current_horz_angle",
      std::bind(
        &OceanCurrentPlugin::UpdateHorzAngle, this, std::placeholders::_1, std::placeholders::_2));

  // Advertise the service to update the current horizontal angle
  this->dataPtr->set_current_vert_angle =
    this->dataPtr->rosNode->create_service<dave_interfaces::srv::SetCurrentDirection>(
      "set_current_vert_angle",
      std::bind(
        &OceanCurrentPlugin::UpdateVertAngle, this, std::placeholders::_1, std::placeholders::_2));

  // Advertise the service to update the stratified current horizontal angle
  this->dataPtr->set_stratified_current_horz_angle =
    this->dataPtr->rosNode->create_service<dave_interfaces::srv::SetStratifiedCurrentDirection>(
      "set_stratified_current_horz_angle", std::bind(
                                             &OceanCurrentPlugin::UpdateStratHorzAngle, this,
                                             std::placeholders::_1, std::placeholders::_2));

  // Advertise the service to update the stratified current vertical angle
  this->dataPtr->set_stratified_current_vert_angle =
    this->dataPtr->rosNode->create_service<dave_interfaces::srv::SetStratifiedCurrentDirection>(
      "set_stratified_current_vert_angle", std::bind(
                                             &OceanCurrentPlugin::UpdateStratVertAngle, this,
                                             std::placeholders::_1, std::placeholders::_2));
}

/////////////////////////////////////////////////
bool OceanCurrentPlugin::UpdateHorzAngle(
  const std::shared_ptr<dave_interfaces::srv::SetCurrentDirection::Request> _req,
  std::shared_ptr<dave_interfaces::srv::SetCurrentDirection::Response> _res)
{
  auto * worldPlugin = dave_gz_world_plugins::OceanCurrentWorldPlugin::Instance();
  if (!worldPlugin || !worldPlugin->sharedDataPtr)
  {
    _res->success = false;
    return true;
  }
  auto & horzModel = worldPlugin->sharedDataPtr->currentHorzAngleModel;
  _res->success = horzModel.SetMean(_req->angle);
  return true;
}

/////////////////////////////////////////////////
bool OceanCurrentPlugin::UpdateStratHorzAngle(
  const std::shared_ptr<dave_interfaces::srv::SetStratifiedCurrentDirection::Request> _req,
  std::shared_ptr<dave_interfaces::srv::SetStratifiedCurrentDirection::Response> _res)
{
  auto * worldPlugin = dave_gz_world_plugins::OceanCurrentWorldPlugin::Instance();
  if (!worldPlugin || !worldPlugin->sharedDataPtr)
  {
    _res->success = false;
    return true;
  }

  auto & stratifiedDatabase = worldPlugin->sharedDataPtr->stratifiedDatabase;
  auto & stratifiedCurrentModels = worldPlugin->sharedDataPtr->stratifiedCurrentModels;

  if (_req->layer >= stratifiedDatabase.size())
  {
    _res->success = false;
    return true;
  }

  _res->success = stratifiedCurrentModels[_req->layer][1].SetMean(_req->angle);
  if (_res->success)
  {
    // Update the database values (new angle, unchanged velocity)
    double velocity =
      hypot(stratifiedDatabase[_req->layer].X(), stratifiedDatabase[_req->layer].Y());
    stratifiedDatabase[_req->layer].X() = cos(_req->angle) * velocity;
    stratifiedDatabase[_req->layer].Y() = sin(_req->angle) * velocity;
  }
  return true;
}

/////////////////////////////////////////////////
bool OceanCurrentPlugin::UpdateVertAngle(
  const std::shared_ptr<dave_interfaces::srv::SetCurrentDirection::Request> _req,
  std::shared_ptr<dave_interfaces::srv::SetCurrentDirection::Response> _res)
{
  auto * worldPlugin = dave_gz_world_plugins::OceanCurrentWorldPlugin::Instance();
  if (!worldPlugin || !worldPlugin->sharedDataPtr)
  {
    _res->success = false;
    return true;
  }

  auto & currentVertAngleModel = worldPlugin->sharedDataPtr->currentVertAngleModel;
  _res->success = currentVertAngleModel.SetMean(_req->angle);
  return true;
}

/////////////////////////////////////////////////
bool OceanCurrentPlugin::UpdateStratVertAngle(
  const std::shared_ptr<dave_interfaces::srv::SetStratifiedCurrentDirection::Request> _req,
  std::shared_ptr<dave_interfaces::srv::SetStratifiedCurrentDirection::Response> _res)
{
  auto * worldPlugin = dave_gz_world_plugins::OceanCurrentWorldPlugin::Instance();
  if (!worldPlugin || !worldPlugin->sharedDataPtr)
  {
    _res->success = false;
    return true;
  }

  auto & stratifiedDatabase = worldPlugin->sharedDataPtr->stratifiedDatabase;
  auto & stratifiedCurrentModels = worldPlugin->sharedDataPtr->stratifiedCurrentModels;

  if (_req->layer >= stratifiedDatabase.size())
  {
    _res->success = false;
    return true;
  }
  _res->success = stratifiedCurrentModels[_req->layer][2].SetMean(_req->angle);
  return true;
}

/////////////////////////////////////////////////
bool OceanCurrentPlugin::UpdateCurrentVelocity(
  const std::shared_ptr<dave_interfaces::srv::SetCurrentVelocity::Request> _req,
  std::shared_ptr<dave_interfaces::srv::SetCurrentVelocity::Response> _res)
{
  auto * worldPlugin = dave_gz_world_plugins::OceanCurrentWorldPlugin::Instance();
  if (!worldPlugin || !worldPlugin->sharedDataPtr)
  {
    _res->success = false;
    return true;
  }

  // Access the "real" currentHorzAngleModel
  auto & currentVelModel = worldPlugin->sharedDataPtr->currentVelModel;
  auto & currentHorzAngleModel = worldPlugin->sharedDataPtr->currentHorzAngleModel;
  auto & currentVertAngleModel = worldPlugin->sharedDataPtr->currentVertAngleModel;

  if (
    currentVelModel.SetMean(_req->velocity) &&
    currentHorzAngleModel.SetMean(_req->horizontal_angle) &&
    currentVertAngleModel.SetMean(_req->vertical_angle))
  {
    gzmsg << "Current velocity [m/s] = " << _req->velocity << std::endl
          << "Current horizontal angle [rad] = " << _req->horizontal_angle << std::endl
          << "Current vertical angle [rad] = " << _req->vertical_angle << std::endl
          << "\tWARNING: Current velocity calculated in the ENU frame" << std::endl;
    _res->success = true;
  }
  else
  {
    gzmsg << "Error while updating the current velocity" << std::endl;
    _res->success = false;
  }
  return true;
}

/////////////////////////////////////////////////
bool OceanCurrentPlugin::UpdateStratCurrentVelocity(
  const std::shared_ptr<dave_interfaces::srv::SetStratifiedCurrentVelocity::Request> _req,
  std::shared_ptr<dave_interfaces::srv::SetStratifiedCurrentVelocity::Response> _res)
{
  auto * worldPlugin = dave_gz_world_plugins::OceanCurrentWorldPlugin::Instance();
  if (!worldPlugin || !worldPlugin->sharedDataPtr)
  {
    _res->success = false;
    return true;
  }

  auto & stratifiedCurrentModels = worldPlugin->sharedDataPtr->stratifiedCurrentModels;
  auto & stratifiedDatabase = worldPlugin->sharedDataPtr->stratifiedDatabase;

  if (_req->layer >= stratifiedDatabase.size())
  {
    _res->success = false;
    return true;
  }
  if (
    stratifiedCurrentModels[_req->layer][0].SetMean(_req->velocity) &&
    stratifiedCurrentModels[_req->layer][1].SetMean(_req->horizontal_angle) &&
    stratifiedCurrentModels[_req->layer][2].SetMean(_req->vertical_angle))
  {
    // Update the database values as well
    stratifiedDatabase[_req->layer].X() = cos(_req->horizontal_angle) * _req->velocity;
    stratifiedDatabase[_req->layer].Y() = sin(_req->horizontal_angle) * _req->velocity;
    gzmsg << "Layer " << _req->layer << " current velocity [m/s] = " << _req->velocity << std::endl
          << "  Horizontal angle [rad] = " << _req->horizontal_angle << std::endl
          << "  Vertical angle [rad] = " << _req->vertical_angle << std::endl
          << "\tWARNING: Current velocity calculated in the ENU frame" << std::endl;
    _res->success = true;
  }
  else
  {
    gzmsg << "Error while updating the current velocity" << std::endl;
    _res->success = false;
  }
  return true;
}

/////////////////////////////////////////////////
bool OceanCurrentPlugin::GetCurrentVelocityModel(
  const std::shared_ptr<dave_interfaces::srv::GetCurrentModel::Request> _req,
  std::shared_ptr<dave_interfaces::srv::GetCurrentModel::Response> _res)
{
  auto * worldPlugin = dave_gz_world_plugins::OceanCurrentWorldPlugin::Instance();
  if (!worldPlugin || !worldPlugin->sharedDataPtr)
  {
    // _res->success = false;
    return true;
  }

  auto & currentVelModel = worldPlugin->sharedDataPtr->currentVelModel;

  _res->mean = currentVelModel.mean;
  _res->min = currentVelModel.min;
  _res->max = currentVelModel.max;
  _res->noise = currentVelModel.noiseAmp;
  _res->mu = currentVelModel.mu;
  return true;
}

/////////////////////////////////////////////////
bool OceanCurrentPlugin::GetCurrentHorzAngleModel(
  const std::shared_ptr<dave_interfaces::srv::GetCurrentModel::Request> _req,
  std::shared_ptr<dave_interfaces::srv::GetCurrentModel::Response> _res)
{
  auto * worldPlugin = dave_gz_world_plugins::OceanCurrentWorldPlugin::Instance();
  if (!worldPlugin || !worldPlugin->sharedDataPtr)
  {
    // _res->success = false;
    return true;
  }

  auto & currentHorzAngleModel = worldPlugin->sharedDataPtr->currentHorzAngleModel;

  _res->mean = currentHorzAngleModel.mean;
  _res->min = currentHorzAngleModel.min;
  _res->max = currentHorzAngleModel.max;
  _res->noise = currentHorzAngleModel.noiseAmp;
  _res->mu = currentHorzAngleModel.mu;
  return true;
}

/////////////////////////////////////////////////
bool OceanCurrentPlugin::GetCurrentVertAngleModel(
  const std::shared_ptr<dave_interfaces::srv::GetCurrentModel::Request> _req,
  std::shared_ptr<dave_interfaces::srv::GetCurrentModel::Response> _res)
{
  auto * worldPlugin = dave_gz_world_plugins::OceanCurrentWorldPlugin::Instance();
  if (!worldPlugin || !worldPlugin->sharedDataPtr)
  {
    // _res->success = false;
    return true;
  }

  auto & currentVertAngleModel = worldPlugin->sharedDataPtr->currentVertAngleModel;

  _res->mean = currentVertAngleModel.mean;
  _res->min = currentVertAngleModel.min;
  _res->max = currentVertAngleModel.max;
  _res->noise = currentVertAngleModel.noiseAmp;
  _res->mu = currentVertAngleModel.mu;
  return true;
}

/////////////////////////////////////////////////
bool OceanCurrentPlugin::UpdateCurrentVelocityModel(
  const std::shared_ptr<dave_interfaces::srv::SetCurrentModel::Request> _req,
  std::shared_ptr<dave_interfaces::srv::SetCurrentModel::Response> _res)
{
  auto * worldPlugin = dave_gz_world_plugins::OceanCurrentWorldPlugin::Instance();
  if (!worldPlugin || !worldPlugin->sharedDataPtr)
  {
    _res->success = false;
    return true;
  }

  auto & currentVelModel = worldPlugin->sharedDataPtr->currentVelModel;
  auto & stratifiedCurrentModels = worldPlugin->sharedDataPtr->stratifiedCurrentModels;

  _res->success = currentVelModel.SetModel(
    std::max(0.0, _req->mean), std::min(0.0, _req->min), std::max(0.0, _req->max), _req->mu,
    _req->noise);

  for (int i = 0; i < stratifiedCurrentModels.size(); i++)
  {
    dave_gz_world_plugins::GaussMarkovProcess & model = stratifiedCurrentModels[i][0];  //(updated)
    model.SetModel(
      model.mean, std::max(0.0, _req->min), std::max(0.0, _req->max), _req->mu, _req->noise);
  }

  gzmsg << "Current velocity model updated" << std::endl
        << "\tWARNING: Current velocity calculated in the ENU frame" << std::endl;
  currentVelModel.Print();

  return true;
}
/////////////////////////////////////////////////
bool OceanCurrentPlugin::UpdateCurrentHorzAngleModel(
  const std::shared_ptr<dave_interfaces::srv::SetCurrentModel::Request> _req,
  std::shared_ptr<dave_interfaces::srv::SetCurrentModel::Response> _res)
{
  auto * worldPlugin = dave_gz_world_plugins::OceanCurrentWorldPlugin::Instance();
  if (!worldPlugin || !worldPlugin->sharedDataPtr)
  {
    _res->success = false;
    return true;
  }

  auto & horzModel = worldPlugin->sharedDataPtr->currentHorzAngleModel;
  auto & stratifiedCurrentModels = worldPlugin->sharedDataPtr->stratifiedCurrentModels;

  _res->success = horzModel.SetModel(_req->mean, _req->min, _req->max, _req->mu, _req->noise);

  for (int i = 0; i < stratifiedCurrentModels.size(); i++)
  {
    dave_gz_world_plugins::GaussMarkovProcess & model = stratifiedCurrentModels[i][1];
    model.SetModel(
      model.mean, std::max(-M_PI, _req->min), std::min(M_PI, _req->max), _req->mu, _req->noise);
  }

  gzmsg << "Horizontal angle model updated" << std::endl
        << "\tWARNING: Current velocity calculated in the ENU frame" << std::endl;
  horzModel.Print();
  return true;
}

/////////////////////////////////////////////////
bool OceanCurrentPlugin::UpdateCurrentVertAngleModel(
  const std::shared_ptr<dave_interfaces::srv::SetCurrentModel::Request> _req,
  std::shared_ptr<dave_interfaces::srv::SetCurrentModel::Response> _res)
{
  auto * worldPlugin = dave_gz_world_plugins::OceanCurrentWorldPlugin::Instance();
  if (!worldPlugin || !worldPlugin->sharedDataPtr)
  {
    _res->success = false;
    return true;
  }

  auto & currentVertAngleModel = worldPlugin->sharedDataPtr->currentVertAngleModel;

  _res->success =
    currentVertAngleModel.SetModel(_req->mean, _req->min, _req->max, _req->mu, _req->noise);

  gzmsg << "Vertical angle model updated" << std::endl
        << "\tWARNING: Current velocity calculated in the ENU frame" << std::endl;
  currentVertAngleModel.Print();
  return true;
}

/////////////////////////////////////////////////
void OceanCurrentPlugin::PostUpdate(
  const gz::sim::UpdateInfo & _info, const gz::sim::EntityComponentManager & _ecm)
{
  if (!_info.paused)
  {
    rclcpp::spin_some(this->dataPtr->rosNode);
    if (_info.iterations % 1000 == 0)
    {
      gzmsg << "dave_ros_gz_plugins::OceanCurrentPlugin::PostUpdate" << std::endl;
    }
  }
  auto * worldPlugin = dave_gz_world_plugins::OceanCurrentWorldPlugin::Instance();
  if (!worldPlugin)
  {
    gzerr
      << "The world plugin hasn't been configured or isn't loaded so we can't publish real data yet"
      << std::endl;
    return;
  }

  auto sharedPtr = worldPlugin->sharedDataPtr;
  if (!sharedPtr)
  {
    return;
  }

  // Publish constant current velocity
  if (sharedPtr->use_constant_current)
  {
    auto flowVelMsg = geometry_msgs::msg::TwistStamped();

    flowVelMsg.header.stamp = this->dataPtr->rosNode->get_clock()->now();
    flowVelMsg.header.frame_id = "world";

    flowVelMsg.twist.linear.x = sharedPtr->currentVelocity.X();
    flowVelMsg.twist.linear.y = sharedPtr->currentVelocity.Y();
    flowVelMsg.twist.linear.z = sharedPtr->currentVelocity.Z();

    this->dataPtr->flowVelocityPub->publish(flowVelMsg);
  }

  // Generate and publish stratified current velocity
  auto stratCurrentVelocityMsg = dave_interfaces::msg::StratifiedCurrentVelocity();

  stratCurrentVelocityMsg.header.stamp = this->dataPtr->rosNode->get_clock()->now();
  stratCurrentVelocityMsg.header.frame_id = "world";

  // Updating for stratified behaviour of Ocean Currents
  for (size_t i = 0; i < sharedPtr->currentStratifiedVelocity.size(); i++)
  {
    geometry_msgs::msg::Vector3 velocity;
    velocity.x = sharedPtr->currentStratifiedVelocity[i].X();
    velocity.y = sharedPtr->currentStratifiedVelocity[i].Y();
    velocity.z = sharedPtr->currentStratifiedVelocity[i].Z();
    stratCurrentVelocityMsg.velocities.push_back(velocity);
    stratCurrentVelocityMsg.depths.push_back(sharedPtr->currentStratifiedVelocity[i].W());
  }

  this->dataPtr->stratifiedCurrentVelocityPub->publish(stratCurrentVelocityMsg);

  // Generate and publish stratified current database
  auto currentDatabaseMsg = dave_interfaces::msg::StratifiedCurrentDatabase();
  for (int i = 0; i < sharedPtr->stratifiedDatabase.size(); i++)
  {
    // Stratified current database entry preparation
    geometry_msgs::msg::Vector3 velocity;
    velocity.x = sharedPtr->stratifiedDatabase[i].X();
    velocity.y = sharedPtr->stratifiedDatabase[i].Y();
    velocity.z = 0.0;  // Assuming z is intentionally set to 0.0
    currentDatabaseMsg.velocities.push_back(velocity);
    currentDatabaseMsg.depths.push_back(sharedPtr->stratifiedDatabase[i].Z());
  }

  if (sharedPtr->tidalHarmonicFlag)
  {
    // Tidal harmonic constituents
    currentDatabaseMsg.m2_amp = sharedPtr->M2_amp;
    currentDatabaseMsg.m2_phase = sharedPtr->M2_phase;
    currentDatabaseMsg.m2_speed = sharedPtr->M2_speed;
    currentDatabaseMsg.s2_amp = sharedPtr->S2_amp;
    currentDatabaseMsg.s2_phase = sharedPtr->S2_phase;
    currentDatabaseMsg.s2_speed = sharedPtr->S2_speed;
    currentDatabaseMsg.n2_amp = sharedPtr->N2_amp;
    currentDatabaseMsg.n2_phase = sharedPtr->N2_phase;
    currentDatabaseMsg.n2_speed = sharedPtr->N2_speed;
    currentDatabaseMsg.tideconstituents = true;
  }
  else
  {
    for (int i = 0; i < sharedPtr->dateGMT.size(); i++)
    {
      // Tidal oscillation database
      currentDatabaseMsg.time_gmt_year.push_back(sharedPtr->dateGMT[i][0]);
      currentDatabaseMsg.time_gmt_month.push_back(sharedPtr->dateGMT[i][1]);
      currentDatabaseMsg.time_gmt_day.push_back(sharedPtr->dateGMT[i][2]);
      currentDatabaseMsg.time_gmt_hour.push_back(sharedPtr->dateGMT[i][3]);
      currentDatabaseMsg.time_gmt_minute.push_back(sharedPtr->dateGMT[i][4]);

      currentDatabaseMsg.tidevelocities.push_back(sharedPtr->speedcmsec[i]);
    }
    currentDatabaseMsg.tideconstituents = false;
  }

  currentDatabaseMsg.ebb_direction = sharedPtr->ebbDirection;
  currentDatabaseMsg.flood_direction = sharedPtr->floodDirection;

  currentDatabaseMsg.world_start_time_year = sharedPtr->world_start_time_year;
  currentDatabaseMsg.world_start_time_month = sharedPtr->world_start_time_month;
  currentDatabaseMsg.world_start_time_day = sharedPtr->world_start_time_day;
  currentDatabaseMsg.world_start_time_hour = sharedPtr->world_start_time_hour;
  currentDatabaseMsg.world_start_time_minute = sharedPtr->world_start_time_minute;

  this->dataPtr->stratifiedCurrentDatabasePub->publish(currentDatabaseMsg);

  // Update the time tracking for publication
  this->dataPtr->lastUpdate = _info.simTime;
}

}  // namespace dave_ros_gz_plugins

// #endif