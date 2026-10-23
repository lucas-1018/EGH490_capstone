// Include the necessary headers for Ocean Current Plugin
#include "dave_gz_world_plugins/OceanCurrentWorldPlugin.hh"

// Include messages for Stratified Current Velocity
#include <dave_gz_world_plugins_msgs/msgs/StratifiedCurrentVelocity.pb.h>

// Include Gazebo and Ignition libraries
#include <gz/msgs/vector3d.pb.h>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Vector4.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>

// Boost libraries for string manipulation, binding, and shared pointers
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <boost/bind/bind.hpp>
#include <boost/shared_ptr.hpp>

// Other necessary libraries
#include <math.h>
#include <chrono>
#include <map>
#include <memory>
#include <string>

// AMENT library for package directory functions
#include <ament_index_cpp/get_package_share_directory.hpp>

// Custom plugin headers
#include "dave_gz_world_plugins/gauss_markov_process.hh"  // For Gauss-Markov process
#include "dave_gz_world_plugins/tidal_oscillation.hh"     // For tidal oscillation modeling

GZ_ADD_PLUGIN(
  dave_gz_world_plugins::OceanCurrentWorldPlugin, gz::sim::System,
  dave_gz_world_plugins::OceanCurrentWorldPlugin::ISystemConfigure,
  dave_gz_world_plugins::OceanCurrentWorldPlugin::ISystemUpdate,
  dave_gz_world_plugins::OceanCurrentWorldPlugin::ISystemPostUpdate)

namespace dave_gz_world_plugins
{
struct OceanCurrentWorldPlugin::PrivateData
{
  // Gazebo Simulation and World Components
  gz::sim::World world{gz::sim::kNullEntity};  // Gazebo world instance
  sdf::ElementPtr sdf;                         // Pointer to the SDF element
  bool hasSurface;                             // Flag to check if surface is present

  // Communication Node for Gazebo
  // std::shared_ptr<gz::transport::Node> gz_node;  // Transport node for communication
  gz::transport::Node gz_node;

  // Publishers
  gz::transport::Node::Publisher gz_node_cvel_world_pub;  // Publisher for current velocity

  // Publishers
  gz::transport::Node::Publisher
    gz_node_scvel_world_pub;  // Publisher for Stratified current velocity

  // Subscribers
  gz::transport::Node subscriber;  // Subscriber for vehicle depth

  // Database File Paths
  std::string databaseFilePath;  // Path to ocean current database (CSV file)
  std::string tidalFilePath;     // Path to tidal oscillation database (TXT file)

  // Namespace for Topics and Services
  std::string ns;  // Namespace for topics and services

  // Tidal Oscillation Variables
  bool tideFlag;
  dave_gz_world_plugins::TidalOscillation tide;

  // Timing
  std::chrono::steady_clock::duration lastUpdate{0};

  // Stratified Current Database Path
  std::string db_path;  // Path to the stratified current database

  // ROS 2 Node
  std::shared_ptr<rclcpp::Node> rosNode;
};

OceanCurrentWorldPlugin * OceanCurrentWorldPlugin::singletonInstance = nullptr;

OceanCurrentWorldPlugin::OceanCurrentWorldPlugin()
: dataPtr(std::make_unique<PrivateData>()), sharedDataPtr(std::shared_ptr<SharedData>())
{
  this->sharedDataPtr = std::make_shared<SharedData>();
}

/////////////////////////////////////////////////

OceanCurrentWorldPlugin::~OceanCurrentWorldPlugin()
{
  // Clear the static pointer if we are the instance
  if (OceanCurrentWorldPlugin::singletonInstance == this)
  {
    OceanCurrentWorldPlugin::singletonInstance = nullptr;
  }
}

OceanCurrentWorldPlugin * OceanCurrentWorldPlugin::Instance()
{
  return OceanCurrentWorldPlugin::singletonInstance;
}

// ----------------------------------------------

void OceanCurrentWorldPlugin::Configure(
  const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr)
{
  this->dataPtr->world = gz::sim::World(_ecm.EntityByComponents(gz::sim::components::World()));
  if (!this->dataPtr->world.Valid(_ecm))  // check
  {
    gzerr << "World entity not found" << std::endl;
    return;
  }
  this->dataPtr->sdf = std::const_pointer_cast<sdf::Element>(_sdf);

  // Read the namespace for topics and services
  this->dataPtr->ns = _sdf->Get<std::string>("namespace");

  gzmsg << "Loading Ocean current world plugin" << std::endl;

  LoadGlobalCurrentConfig();
  LoadStratifiedCurrentDatabase();

  if (_sdf->HasElement("tidal_oscillation"))
  {
    LoadTidalOscillationDatabase();
    gzmsg << "Tidal oscillation is enabled" << std::endl;
  }
  else
  {
    gzmsg << "Tidal oscillation not enabled" << std::endl;
  }

  if (!OceanCurrentWorldPlugin::singletonInstance)
  {
    OceanCurrentWorldPlugin::singletonInstance = this;
  }

  gzmsg << "Underwater current plugin loaded!" << std::endl
        << "\tWARNING: Current velocity calculated in the ENU frame" << std::endl;
}

void OceanCurrentWorldPlugin::LoadTidalOscillationDatabase()
{
  this->dataPtr->tideFlag = true;
  this->sharedDataPtr->tidalHarmonicFlag = false;

  sdf::ElementPtr tidalOscillationParams = this->dataPtr->sdf->GetElement("tidal_oscillation");
  sdf::ElementPtr tidalHarmonicParams;

  // Read the tidal oscillation parameter from the SDF file
  if (tidalOscillationParams->HasElement("databasefilePath"))
  {
    this->dataPtr->tidalFilePath = tidalOscillationParams->Get<std::string>("databasefilePath");
    gzmsg << "Tidal current database configuration found" << std::endl;
  }
  else
  {
    if (tidalOscillationParams->HasElement("harmonic_constituents"))
    {
      tidalHarmonicParams = tidalOscillationParams->GetElement("harmonic_constituents");
      gzmsg << "Tidal harmonic constituents " << "configuration found" << std::endl;
      this->sharedDataPtr->tidalHarmonicFlag = true;
    }
    else
    {
      this->dataPtr->tidalFilePath = ament_index_cpp::get_package_share_directory("dave_worlds") +
                                     "/worlds/ACT1951_1_Annual_2021.csv";
    }
  }

  // Read the tidal oscillation direction from the SDF file
  if (!(tidalOscillationParams->HasElement("mean_direction")))
  {
    gzerr << "Tidal mean direction not defined" << std::endl;
  }

  if (tidalOscillationParams->HasElement("mean_direction"))
  {
    sdf::ElementPtr elem = tidalOscillationParams->GetElement("mean_direction");
    if (elem->HasElement("ebb"))
    {
      this->sharedDataPtr->ebbDirection = elem->Get<double>("ebb");
    }
    else
    {
      gzerr << "Tidal mean ebb direction not defined" << std::endl;
    }

    if (elem->HasElement("flood"))
    {
      this->sharedDataPtr->floodDirection = elem->Get<double>("flood");
    }
    else
    {
      gzerr << "Tidal mean flood direction not defined" << std::endl;
    }
  }

  // Read the world start time (GMT) from the SDF file

  if (tidalOscillationParams->HasElement("world_start_time_GMT"))
  {
    sdf::ElementPtr elem = tidalOscillationParams->GetElement("world_start_time_GMT");

    if (elem->HasElement("day"))
    {
      this->sharedDataPtr->world_start_time_day = elem->Get<double>("day");
    }
    else
    {
      gzerr << "World start time (day) not defined" << std::endl;
    }

    if (elem->HasElement("month"))
    {
      this->sharedDataPtr->world_start_time_month = elem->Get<double>("month");
    }
    else
    {
      gzerr << "World start time (month) not defined" << std::endl;
    }

    if (elem->HasElement("year"))
    {
      this->sharedDataPtr->world_start_time_year = elem->Get<double>("year");
    }
    else
    {
      gzerr << "World start time (year) not defined" << std::endl;
    }

    if (elem->HasElement("hour"))
    {
      this->sharedDataPtr->world_start_time_hour = elem->Get<double>("hour");
    }
    else
    {
      gzerr << "World start time (hour) not defined" << std::endl;
    }

    if (elem->HasElement("minute"))
    {
      this->sharedDataPtr->world_start_time_minute = elem->Get<double>("minute");
    }
    else
    {
      gzerr << "World start time (minute) not defined" << std::endl;
    }
  }
  else
  {
    gzerr << "World start time (GMT) not defined" << std::endl;
  }

  if (this->sharedDataPtr->tidalHarmonicFlag)
  {
    // Read harmonic constituents
    if (tidalHarmonicParams->HasElement("M2"))
    {
      sdf::ElementPtr M2Params = tidalHarmonicParams->GetElement("M2");
      this->sharedDataPtr->M2_amp = M2Params->Get<double>("amp");
      this->sharedDataPtr->M2_phase = M2Params->Get<double>("phase");
      this->sharedDataPtr->M2_speed = M2Params->Get<double>("speed");
    }
    else
    {
      gzerr << "Harcomnic constituents M2 not found" << std::endl;
    }

    if (tidalHarmonicParams->HasElement("S2"))
    {
      sdf::ElementPtr S2Params = tidalHarmonicParams->GetElement("S2");
      this->sharedDataPtr->S2_amp = S2Params->Get<double>("amp");
      this->sharedDataPtr->S2_phase = S2Params->Get<double>("phase");
      this->sharedDataPtr->S2_speed = S2Params->Get<double>("speed");
    }
    else
    {
      gzerr << "Harcomnic constituents S2 not found" << std::endl;
    }

    if (tidalHarmonicParams->HasElement("N2"))
    {
      sdf::ElementPtr N2Params = tidalHarmonicParams->GetElement("N2");
      this->sharedDataPtr->N2_amp = N2Params->Get<double>("amp");
      this->sharedDataPtr->N2_phase = N2Params->Get<double>("phase");
      this->sharedDataPtr->N2_speed = N2Params->Get<double>("speed");
    }
    else
    {
      gzerr << "Harcomnic constituents N2 not found" << std::endl;
    }

    gzmsg << "Tidal harmonic constituents loaded : " << std::endl;
    gzmsg << "M2 amp: " << this->sharedDataPtr->M2_amp
          << " phase: " << this->sharedDataPtr->M2_phase
          << " speed: " << this->sharedDataPtr->M2_speed << std::endl;
    gzmsg << "S2 amp: " << this->sharedDataPtr->S2_amp
          << " phase: " << this->sharedDataPtr->S2_phase
          << " speed: " << this->sharedDataPtr->S2_speed << std::endl;
    gzmsg << "N2 amp: " << this->sharedDataPtr->N2_amp
          << " phase: " << this->sharedDataPtr->N2_phase
          << " speed: " << this->sharedDataPtr->N2_speed << std::endl;
  }
  else
  {
    // Read database
    std::ifstream csvFile;
    std::string line;
    csvFile.open(this->dataPtr->tidalFilePath);
    if (!csvFile)
    {
      gz::common::SystemPaths paths;
      this->dataPtr->tidalFilePath = paths.FindFile(this->dataPtr->tidalFilePath, true);
      csvFile.open(this->dataPtr->tidalFilePath);
    }

    if (!csvFile)
    {
      gzerr << "Tidal Oscillation database file does not exist" << std::endl;
    }

    gzmsg << "Tidal Oscillation  Database loaded : " << this->dataPtr->tidalFilePath << std::endl;

    // skip the first line
    getline(csvFile, line);
    while (getline(csvFile, line))
    {
      if (line.empty())  // skip empty lines:
      {
        continue;
      }
      std::istringstream iss(line);
      std::string lineStream;
      std::string::size_type sz;
      std::vector<std::string> row;
      std::array<int, 5> tmpDateArray;
      while (getline(iss, lineStream, ','))
      {
        row.push_back(lineStream);
      }
      if (strcmp(row[1].c_str(), " slack"))  // skip 'slack' category
      {
        tmpDateArray[0] = std::stoi(row[0].substr(0, 4));
        tmpDateArray[1] = std::stoi(row[0].substr(5, 7));
        tmpDateArray[2] = std::stoi(row[0].substr(8, 10));
        tmpDateArray[3] = std::stoi(row[0].substr(11, 13));
        tmpDateArray[4] = std::stoi(row[0].substr(14, 16));

        this->sharedDataPtr->dateGMT.push_back(tmpDateArray);
        this->sharedDataPtr->speedcmsec.push_back(stold(row[2], &sz));
      }
    }
    csvFile.close();

    // Eliminate data with same consecutive type
    std::vector<int> duplicated;
    for (int i = 0; i < this->sharedDataPtr->dateGMT.size() - 1; i++)
    {
      // delete latter if same sign
      if (
        ((this->sharedDataPtr->speedcmsec[i] > 0) - (this->sharedDataPtr->speedcmsec[i] < 0)) ==
        ((this->sharedDataPtr->speedcmsec[i + 1] > 0) -
         (this->sharedDataPtr->speedcmsec[i + 1] < 0)))
      {
        duplicated.push_back(i + 1);
      }
    }
    int eraseCount = 0;
    for (int i = 0; i < duplicated.size(); i++)
    {
      this->sharedDataPtr->dateGMT.erase(
        this->sharedDataPtr->dateGMT.begin() + duplicated[i] - eraseCount);
      this->sharedDataPtr->speedcmsec.erase(
        this->sharedDataPtr->speedcmsec.begin() + duplicated[i] - eraseCount);
      eraseCount++;
    }
  }
}

/////////////////////////////////////////////////
void OceanCurrentWorldPlugin::LoadStratifiedCurrentDatabase()
{
  sdf::ElementPtr transientCurrentParams;

  if (this->dataPtr->sdf->HasElement("transient_current"))
  {
    transientCurrentParams = this->dataPtr->sdf->GetElement("transient_current");
  }
  else
  {
    gzerr << "Transient current configuration not available" << std::endl;
  }

  if (transientCurrentParams->HasElement("topic_stratified"))
  {
    this->sharedDataPtr->stratifiedCurrentVelocityTopic =
      transientCurrentParams->Get<std::string>("topic_stratified");
  }
  else
  {
    this->sharedDataPtr->stratifiedCurrentVelocityTopic = "stratified_current_velocity";
  }

  if (this->sharedDataPtr->stratifiedCurrentVelocityTopic.empty())
  {
    gzerr << "Empty stratified ocean current velocity topic" << std::endl;
  }

  // Read the depth dependent ocean current file path from the SDF file
  if (transientCurrentParams->HasElement("databasefileName"))
  {
    this->dataPtr->databaseFilePath = ament_index_cpp::get_package_share_directory("dave_worlds") +
                                      "/worlds/" +
                                      transientCurrentParams->Get<std::string>("databasefileName");
  }
  else
  {
    this->dataPtr->databaseFilePath = ament_index_cpp::get_package_share_directory("dave_worlds") +
                                      "/worlds/transientOceanCurrentDatabase.csv";
  }

  if (this->dataPtr->databaseFilePath.empty())
  {
    gzerr << "Empty stratified ocean current database file path" << std::endl;
  }

  gzmsg << this->dataPtr->databaseFilePath << std::endl;

  // Read database
  std::ifstream csvFile;
  std::string line;
  csvFile.open(this->dataPtr->databaseFilePath);
  if (!csvFile)
  {
    gz::common::SystemPaths * paths = new gz::common::SystemPaths();
    this->dataPtr->databaseFilePath = paths->FindFile(this->dataPtr->databaseFilePath, true);
    csvFile.open(this->dataPtr->databaseFilePath);
  }

  if (!csvFile)
  {
    gzerr << "Stratified Ocean database file does not exist" << std::endl;
  }

  gzmsg << "Statified Ocean Current Database loaded : " << this->dataPtr->databaseFilePath
        << std::endl;

  // skip the 3 lines
  getline(csvFile, line);
  getline(csvFile, line);
  getline(csvFile, line);
  while (getline(csvFile, line))
  {
    if (line.empty())  // skip empty lines:
    {
      continue;
    }
    std::istringstream iss(line);
    std::string lineStream;
    std::string::size_type sz;
    std::vector<long double> row;
    while (getline(iss, lineStream, ','))
    {
      row.push_back(stold(lineStream, &sz));  // convert to double
    }
    gz::math::Vector3d read;
    read.X() = row[0];
    read.Y() = row[1];
    read.Z() = row[2];
    this->sharedDataPtr->stratifiedDatabase.push_back(read);

    // Create Gauss-Markov processes for the stratified currents
    // Means are the database-specified magnitudes & angles, and
    // the other values come from the constant current models

    dave_gz_world_plugins::GaussMarkovProcess magnitudeModel;
    magnitudeModel.mean = hypot(row[1], row[0]);
    magnitudeModel.var = magnitudeModel.mean;
    magnitudeModel.max = this->sharedDataPtr->currentVelModel.max;
    magnitudeModel.min = 0.0;
    magnitudeModel.mu = this->sharedDataPtr->currentVelModel.mu;
    magnitudeModel.noiseAmp = this->sharedDataPtr->currentVelModel.noiseAmp;
    magnitudeModel.lastUpdate = std::chrono::duration<double>(this->dataPtr->lastUpdate).count();

    dave_gz_world_plugins::GaussMarkovProcess hAngleModel;
    hAngleModel.mean = atan2(row[1], row[0]);
    hAngleModel.var = hAngleModel.mean;
    hAngleModel.max = M_PI;
    hAngleModel.min = -M_PI;
    hAngleModel.mu = this->sharedDataPtr->currentHorzAngleModel.mu;
    hAngleModel.noiseAmp = this->sharedDataPtr->currentHorzAngleModel.noiseAmp;
    hAngleModel.lastUpdate = std::chrono::duration<double>(this->dataPtr->lastUpdate).count();

    dave_gz_world_plugins::GaussMarkovProcess vAngleModel;
    vAngleModel.mean = 0.0;
    vAngleModel.var = vAngleModel.mean;
    vAngleModel.max = M_PI / 2.0;
    vAngleModel.min = -M_PI / 2.0;
    vAngleModel.mu = this->sharedDataPtr->currentVertAngleModel.mu;
    vAngleModel.noiseAmp = this->sharedDataPtr->currentVertAngleModel.noiseAmp;
    vAngleModel.lastUpdate = std::chrono::duration<double>(this->dataPtr->lastUpdate).count();

    std::vector<dave_gz_world_plugins::GaussMarkovProcess> depthModels;
    depthModels.push_back(magnitudeModel);
    depthModels.push_back(hAngleModel);
    depthModels.push_back(vAngleModel);
    this->sharedDataPtr->stratifiedCurrentModels.push_back(depthModels);
  }
  csvFile.close();

  this->dataPtr->gz_node_scvel_world_pub =
    this->dataPtr->gz_node.Advertise<dave_gz_world_plugins_msgs::msgs::StratifiedCurrentVelocity>(
      this->dataPtr->ns + "/" + this->sharedDataPtr->stratifiedCurrentVelocityTopic);

  gzmsg << "Stratified current velocity topic name: "
        << this->dataPtr->ns + "/" + this->sharedDataPtr->stratifiedCurrentVelocityTopic
        << std::endl;
}

/////////////////////////////////////////////////
void OceanCurrentWorldPlugin::LoadGlobalCurrentConfig()
{
  // Retrieve the velocity configuration, if existent

  sdf::ElementPtr currentVelocityParams;
  if (this->dataPtr->sdf->HasElement("constant_current"))
  {
    currentVelocityParams = this->dataPtr->sdf->GetElement("constant_current");
  }
  else
  {
    gzerr << "Current configuration not available" << std::endl;
  }

  if (currentVelocityParams->HasElement("use_constant_current"))
  {
    this->sharedDataPtr->use_constant_current =
      currentVelocityParams->Get<bool>("use_constant_current");
  }
  else
  {
    this->sharedDataPtr->use_constant_current = false;
  }

  // Read the topic names from the SDF file
  if (currentVelocityParams->HasElement("topic"))
  {
    this->sharedDataPtr->currentVelocityTopic = currentVelocityParams->Get<std::string>("topic");
  }
  else
  {
    this->sharedDataPtr->currentVelocityTopic = "ocean_current";
  }

  if (this->sharedDataPtr->currentVelocityTopic.empty())
  {
    gzerr << "Empty ocean current velocity topic" << std::endl;
  }

  if (currentVelocityParams->HasElement("velocity"))
  {
    sdf::ElementPtr elem = currentVelocityParams->GetElement("velocity");
    if (elem->HasElement("mean"))
    {
      this->sharedDataPtr->currentVelModel.mean = elem->Get<double>("mean");
    }
    if (elem->HasElement("min"))
    {
      this->sharedDataPtr->currentVelModel.min = elem->Get<double>("min");
    }
    if (elem->HasElement("max"))
    {
      this->sharedDataPtr->currentVelModel.max = elem->Get<double>("max");
    }
    if (elem->HasElement("mu"))
    {
      this->sharedDataPtr->currentVelModel.mu = elem->Get<double>("mu");
    }
    if (elem->HasElement("noiseAmp"))
    {
      this->sharedDataPtr->currentVelModel.noiseAmp = elem->Get<double>("noiseAmp");
    }

    if (!(this->sharedDataPtr->currentVelModel.min < this->sharedDataPtr->currentVelModel.max))
    {
      gzerr << "Invalid current velocity limits" << std::endl;
    }

    if (!(this->sharedDataPtr->currentVelModel.mean >= this->sharedDataPtr->currentVelModel.min))
    {
      gzerr << "Mean velocity must be greater than minimum" << std::endl;
    }
    if (!(this->sharedDataPtr->currentVelModel.mean <= this->sharedDataPtr->currentVelModel.max))
    {
      gzerr << "Mean velocity must be smaller than maximum" << std::endl;
    }

    if (!(this->sharedDataPtr->currentVelModel.mu >= 0 &&
          this->sharedDataPtr->currentVelModel.mu < 1))
    {
      gzerr << "Invalid process constant" << std::endl;
    }

    if (!(this->sharedDataPtr->currentVelModel.noiseAmp < 1 &&
          this->sharedDataPtr->currentVelModel.noiseAmp >= 0))
    {
      gzerr << "Noise amplitude has to be smaller than 1" << std::endl;
    }
  }

  this->sharedDataPtr->currentVelModel.var = this->sharedDataPtr->currentVelModel.mean;
  gzmsg << "Current velocity [m/s] Gauss-Markov process model:" << std::endl;
  this->sharedDataPtr->currentVelModel.Print();

  if (currentVelocityParams->HasElement("horizontal_angle"))
  {
    sdf::ElementPtr elem = currentVelocityParams->GetElement("horizontal_angle");

    if (elem->HasElement("mean"))
    {
      this->sharedDataPtr->currentHorzAngleModel.mean = elem->Get<double>("mean");
    }
    if (elem->HasElement("min"))
    {
      this->sharedDataPtr->currentHorzAngleModel.min = elem->Get<double>("min");
    }
    if (elem->HasElement("max"))
    {
      this->sharedDataPtr->currentHorzAngleModel.max = elem->Get<double>("max");
    }
    if (elem->HasElement("mu"))
    {
      this->sharedDataPtr->currentHorzAngleModel.mu = elem->Get<double>("mu");
    }
    if (elem->HasElement("noiseAmp"))
    {
      this->sharedDataPtr->currentHorzAngleModel.noiseAmp = elem->Get<double>("noiseAmp");
    }

    if (!(this->sharedDataPtr->currentHorzAngleModel.min <
          this->sharedDataPtr->currentHorzAngleModel.max))
    {
      gzerr << "Invalid current horizontal angle limits" << std::endl;
    }

    if (!(this->sharedDataPtr->currentHorzAngleModel.mean >=
          this->sharedDataPtr->currentHorzAngleModel.min))
    {
      gzerr << "Mean horizontal angle must be greater than minimum" << std::endl;
    }

    if (!(this->sharedDataPtr->currentHorzAngleModel.mean <=
          this->sharedDataPtr->currentHorzAngleModel.max))
    {
      gzerr << "Mean horizontal angle must be smaller than maximum" << std::endl;
    }

    if (!(this->sharedDataPtr->currentHorzAngleModel.mu >= 0 &&
          this->sharedDataPtr->currentHorzAngleModel.mu < 1))
    {
      gzerr << "Invalid process constant" << std::endl;
    }

    if (!(this->sharedDataPtr->currentHorzAngleModel.noiseAmp < 1 &&
          this->sharedDataPtr->currentHorzAngleModel.noiseAmp >= 0))
    {
      gzerr << "Noise amplitude for horizontal angle has to be between 0 and 1" << std::endl;
    }
  }

  this->sharedDataPtr->currentHorzAngleModel.var = this->sharedDataPtr->currentHorzAngleModel.mean;
  gzmsg << "Current velocity horizontal angle [rad] Gauss-Markov process model:" << std::endl;
  this->sharedDataPtr->currentHorzAngleModel.Print();

  if (currentVelocityParams->HasElement("vertical_angle"))
  {
    sdf::ElementPtr elem = currentVelocityParams->GetElement("vertical_angle");

    if (elem->HasElement("mean"))
    {
      this->sharedDataPtr->currentVertAngleModel.mean = elem->Get<double>("mean");
    }
    if (elem->HasElement("min"))
    {
      this->sharedDataPtr->currentVertAngleModel.min = elem->Get<double>("min");
    }
    if (elem->HasElement("max"))
    {
      this->sharedDataPtr->currentVertAngleModel.max = elem->Get<double>("max");
    }
    if (elem->HasElement("mu"))
    {
      this->sharedDataPtr->currentVertAngleModel.mu = elem->Get<double>("mu");
    }
    if (elem->HasElement("noiseAmp"))
    {
      this->sharedDataPtr->currentVertAngleModel.noiseAmp = elem->Get<double>("noiseAmp");
    }

    if (!(this->sharedDataPtr->currentVertAngleModel.min <
          this->sharedDataPtr->currentVertAngleModel.max))
    {
      gzerr << "Invalid current vertical angle limits" << std::endl;
    }

    if (!(this->sharedDataPtr->currentVertAngleModel.mean >=
          this->sharedDataPtr->currentVertAngleModel.min))
    {
      gzerr << "Mean vertical angle must be greater than minimum" << std::endl;
    }

    if (!(this->sharedDataPtr->currentVertAngleModel.mean <=
          this->sharedDataPtr->currentVertAngleModel.max))
    {
      gzerr << "Mean vertical angle must be smaller than maximum" << std::endl;
    }

    if (!(this->sharedDataPtr->currentVertAngleModel.mu >= 0 &&
          this->sharedDataPtr->currentVertAngleModel.mu < 1))
    {
      gzerr << "Invalid process constant" << std::endl;
    }

    if (!(this->sharedDataPtr->currentVertAngleModel.noiseAmp < 1 &&
          this->sharedDataPtr->currentVertAngleModel.noiseAmp >= 0))
    {
      gzerr << "Noise amplitude for vertical angle has to be between 0 and 1" << std::endl;
    }
  }

  this->sharedDataPtr->currentVertAngleModel.var = this->sharedDataPtr->currentVertAngleModel.mean;
  gzmsg << "Current velocity vertical angle [rad] Gauss-Markov process model:" << std::endl;
  this->sharedDataPtr->currentVertAngleModel.Print();

  this->sharedDataPtr->currentVelModel.lastUpdate =
    std::chrono::duration<double>(this->dataPtr->lastUpdate).count();
  this->sharedDataPtr->currentHorzAngleModel.lastUpdate =
    std::chrono::duration<double>(this->dataPtr->lastUpdate).count();
  this->sharedDataPtr->currentVertAngleModel.lastUpdate =
    std::chrono::duration<double>(this->dataPtr->lastUpdate).count();

  if (this->sharedDataPtr->use_constant_current)
  {  // Advertise the current velocity & stratified current velocity topics
    this->dataPtr->gz_node_cvel_world_pub = this->dataPtr->gz_node.Advertise<gz::msgs::Vector3d>(
      this->sharedDataPtr->currentVelocityTopic);
    gzmsg << "Current velocity topic name: " << this->sharedDataPtr->currentVelocityTopic
          << std::endl;
  }
}

// ----------------------------------------------

/////////////////////////////////////////////////
void OceanCurrentWorldPlugin::Update(
  const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm)
{
  // Update the time
  auto time = std::chrono::duration<double>(_info.simTime).count();

  // Calculate the flow velocity and the direction using the Gauss-Markov
  // model

  // Update current velocity
  double currentVelMag = this->sharedDataPtr->currentVelModel.Update(time);

  // Update current horizontal direction around z axis of flow frame
  double horzAngle = this->sharedDataPtr->currentHorzAngleModel.Update(time);

  // Update current horizontal direction around z axis of flow frame
  double vertAngle = this->sharedDataPtr->currentVertAngleModel.Update(time);

  // Generating the current velocity vector as in the North-East-Down frame
  this->sharedDataPtr->currentVelocity = gz::math::Vector3d(
    currentVelMag * cos(horzAngle) * cos(vertAngle),
    currentVelMag * sin(horzAngle) * cos(vertAngle), currentVelMag * sin(vertAngle));

  // Generate the depth-specific velocities
  this->sharedDataPtr->currentStratifiedVelocity.clear();
  for (int i = 0; i < this->sharedDataPtr->stratifiedDatabase.size(); i++)
  {
    double depth = this->sharedDataPtr->stratifiedDatabase[i].Z();
    currentVelMag = this->sharedDataPtr->stratifiedCurrentModels[i][0].Update(time);
    horzAngle = this->sharedDataPtr->stratifiedCurrentModels[i][1].Update(time);
    vertAngle = this->sharedDataPtr->stratifiedCurrentModels[i][2].Update(time);
    gz::math::Vector4d depthVel(
      currentVelMag * cos(horzAngle) * cos(vertAngle),
      currentVelMag * sin(horzAngle) * cos(vertAngle), currentVelMag * sin(vertAngle), depth);
    this->sharedDataPtr->currentStratifiedVelocity.push_back(depthVel);
  }
}

// ----------------------------------------------
/////////////////////////////////////////////////
void OceanCurrentWorldPlugin::PublishCurrentVelocity()
{
  gz::msgs::Vector3d currVel;
  gz::msgs::Set(
    &currVel, gz::math::Vector3d(
                this->sharedDataPtr->currentVelocity.X(), this->sharedDataPtr->currentVelocity.Y(),
                this->sharedDataPtr->currentVelocity.Z()));
  this->dataPtr->gz_node_cvel_world_pub.Publish(currVel);
}

/////////////////////////////////////////////////
void OceanCurrentWorldPlugin::PublishStratifiedCurrentVelocity()
{
  dave_gz_world_plugins_msgs::msgs::StratifiedCurrentVelocity stratcurrentVel;  // check
  for (std::vector<gz::math::Vector4d>::iterator it =
         this->sharedDataPtr->currentStratifiedVelocity.begin();
       it != this->sharedDataPtr->currentStratifiedVelocity.end(); ++it)
  {
    gz::msgs::Set(stratcurrentVel.add_velocity(), gz::math::Vector3d(it->X(), it->Y(), it->Z()));
    stratcurrentVel.add_depth(it->W());
  }
  if (stratcurrentVel.velocity_size() == 0)
  {
    return;
  }
  this->dataPtr->gz_node_scvel_world_pub.Publish(stratcurrentVel);
}

/////////////////////////////////////////////////
void OceanCurrentWorldPlugin::PostUpdate(
  const gz::sim::UpdateInfo & _info, const gz::sim::EntityComponentManager & _ecm)
{
  // gzmsg << "Ocean current world plugin post update" << std::endl;
  // Update time stamp
  this->dataPtr->lastUpdate = _info.simTime;
  if (this->sharedDataPtr->use_constant_current)
  {
    PublishCurrentVelocity();
  }
  PublishStratifiedCurrentVelocity();
}
// ----------------------------------------------

}  // namespace dave_gz_world_plugins