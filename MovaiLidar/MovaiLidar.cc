/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <ignition/msgs/pointcloud_packed.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/transport/Node.hh>
#include "MovaiLidar.hh"
#include <ignition/sensors/SensorFactory.hh>

using namespace gz::sensors;
using namespace custom;

/// \brief Private data for the GpuLidar class
class custom::MovaiLidarPrivate
{
  /// \brief Fill the point cloud packed message
  /// \param[in] _laserBuffer Lidar data buffer.
  public: void FillPointCloudMsg(const float *_laserBuffer);

  /// \brief Rendering camera
  public: ignition::rendering::GpuRaysPtr gpuRays;

  /// \brief Connection to the Manager's scene change event.
  public: ignition::common::ConnectionPtr sceneChangeConnection;

  /// \brief Event that is used to trigger callbacks when a new
  /// lidar frame is available
  public: ignition::common::EventT<
          void(const float *_scan, unsigned int _width,
               unsigned int _height, unsigned int _channels,
               const std::string &_format)> lidarEvent;

  /// \brief Callback when new lidar frame is received
  public: void OnNewLidarFrame(const float *_scan, unsigned int _width,
               unsigned int _height, unsigned int _channels,
               const std::string &_format);

  /// \brief Connection to gpuRays new lidar frame event
  public: ignition::common::ConnectionPtr lidarFrameConnection;

  /// \brief The point cloud message.
  public: ignition::msgs::PointCloudPacked pointMsg;

  /// \brief Transport node.
  public: ignition::transport::Node node;

  /// \brief Publisher for the publish point cloud message.
  public: ignition::transport::Node::Publisher pointPub;

  public: ignition::transport::Node::Publisher pub;
    /// \brief Laser message to publish data.
  public: ignition::msgs::LaserScan laserMsg;

  /// \brief Noise added to sensor data
  // public: std::map<SensorNoiseType, NoisePtr> noises;

  /// \brief Sdf sensor.
  public: sdf::Lidar sdfLidar;
};

//////////////////////////////////////////////////
MovaiLidar::MovaiLidar()
  : dataPtr(new MovaiLidarPrivate())
{
}

//////////////////////////////////////////////////
MovaiLidar::~MovaiLidar()
{
  this->RemoveGpuRays(this->Scene());

  this->dataPtr->sceneChangeConnection.reset();

  if (this->laserBuffer)
  {
    delete [] this->laserBuffer;
    this->laserBuffer = nullptr;
  }
}

/////////////////////////////////////////////////
void MovaiLidar::SetScene(ignition::rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->lidarMutex);
  // APIs make it possible for the scene pointer to change
  if (this->Scene() != _scene)
  {
    this->RemoveGpuRays(this->Scene());
    RenderingSensor::SetScene(_scene);

    if (this->initialized)
      this->CreateLidar();
  }
}

//////////////////////////////////////////////////
void MovaiLidar::RemoveGpuRays(
    ignition::rendering::ScenePtr _scene)
{
  if (_scene)
  {
    _scene->DestroySensor(this->dataPtr->gpuRays);
  }
  this->dataPtr->gpuRays.reset();
  this->dataPtr->gpuRays = nullptr;
}

bool MovaiLidar::NewLoad(const sdf::Sensor &_sdf){
 // Load sensor element
  ignwarn<< "New Load " << std::endl;
  if (!this->Sensor::Load(_sdf))
  {
    ignwarn<< "returned false " << std::endl;
    return false;
  }


  // // Check if this is the right type
  // if (_sdf.Type() != sdf::SensorType::LIDAR &&
  //     _sdf.Type() != sdf::SensorType::GPU_LIDAR)
  // {
  //   ignerr << "Attempting to a load a Lidar sensor, but received "
  //     << "a " << _sdf.TypeStr() << std::endl;
  // }

  if (_sdf.LidarSensor() == nullptr)
  {
    ignerr << "Attempting to a load a Lidar sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  // Register publisher
  if (this->Topic().empty())
    this->SetTopic("/intense");

  this->dataPtr->pub =
      this->dataPtr->node.Advertise<ignition::msgs::LaserScan>(
        this->Topic());
  if (!this->dataPtr->pub)
  {
    ignerr << "Unable to create publisher on topic["
      << this->Topic() << "].\n";
    return false;
  }

  igndbg << "Laser scans for [" << this->Name() << "] advertised on ["
         << this->Topic() << "]" << std::endl;

  // Load ray atributes
  this->dataPtr->sdfLidar = *_sdf.LidarSensor();

  if (this->RayCount() == 0 || this->VerticalRayCount() == 0)
  {
    ignerr << "Lidar: Image has 0 size!\n";
  }

  // create message
  this->dataPtr->laserMsg.set_count(this->RangeCount());
  this->dataPtr->laserMsg.set_range_min(this->RangeMin());
  this->dataPtr->laserMsg.set_range_max(this->RangeMax());
  this->dataPtr->laserMsg.set_angle_min(this->AngleMin().Radian());
  this->dataPtr->laserMsg.set_angle_max(this->AngleMax().Radian());
  this->dataPtr->laserMsg.set_angle_step(this->AngleResolution());
  this->dataPtr->laserMsg.set_vertical_angle_min(
      this->VerticalAngleMin().Radian());
  this->dataPtr->laserMsg.set_vertical_angle_max(
      this->VerticalAngleMax().Radian());
  this->dataPtr->laserMsg.set_vertical_angle_step(
      this->VerticalAngleResolution());
  this->dataPtr->laserMsg.set_vertical_count(
      this->VerticalRangeCount());

  // Handle noise model settings.
  // const std::map<SensorNoiseType, sdf::Noise> noises = {
  //   {LIDAR_NOISE, this->dataPtr->sdfLidar.LidarNoise()},
  // };

  // for (const auto & [noiseType, noiseSdf] : noises)
  // {
  //   if (noiseSdf.Type() == sdf::NoiseType::GAUSSIAN)
  //   {
  //     this->dataPtr->noises[noiseType] =
  //       NoiseFactory::NewNoiseModel(noiseSdf);
  //   }
  //   else if (noiseSdf.Type() != sdf::NoiseType::NONE)
  //   {
  //     ignwarn << "The lidar sensor only supports Gaussian noise. "
  //      << "The supplied noise type[" << static_cast<int>(noiseSdf.Type())
  //      << "] is not supported." << std::endl;
  //   }
  // }
  ignwarn<< "initialized " << std::endl;
  this->initialized = true;
  return true;



}
//////////////////////////////////////////////////
bool MovaiLidar::Load(const sdf::Sensor &_sdf)
{
  ignwarn<< "Load sensor " << std::endl;
  // Check if this is being loaded via "builtin" or via another sensor
  if (!this->NewLoad(_sdf))
  {
    return false;
  }

  // Initialize the point message.
  // \todo(anyone) The true value in the following function call forces
  // the xyz and rgb fields to be aligned to memory boundaries. This is need
  // by ROS1: https://github.com/ros/common_msgs/pull/77. Ideally, memory
  // alignment should be configured. This same problem is in the
  // RgbdCameraSensor.
  ignition::msgs::InitPointCloudPacked(this->dataPtr->pointMsg, this->Name(), true,
      {{"xyz", ignition::msgs::PointCloudPacked::Field::FLOAT32},
      {"intensity", ignition::msgs::PointCloudPacked::Field::FLOAT32},
      {"ring", ignition::msgs::PointCloudPacked::Field::UINT16}});

  if (this->Scene())
    this->CreateLidar();

  this->dataPtr->sceneChangeConnection =
    RenderingEvents::ConnectSceneChangeCallback(
        std::bind(&MovaiLidar::SetScene, this, std::placeholders::_1));

  // Create the point cloud publisher
  this->SetTopic(this->Topic() + "/points");

  this->dataPtr->pointPub =
      this->dataPtr->node.Advertise<ignition::msgs::PointCloudPacked>(
          this->Topic());

  if (!this->dataPtr->pointPub)
  {
    ignerr << "Unable to create publisher on topic["
      << this->Topic() << "].\n";
    return false;
  }

  igndbg << "Lidar points for [" << this->Name() << "] advertised on ["
         << this->Topic() << "]" << std::endl;

  this->initialized = true;

  return true;
}

//////////////////////////////////////////////////
bool MovaiLidar::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  ignwarn<< "Pointer Load " << std::endl;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool MovaiLidar::Init()
{
  return this->Sensor::Init();
  ignwarn<< "Init " << std::to_string(this->Sensor::Init()) << std::endl;
}

//////////////////////////////////////////////////
bool MovaiLidar::CreateLidar()
{
  ignwarn<< "CreateLidar" << std::endl;
  this->dataPtr->gpuRays = this->Scene()->CreateGpuRays(
      this->Name());

  if (!this->dataPtr->gpuRays)
  {
    ignerr << "Unable to create gpu laser sensor\n";
    return false;
  }

  this->dataPtr->gpuRays->SetWorldPosition(this->Pose().Pos());
  this->dataPtr->gpuRays->SetWorldRotation(this->Pose().Rot());

  this->dataPtr->gpuRays->SetNearClipPlane(this->RangeMin());
  this->dataPtr->gpuRays->SetFarClipPlane(this->RangeMax());

  // Mask ranges outside of min/max to +/- inf, as per REP 117
  this->dataPtr->gpuRays->SetClamp(false);

  this->dataPtr->gpuRays->SetAngleMin(this->AngleMin().Radian());
  this->dataPtr->gpuRays->SetAngleMax(this->AngleMax().Radian());

  this->dataPtr->gpuRays->SetVerticalAngleMin(
      this->VerticalAngleMin().Radian());
  this->dataPtr->gpuRays->SetVerticalAngleMax(
      this->VerticalAngleMax().Radian());

  this->dataPtr->gpuRays->SetRayCount(this->RayCount());
  this->dataPtr->gpuRays->SetVerticalRayCount(
      this->VerticalRayCount());

  this->Scene()->RootVisual()->AddChild(
      this->dataPtr->gpuRays);

  // Set the values on the point message.
  this->dataPtr->pointMsg.set_width(this->dataPtr->gpuRays->RangeCount());
  this->dataPtr->pointMsg.set_height(
      this->dataPtr->gpuRays->VerticalRangeCount());
  this->dataPtr->pointMsg.set_row_step(
      this->dataPtr->pointMsg.point_step() *
      this->dataPtr->pointMsg.width());
  this->dataPtr->gpuRays->SetVisibilityMask(this->VisibilityMask());

  this->dataPtr->lidarFrameConnection =
      this->dataPtr->gpuRays->ConnectNewGpuRaysFrame(
      std::bind(&MovaiLidar::OnNewLidarFrame, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));

  this->AddSensor(this->dataPtr->gpuRays);
  ignwarn<< "Added sensor"<< std::endl;
  return true;
}

/////////////////////////////////////////////////
void MovaiLidar::OnNewLidarFrame(const float *_data,
    unsigned int _width, unsigned int _height, unsigned int _channels,
    const std::string &_format)
{
  ignwarn<< "On new lidar frame"<< std::endl;
  std::lock_guard<std::mutex> lock(this->lidarMutex);

  unsigned int samples = _width * _height * _channels;
  unsigned int lidarBufferSize = samples * sizeof(float);

  if (!this->laserBuffer)
    this->laserBuffer = new float[samples];

  memcpy(this->laserBuffer, _data, lidarBufferSize);

  if (this->dataPtr->lidarEvent.ConnectionCount() > 0)
  {
    this->dataPtr->lidarEvent(_data, _width, _height, _channels, _format);
  }
}

//////////////////////////////////////////////////
bool MovaiLidar::Update(const std::chrono::steady_clock::duration &_now)
{
  IGN_PROFILE("MovaiLidar::Update");
  if (!this->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->gpuRays)
  {
    ignerr << "GpuRays doesn't exist.\n";
    return false;
  }
  ignwarn<< "Clock Update " << std::endl;
  this->Render();

  // Apply noise before publishing the data.
  this->ApplyNoise();

  this->PublishLidarScan(_now);

  if (this->dataPtr->pointPub.HasConnections())
  {
    // Set the time stamp
    *this->dataPtr->pointMsg.mutable_header()->mutable_stamp() =
      ignition::msgs::Convert(_now);
    // Set frame_id
    for (auto i = 0;
         i < this->dataPtr->pointMsg.mutable_header()->data_size();
         ++i)
    {
      if (this->dataPtr->pointMsg.mutable_header()->data(i).key() == "frame_id"
          && this->dataPtr->pointMsg.mutable_header()->data(i).value_size() > 0)
      {
        this->dataPtr->pointMsg.mutable_header()->mutable_data(i)->set_value(
              0,
              this->FrameId());
      }
    }

    this->dataPtr->FillPointCloudMsg(this->laserBuffer);

    {
      this->AddSequence(this->dataPtr->pointMsg.mutable_header());
      IGN_PROFILE("MovaiLidar::Update Publish point cloud");
      this->dataPtr->pointPub.Publish(this->dataPtr->pointMsg);
    }
  }
  return true;
}

/////////////////////////////////////////////////
ignition::common::ConnectionPtr MovaiLidar::ConnectNewLidarFrame(
          std::function<void(const float *_scan, unsigned int _width,
                  unsigned int _height, unsigned int _channels,
                  const std::string &/*_format*/)> _subscriber)
{
  return this->dataPtr->lidarEvent.Connect(_subscriber);
}

/////////////////////////////////////////////////
ignition::rendering::GpuRaysPtr MovaiLidar::GpuRays() const
{
  return this->dataPtr->gpuRays;
}

//////////////////////////////////////////////////
bool MovaiLidar::IsHorizontal() const
{
  return this->dataPtr->gpuRays->IsHorizontal();
}

//////////////////////////////////////////////////
ignition::math::Angle MovaiLidar::HFOV() const
{
  return this->dataPtr->gpuRays->HFOV();
}

//////////////////////////////////////////////////
ignition::math::Angle MovaiLidar::VFOV() const
{
  return this->dataPtr->gpuRays->VFOV();
}

//////////////////////////////////////////////////
bool MovaiLidar::HasConnections() const
{
  return Lidar::HasConnections() ||
     (this->dataPtr->pointPub && this->dataPtr->pointPub.HasConnections()) ||
     this->dataPtr->lidarEvent.ConnectionCount() > 0u;
}

//////////////////////////////////////////////////
void MovaiLidarPrivate::FillPointCloudMsg(const float *_laserBuffer)
{
  IGN_PROFILE("MovaiLidarPrivate::FillPointCloudMsg");
  uint32_t width = this->pointMsg.width();
  uint32_t height = this->pointMsg.height();
  unsigned int channels = 3;
  ignwarn << std::to_string(width) << std::endl;
  ignerr << std::to_string(height) << std::endl;
  float angleStep =
    (this->gpuRays->AngleMax() - this->gpuRays->AngleMin()).Radian() /
    (this->gpuRays->RangeCount()-1);

  float verticleAngleStep = (this->gpuRays->VerticalAngleMax() -
      this->gpuRays->VerticalAngleMin()).Radian() /
    (this->gpuRays->VerticalRangeCount()-1);

  // Angles of ray currently processing, azimuth is horizontal, inclination
  // is vertical
  float inclination = this->gpuRays->VerticalAngleMin().Radian();

  std::string *msgBuffer = this->pointMsg.mutable_data();
  msgBuffer->resize(this->pointMsg.row_step() *
      this->pointMsg.height());
  char *msgBufferIndex = msgBuffer->data();
  // Set Pointcloud as dense. Change if invalid points are found.
  bool isDense { true };
  // Iterate over scan and populate point cloud
  for (uint32_t j = 0; j < height; ++j)
  {
    float azimuth = this->gpuRays->AngleMin().Radian();

    for (uint32_t i = 0; i < width; ++i)
    {
      // Index of current point, and the depth value at that point
      auto index = j * width * channels + i * channels;
      float depth = _laserBuffer[index];
      // Validate Depth/Radius and update pointcloud density flag
      if (isDense)
        isDense = !(ignition::math::isnan(depth) || std::isinf(depth));

      float intensity = _laserBuffer[index + 1];
      uint16_t ring = j;

      int fieldIndex = 0;

      // Convert spherical coordinates to Cartesian for pointcloud
      // See https://en.wikipedia.org/wiki/Spherical_coordinate_system
      *reinterpret_cast<float *>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) =
        depth * std::cos(inclination) * std::cos(azimuth);

      *reinterpret_cast<float *>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) =
        depth * std::cos(inclination) * std::sin(azimuth);

      *reinterpret_cast<float *>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) =
        depth * std::sin(inclination);

      // Intensity
      *reinterpret_cast<float *>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) = intensity;

      // Ring
      *reinterpret_cast<uint16_t *>(msgBufferIndex +
          this->pointMsg.field(fieldIndex++).offset()) = ring;

      // Move the index to the next point.
      msgBufferIndex += this->pointMsg.point_step();

      azimuth += angleStep;
    }
    inclination += verticleAngleStep;
  }
  this->pointMsg.set_is_dense(isDense);
}
