#ifndef MOVAILIDAR_HH_
#define MOVAILIDAR_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include "ignition/gazebo/Model.hh"
#include <ignition/common/Profiler.hh>
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/World.hh"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// #include <boost/bind.hpp>
// #include <boost/foreach.hpp>
// #include <boost/thread.hpp>

#include "Containers.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief MovaiLidar Plugin that can publish a scan/points message with random intensities
class MovaiLidar : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemUpdate
{
  /// \brief Constructor
  public: MovaiLidar();

  /// \brief Destructor
  public: ~MovaiLidar() override;

  //////////////////////////////////////////////////
  /// \brief This function is called when the model attached is loaded in the simulation
  /// \param[in] _entity Object model that this plugin is attached 
  /// \param[in] _sdf SDF element of the plugin in the model attached 
  /// \param[in] _ecm Entity Component Manager
  public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

  //////////////////////////////////////////////////
  /// \brief This function is called in each simulation step update.
  /// \param[in] _info Simulation state information
  /// \param[in] _ecm Entity Component Manager
  public: void Update(const ignition::gazebo::UpdateInfo &_info,
              ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Callbacks for control topic subscription
  /// \param[in] _msg Message that dictates if intensities are published
  private: void OnIntensityFlag(const ignition::msgs::Boolean &_msg);
  
  private: void InitLidarParams();

  private: void construct_lidar_frame(std::vector<float> const& ranges, cv::Mat const* cam_feed_);

  private: void get_colored_pc(PointCloud& pcl_msg);

  private: void get_intensity_pc(PointCloud_I& pcl_msg);

  private: void color_point(cv::Mat const& cam_feed, int const& azimuthIdx, int const& elevationIdx);

  private: void mount_position_compensation(int const& azimuthIdx, int const& elevationIdx);
  
  private: void Populate(double minVal, double maxVal, double resolution, std::vector<double>&vec);
  
  /// \brief Node responsible for subring and publishing desired topics
  private: ignition::transport::Node node;

  /// \brief Declare the needed publishers
  private: ignition::transport::Node::Publisher pcl_pub_;
  private: ignition::transport::Node::Publisher pcl_pub_I_;

  /// \brief Messages to send
  private: ignition::msgs::PointCloudPacked intensityPoints;
  private: ignition::msgs::LaserScan intensityScan;
  private: ignition::msgs::PointCloudPacked tmp_pcl;

  /// \brief Name of the model to which the plugin is attached
   public: std::string  objectName;

  /// \brief World name
  public: std::string  worldName;
  
  /// \brief Model entity that this plugin is attached
  public: Model model{kNullEntity};

  /// \brief Control flag
  public: bool allowIntensities{true};

  MovaiLidarParameters MLP;

  cv::dnn::Net net;

  std::string cam_topic_name[4];

  std::unique_ptr<SGM> sgm_ptr;
  private: PointCloud::Ptr colored_pcl_msg_ptr;
  private: PointCloud_I::Ptr intensity_pcl_msg_ptr;

  /// \brief topic name
  private:
  std::string topic_name_;

  /// \brief frame transform name, should match link name
  private:
  std::string frame_name_;

  /// \brief tf prefix
  private:
  std::string tf_prefix_;
  
  // Sensor callbacks
  private:
    void imageFnc(const ignition::msgs::Image &msg, int index);
    void imageCb0(const ignition::msgs::Image &msg);
    void imageCb1(const ignition::msgs::Image &msg);
    void imageCb2(const ignition::msgs::Image &msg);
    void imageCb3(const ignition::msgs::Image &msg);
    void OnScan(const ignition::msgs::LaserScan &_msg);

  private:
    std::string sensor_id;
    cv::Mat cam_feed[4];
    std::mutex mtx;

};

#endif
