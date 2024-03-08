#include <ignition/plugin/Register.hh>

#include "include/MovaiLidar.hh"
#include <random>
#include <ignition/msgs/PointCloudPackedUtils.hh>
#include <ignition/msgs/pointcloud_packed.pb.h>
#include <ignition/msgs/image.pb.h>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ctime>

using namespace ignition;
using namespace gazebo;
using namespace systems;

/////////////////////////////////////////////////
/// \brief Constructor
MovaiLidar::MovaiLidar(){
  this->MLP.parameters_initialized = false;
}

/////////////////////////////////////////////////
/// \brief Destructor
MovaiLidar::~MovaiLidar(){}

// Initialize MovaiLidar Parameters class and DNN
void MovaiLidar::InitLidarParams(){
  this->MLP.parameters_initialized = true;
  Populate(this->MLP.min_horizontal_angle, this->MLP.max_horizontal_angle + this->MLP.horizontal_step_angle, 
           this->MLP.horizontal_step_angle, this->MLP.azimuthVec);
  Populate(this->MLP.min_vertical_angle, this->MLP.max_vertical_angle + this->MLP.vertical_step_angle, 
           this->MLP.vertical_step_angle, this->MLP.elevationVec);
  this-> net = cv::dnn::readNet(this->MLP.DnnPath);
}

// Based on the received laserscan message, create 2 PCLs: one with color and one with intensity values
void MovaiLidar::construct_lidar_frame(std::vector<float> const& ranges, cv::Mat const* cam_feed_) {
    if (ranges.empty()) {
        ignerr << "Ranges are empty" << std::endl;
        return;
    }
    
    colored_pcl_msg_ptr = PointCloud::Ptr(new PointCloud());
    intensity_pcl_msg_ptr = PointCloud_I::Ptr(new PointCloud_I());
    
    // Create segmentation pointer that holds information on the pointcloud fields
    sgm_ptr = std::make_unique<SGM>(this->MLP.vertical_points_count, this->MLP.horizontal_points_count);
    
    // Based on the size of the laserscan, create 2 pointclouds:
    // 1) Color pointcloud: has the xyz fields along with rgba
    // 2) Intensity pointcloud: has xyz fields along with intensity value
    for (int pointIdx = 0; pointIdx < ranges.size(); pointIdx++) {
        int azimuthIdx = pointIdx % static_cast<int>(this->MLP.horizontal_points_count);
        int elevationIdx = pointIdx / static_cast<int>(this->MLP.horizontal_points_count);
        float r{0};
        
        // Disregard ranges values that returned infinity
        if (std::numeric_limits<float>::infinity() == ranges[pointIdx])
            r = 0;
        else
            r = ranges[pointIdx];
        
        // Spherical to cartesian coordinates transformation from LIDAR frame of reference.
        // We use elevation instead of inclination, which means sin and cos of theta are switched from normal spherical equations
        sgm_ptr->x.at<float>(elevationIdx, azimuthIdx) =
            r * cos(this->MLP.azimuthVec[azimuthIdx]) * cos(this->MLP.elevationVec[elevationIdx]);
        sgm_ptr->y.at<float>(elevationIdx, azimuthIdx) =
            r * sin(this->MLP.azimuthVec[azimuthIdx]) * cos(this->MLP.elevationVec[elevationIdx]);
        sgm_ptr->z.at<float>(elevationIdx, azimuthIdx) = r * sin(this->MLP.elevationVec[elevationIdx]);
        sgm_ptr->d.at<float>(elevationIdx, azimuthIdx) = r / this->MLP.maxSensorTrainingRange;
        sgm_ptr->a.at<float>(elevationIdx, azimuthIdx)  = 0;

        // Extract Point color from LIDAR frame of reference.
        // TODO: Remove hardcoded values
        (void)color_point(cam_feed_[int(azimuthIdx/512)], azimuthIdx, elevationIdx);

        // Compensate the mounting position of the LiDAR sensor to the Frame of reference of the vehicle.
        (void)mount_position_compensation(azimuthIdx, elevationIdx);
        
        // Populate color PCL
        pcl::PointXYZRGBL colored_pt = pcl::PointXYZRGBL();
        colored_pt.x = sgm_ptr->x.at<float>(elevationIdx, azimuthIdx);
        colored_pt.y = sgm_ptr->y.at<float>(elevationIdx, azimuthIdx);
        colored_pt.z = sgm_ptr->z.at<float>(elevationIdx, azimuthIdx);
        colored_pt.r = sgm_ptr->r.at<float>(elevationIdx, azimuthIdx) * 255.0;
        colored_pt.g = sgm_ptr->g.at<float>(elevationIdx, azimuthIdx) * 255.0;
        colored_pt.b = sgm_ptr->b.at<float>(elevationIdx, azimuthIdx) * 255.0;
        colored_pt.label = 0;
        colored_pcl_msg_ptr->points.push_back(colored_pt);
        
        // Populate intensity PCL
        pcl::PointXYZI Intensity_pt = pcl::PointXYZI();
        Intensity_pt.x = colored_pt.x;
        Intensity_pt.y = colored_pt.y;
        Intensity_pt.z = colored_pt.z;
        Intensity_pt.intensity = 0;
        intensity_pcl_msg_ptr->points.push_back(Intensity_pt);
    }
    
    // Setup input matrix fro DNN 
    std::vector<cv::Mat> channels = {sgm_ptr->d, sgm_ptr->a, sgm_ptr->r, sgm_ptr->g, sgm_ptr->b};
    cv::Mat inputBlob = cv::dnn::blobFromImages(channels, 1.0,      // scale factor
                                                sgm_ptr->d.size(),  // spatial size for output image
                                                cv::Scalar(0),      // mean
                                                false,              // swapRB: BGR to RGB
                                                false,              // crop
                                                CV_32F              // Depth of output blob. Choose CV_32F or CV_8U.
    );
    inputBlob = inputBlob.reshape(1, std::vector<int>({1, 5, sgm_ptr->height, sgm_ptr->width}));
    inputBlob.setTo(0, inputBlob < 0);
    inputBlob.setTo(1, inputBlob > 1);
    cv::patchNaNs(inputBlob, 0);

    net.setInput(inputBlob);
    cv::Mat output = net.forward();

    // Get output from the network
    std::vector<cv::Mat> outputVectors;
    cv::dnn::imagesFromBlob(output, outputVectors);

    // Populate intensity field
    for (int pointIdx = 0; pointIdx < intensity_pcl_msg_ptr->points.size(); pointIdx++) {
        int azimuthIdx = pointIdx % static_cast<int>(this->MLP.horizontal_points_count);
        int elevationIdx = pointIdx / static_cast<int>(this->MLP.horizontal_points_count);
        // Limiting of the intensity from 0-1.
        if (outputVectors[0].at<float>(elevationIdx, azimuthIdx) < 0){
            outputVectors[0].at<float>(elevationIdx, azimuthIdx) = 0;
        }
        if (outputVectors[0].at<float>(elevationIdx, azimuthIdx) > 1){
            outputVectors[0].at<float>(elevationIdx, azimuthIdx) = 1;
        }
        intensity_pcl_msg_ptr->points[pointIdx].intensity = outputVectors[0].at<float>(elevationIdx, azimuthIdx);
       
    }


}
// Returns a pointcloud<XYZRGB>
void MovaiLidar::get_colored_pc(PointCloud& pcl_msg) {
    pcl_msg.points = colored_pcl_msg_ptr->points;

}

// Returns a pointcloud <XYZI>
void MovaiLidar::get_intensity_pc(PointCloud_I& pcl_msg) {
    pcl_msg.points = intensity_pcl_msg_ptr->points;
}

// Fuse camera with lidar point cloud.
void MovaiLidar::color_point(cv::Mat const& cam_feed, int const& azimuthIdx, int const& elevationIdx) {
    std::uint8_t R{0}, G{0}, B{0};
    if (this->MLP.enable_fusion) {

        // Compute camera intrinsic params
        float focalLength = (cam_feed.cols / 2) / tan(this->MLP.cam_fov / 2);
        float cx = cam_feed.cols / 2;
        float cy = cam_feed.rows / 2;

        float x = sgm_ptr->x.at<float>(elevationIdx, azimuthIdx);
        float y = sgm_ptr->y.at<float>(elevationIdx, azimuthIdx);
        float z = sgm_ptr->z.at<float>(elevationIdx, azimuthIdx);

        // TODO: Remove hardcoded values
        float angle_cameras[] = {-2.3562, -0.7854, 0.7854, 2.3562};
        float angle_radians = -1.0 * angle_cameras[azimuthIdx/512];
        float new_x = x * cos(angle_radians) - y * sin(angle_radians);
        float new_y = x * sin(angle_radians) + y * cos(angle_radians);
        float new_z = z;

        // Camera x-axis is LiDAR -1*y-axis, 
        float X_cam = -new_y;
        // camera y-axis is LiDAR -1*z-axis, and
        float Y_cam = -new_z;
        // camera z-axis(depth) is LiDAR x_axis.
        float Z_cam = new_x;

        X_cam = static_cast<int>(cx + X_cam * focalLength / Z_cam);
        Y_cam = static_cast<int>(cy + Y_cam * focalLength / Z_cam);
        if (Y_cam < cy * 2 - 1 && Y_cam > 0 && X_cam < cx * 2 - 1 && X_cam > 0 && Z_cam > 0) {
            sgm_ptr->b.at<float>(elevationIdx, azimuthIdx) = cam_feed.at<cv::Vec3b>(Y_cam, X_cam)[2] / 255.0;
            sgm_ptr->g.at<float>(elevationIdx, azimuthIdx) = cam_feed.at<cv::Vec3b>(Y_cam, X_cam)[1] / 255.0;
            sgm_ptr->r.at<float>(elevationIdx, azimuthIdx) = cam_feed.at<cv::Vec3b>(Y_cam, X_cam)[0] / 255.0;
        }
    } else {
        ignerr << "Failed coloring points " << std::endl;
    }
}

// Account for Lidar offset from origin
void MovaiLidar::mount_position_compensation(int const& azimuthIdx, int const& elevationIdx) {
    float x = sgm_ptr->x.at<float>(elevationIdx, azimuthIdx) * cos(this->MLP.mp.yaw) * cos(this->MLP.mp.pitch) +
              sgm_ptr->y.at<float>(elevationIdx, azimuthIdx) *
                  (cos(this->MLP.mp.yaw) * sin(this->MLP.mp.pitch) * sin(this->MLP.mp.roll) -
                   sin(this->MLP.mp.yaw) * cos(this->MLP.mp.roll)) +
              sgm_ptr->z.at<float>(elevationIdx, azimuthIdx) *
                  (cos(this->MLP.mp.yaw) * sin(this->MLP.mp.pitch) * cos(this->MLP.mp.roll) +
                   sin(this->MLP.mp.yaw) * sin(this->MLP.mp.roll)) +
              this->MLP.mp.x;
    float y = sgm_ptr->x.at<float>(elevationIdx, azimuthIdx) * sin(this->MLP.mp.yaw) * cos(this->MLP.mp.pitch) +
              sgm_ptr->y.at<float>(elevationIdx, azimuthIdx) *
                  (sin(this->MLP.mp.yaw) * sin(this->MLP.mp.pitch) * sin(this->MLP.mp.roll) +
                   cos(this->MLP.mp.yaw) * cos(this->MLP.mp.roll)) +
              sgm_ptr->z.at<float>(elevationIdx, azimuthIdx) *
                  (sin(this->MLP.mp.yaw) * sin(this->MLP.mp.pitch) * cos(this->MLP.mp.roll) -
                   cos(this->MLP.mp.yaw) * sin(this->MLP.mp.roll)) +
              this->MLP.mp.y;
    float z = sgm_ptr->x.at<float>(elevationIdx, azimuthIdx) * (-sin(this->MLP.mp.pitch)) +
              sgm_ptr->y.at<float>(elevationIdx, azimuthIdx) * cos(this->MLP.mp.pitch) * sin(this->MLP.mp.roll) +
              sgm_ptr->z.at<float>(elevationIdx, azimuthIdx) * cos(this->MLP.mp.pitch) * cos(this->MLP.mp.roll) +
              this->MLP.mp.z;
    sgm_ptr->x.at<float>(elevationIdx, azimuthIdx) = x;
    sgm_ptr->y.at<float>(elevationIdx, azimuthIdx) = y;
    sgm_ptr->z.at<float>(elevationIdx, azimuthIdx) = z;
}

// Populate vector based on min, max and resolution values
void MovaiLidar::Populate(double minVal, double maxVal, double resolution, std::vector<double>&vec){
  if (!vec.empty()){
    ignerr << "Tried to populate vector, but it is not empty" << std::endl; 
  }
  // We get a vector varying from min val to max val with a step resolution.
  double minimum = std::min(minVal, maxVal);
  double maximum = std::max(minVal, maxVal);
  for (double i = minimum; i < maximum; i += resolution) {
      vec.push_back(i);
  }
}


//////////////////////////////////////////////////
/// \brief This function is called when the model attached is loaded in the simulation
/// \param[in] _entity Object model that this plugin is attached 
/// \param[in] _sdf SDF element of the plugin in the model attached 
/// \param[in] _ecm Entity Component Manager
void MovaiLidar::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  // Get the model and check if it is valid
  this->model = Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    ignerr << "Lidar Model plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
  
  // Get the world name in the simulation
  auto worldEntity = _ecm.EntityByComponents(components::World());
  if (worldEntity == kNullEntity)
    return;

  // Get world name to build the topic string
  this->worldName = _ecm.Component<components::Name>(worldEntity)->Data();


 // this->parentEntity = _ecm.Component<components::ParentEntity>(_entity)->Data();
 // TODO: Verify parent entity is a gpu_lidar

// Get Plugin parameter values
if (!_sdf->HasElement("frameName")) {
        ignerr << "No frame name defined, defaulting to /world" << std::endl;
        this->frame_name_ = "/world";
    } else
        this->frame_name_ = _sdf->Get<std::string>("frameName");

    if (!_sdf->HasElement("topicName")) {
      ignerr << "No topic name defined, defaulting to /world" << std::endl;
        this->topic_name_ = "/world";
    } else
        this->topic_name_ = _sdf->Get<std::string>("topicName");

   // Get sensor position
    std::string sensor_pose;
    if (!_sdf->HasElement("pose")) {
        ignerr << "LiDAR, Laser plugin missing <pose>, defaults to <0, 0, 0, 0, 0, 0>" << std::endl;
        sensor_pose = "0 0 0 0 0 0";
    } else
        sensor_pose = _sdf->Get<std::string>("pose");
    this->MLP.mp.Set(sensor_pose);


    if (!_sdf->HasElement("cam_topic_name")) {
        ignerr << "LiDAR, Laser plugin missing <cam_topic_name>, fusion disabled" << std::endl;
        this->MLP.enable_fusion = false;
    } else {
        this->cam_topic_name[0] = "/world/world_demo/model/tugbot7/link/scan_omni/sensor/camera_back_right/image";
        this->cam_topic_name[1] = "/world/world_demo/model/tugbot7/link/scan_omni/sensor/camera_front_right/image";
        this->cam_topic_name[2] = "/world/world_demo/model/tugbot7/link/scan_omni/sensor/camera_front_left/image";
        this->cam_topic_name[3] = "/world/world_demo/model/tugbot7/link/scan_omni/sensor/camera_back_left/image";
        this->MLP.enable_fusion = true;
    }

    if (!_sdf->HasElement("cam_fov")) {
        ignerr<<"LiDAR, Laser plugin missing <cam_fov>, defaults to 0.0001" << std::endl;
        this->MLP.cam_fov = 0.0001;
    } else {
        std::string s_cam_fov = _sdf->Get<std::string>("cam_fov");
        this->MLP.cam_fov = std::stof(s_cam_fov);
    }

    if (!_sdf->HasElement("numOfClasses")) {
        ignerr<<"LiDAR, Laser plugin missing <numOfClasses>, defaults to 4"<< std::endl;
        this->MLP.numOfClasses = 4;
    } else {
        std::string s_numOfClasses = _sdf->Get<std::string>("numOfClasses");
        this->MLP.numOfClasses = std::stof(s_numOfClasses);
    }

    if (!_sdf->HasElement("maxSensorTrainingRange")) {
        ignerr<<"LiDAR, Laser plugin missing <maxSensorTrainingRange>, defaults to 80" << std::endl;
        this->MLP.maxSensorTrainingRange = 80;
    } else {
        std::string s_maxSensorTrainingRange = _sdf->Get<std::string>("maxSensorTrainingRange");
        this->MLP.maxSensorTrainingRange = std::stof(s_maxSensorTrainingRange);
    }

    if (!_sdf->HasElement("DnnPath")) {
        ignerr<<"LiDAR,Laser plugin missing <DnnPath>, default" << std::endl;
        this->MLP.DnnPath = "/movai_ign_plugins/system/network/frozen_graph_fastUnet.pb";
    } else {
        std::string s_DnnPath = _sdf->Get<std::string>("DnnPath");
        this->MLP.DnnPath = s_DnnPath;
    }

    if (!_sdf->HasElement("sensorId")) {
        ignerr<<"LiDAR, Laser plugin missing <sensorId>, defaults to 0"<< std::endl;
        this->sensor_id = "0";
    } else {
        this->sensor_id = _sdf->Get<std::string>("sensorId");
    }

  // Get the component's name to create the scan and points topic subscribers
  std::string objectName = _ecm.Component<components::Name>(_entity)->Data();

  // Get world name to build the topic string
  this->worldName = _ecm.Component<components::Name>(worldEntity)->Data();

  // Topics to subscribe
  std::string scan_topic = "/world/" + this->worldName + "/model/" + objectName + "/link/scan_omni/sensor/scan_omni/scan";
  std::string points_topic = "/world/" + this->worldName + "/model/" + objectName + "/link/scan_omni/sensor/scan_omni/scan/points";
  std::string intensity_flag = "/allow_intensities";

  // Declare subriber for intensity enabler flag
  this->node.Subscribe(intensity_flag, &MovaiLidar::OnIntensityFlag,
                                this);

  // Topics to publish
  std::string pcl_topic = "LiDAR" + this->sensor_id + "/pcl";
  std::string pclI_topic = "LiDAR" + this->sensor_id + "/pcl_I";
  if (this->topic_name_ != "") {
      
      // Declare publisher for color pcl 
      this->pcl_pub_ = this->node.Advertise<ignition::msgs::PointCloudPacked>(pcl_topic);
  }

  // Subscribe to input video
  this->node.Subscribe(this->cam_topic_name[0], &MovaiLidar::imageCb0, this);
  this->node.Subscribe(this->cam_topic_name[1], &MovaiLidar::imageCb1, this);
  this->node.Subscribe(this->cam_topic_name[2], &MovaiLidar::imageCb2, this);
  this->node.Subscribe(this->cam_topic_name[3], &MovaiLidar::imageCb3, this);
  this->node.Subscribe(scan_topic, &MovaiLidar::OnScan, this);

  this->pcl_pub_I_ = this->node.Advertise<ignition::msgs::PointCloudPacked>(pclI_topic);
}

//////////////////////////////////////////////////
/// \brief This function is called in each simulation step update.
/// \param[in] _info Simulation state information
/// \param[in] _ecm Entity Component Manager
void MovaiLidar::Update(const ignition::gazebo::UpdateInfo &_info,
                            ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("MovaiLidar::Update");

  using cast = std::chrono::duration<std::uint64_t>;
   this-> current_time = std::chrono::duration_cast< cast >(_info.simTime).count();
}

// //////////////////////////////////////////////////
/// \brief Callback for model PoseArray message subscription
/// \param[in] _msg Message
void MovaiLidar::OnIntensityFlag(const ignition::msgs::Boolean &_msg)
{   
    //Flag that defines if intensities are to be published
    this->allowIntensities = _msg.data();

   
}

// Receives an image message and transform it into a matrix
void MovaiLidar::imageFnc(const ignition::msgs::Image &msg, int index) {

    int width;
    int height;
    char *data;

    width = (int) msg.width();
    height = (int) msg.height();
    data = new char[msg.data().length()];

    memcpy(data, msg.data().c_str(), msg.data().length());
    cv::Mat image(height, width, CV_8UC3, data);
    mtx.lock();
    this->cam_feed[index] = image.clone();
    mtx.unlock();

}

////////////////////////////////////////////////////////////////////////////////
// Camera callback (we have 4 cameras to match the lidar fov)
void MovaiLidar::imageCb0(const ignition::msgs::Image &msg) {

  this->imageFnc(msg,0);

}
void MovaiLidar::imageCb1(const ignition::msgs::Image &msg) {

  this->imageFnc(msg,1);

}
void MovaiLidar::imageCb2(const ignition::msgs::Image &msg) {

  this->imageFnc(msg,2);

}
void MovaiLidar::imageCb3(const ignition::msgs::Image &msg) {

  this->imageFnc(msg,3);

}


// //////////////////////////////////////////////////////////////////////////////////TODO: Account for new lasers connections
// // Increment count
// void RealLiDARSensor::LaserConnect() {
//     lock.lock();
//     this->laser_connect_count_++;
//     lock.unlock();
//     if (this->laser_connect_count_ == 1) {
//         this->laser_scan_sub_ =
//             this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(), &RealLiDARSensor::OnScan, this);
//         parent_ray_sensor_->SetActive(true);
//     }
// }

// // Decrement count
// void RealLiDARSensor::LaserDisconnect() {
//     lock.lock();
//     this->laser_connect_count_--;
//     lock.unlock();
//     if (this->laser_connect_count_ == 0) 
//         this->laser_scan_sub_.reset();
//     parent_ray_sensor_->SetActive(false);
// }

////////////////////////////////////////////////////////////////////////////////
//Laserscan form the 3D lidar callback
void MovaiLidar::OnScan(const ignition::msgs::LaserScan& _msg) {

    //Populate MovaiLidar Parameters
    this->MLP.min_horizontal_angle = _msg.angle_min();
    this->MLP.max_horizontal_angle = _msg.angle_max();
    this->MLP.horizontal_step_angle = _msg.angle_step();

    this->MLP.min_vertical_angle = _msg.vertical_angle_min();
    this->MLP.max_vertical_angle = _msg.vertical_angle_max();
    this->MLP.vertical_step_angle = _msg.vertical_angle_step();

    this->MLP.vertical_points_count = _msg.vertical_count();
    this->MLP.horizontal_points_count = _msg.count();

    if (!this->MLP.parameters_initialized) {
        this->InitLidarParams();
    }
    // start the clock (for frequency debug purposes)
    clock_t begin = clock();

    // copy the ranges of distances from msg data containers, to a vector.
    std::vector<float> ranges;
    ranges.resize(_msg.ranges_size());
    std::copy(_msg.ranges().begin(), _msg.ranges().end(), ranges.begin());

    // Get images feedback
    mtx.lock();
    cv::Mat cam_feed_[4];
    for(int i=0; i<4; i++)
        cam_feed_[i] = this->cam_feed[i].clone();
    mtx.unlock();

    // Create a PCL_msg
    PointCloud pcl_msg;
    pcl_msg.header.frame_id = this->frame_name_;
    pcl_msg.header.stamp = this->current_time;
    pcl_msg.height = this->MLP.vertical_points_count;
    pcl_msg.width = this->MLP.horizontal_points_count;

    // Construct 3D PCL based on Laserscan and Images feedback
    (void)this->construct_lidar_frame(ranges, cam_feed_);

    //Color 3D PCL
    (void)this->get_colored_pc(pcl_msg);

    //Create Ignition pcl colored message
    ignition::msgs::PointCloudPacked pclpc;
    InitPointCloudPacked(pclpc, this->frame_name_, true,
      {{"xyz", ignition::msgs::PointCloudPacked::Field::FLOAT32},
       {"rgba", ignition::msgs::PointCloudPacked::Field::FLOAT32}});

    pclpc.set_height(pcl_msg.height);
    pclpc.set_width(pcl_msg.width);
    pclpc.set_is_bigendian(false);
    pclpc.set_is_dense(true);
    pclpc.mutable_data()->resize(pcl_msg.points.size()*pclpc.point_step());

    // Declare iterators for each field present in the new Ignition PCL<XYZRGB>
    ignition::msgs::PointCloudPackedIterator<uint8_t> iter_cr(pclpc, "r");
    ignition::msgs::PointCloudPackedIterator<uint8_t> iter_cg(pclpc, "g");
    ignition::msgs::PointCloudPackedIterator<uint8_t> iter_cb(pclpc, "b");
    ignition::msgs::PointCloudPackedIterator<uint8_t> iter_ca(pclpc, "a");
    ignition::msgs::PointCloudPackedIterator<float> iter_cx(pclpc, "x");
    ignition::msgs::PointCloudPackedIterator<float> iter_cy(pclpc, "y");
    ignition::msgs::PointCloudPackedIterator<float> iter_cz(pclpc, "z");

    // Populate new IGN PCL with colored points
    for(int counter = 0; counter <= pcl_msg.points.size() - 1; counter++){

      *iter_cr = int(pcl_msg.points[counter].r);
      *iter_cg = int(pcl_msg.points[counter].g);
      *iter_cb = int(pcl_msg.points[counter].b);
      *iter_ca = int(pcl_msg.points[counter].a);
      *iter_cx = pcl_msg.points[counter].x;
      *iter_cy = pcl_msg.points[counter].y;
      *iter_cz = pcl_msg.points[counter].z;


      ++iter_cr;
      ++iter_cg;
      ++iter_cb;
      ++iter_ca;
      ++iter_cx;
      ++iter_cy;
      ++iter_cz;
    }
    this->pcl_pub_.Publish(pclpc);



    // Create a PCL_I_msg
    PointCloud_I pcl_msg_I;

    pcl_msg_I.header.frame_id = this->frame_name_;
    pcl_msg_I.header.stamp = this->current_time;
    pcl_msg_I.height = this->MLP.vertical_points_count;
    pcl_msg_I.width = this->MLP.horizontal_points_count;
    (void)this->get_intensity_pc(pcl_msg_I);
   
    //Create Ignition pcl with intensity field message
    ignition::msgs::PointCloudPacked pclp;
    InitPointCloudPacked(pclp, this->frame_name_, true,
      {{"xyz", ignition::msgs::PointCloudPacked::Field::FLOAT32},
       {"intensity", ignition::msgs::PointCloudPacked::Field::FLOAT32}});

    pclp.set_height(pcl_msg_I.height);
    pclp.set_width(pcl_msg_I.width);
    pclp.set_is_bigendian(false);
    pclp.set_is_dense(true);
    pclp.mutable_data()->resize(pcl_msg_I.points.size()*pclp.point_step());

    // Declare iterators for each field present in the new Ignition PCL<XYZI>
    ignition::msgs::PointCloudPackedIterator<float> iter_i(pclp, "intensity");
    ignition::msgs::PointCloudPackedIterator<float> iter_x(pclp, "x");
    ignition::msgs::PointCloudPackedIterator<float> iter_y(pclp, "y");
    ignition::msgs::PointCloudPackedIterator<float> iter_z(pclp, "z");

    // Populate new IGN PCL with intensity values
    for(int counter = 0; counter <= pcl_msg_I.points.size() - 1; counter++){

      *iter_i = pcl_msg_I.points[counter].intensity;
      *iter_x = pcl_msg_I.points[counter].x;
      *iter_y = pcl_msg_I.points[counter].y;
      *iter_z = pcl_msg_I.points[counter].z;

      ++iter_i;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
    this->pcl_pub_I_.Publish(pclp);
    // end the clock
    clock_t end = clock();
    
    // Compute time between receiving a LaserScan message and publishing  PCLs with color and intensity fields
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

    //igndbg << "Elapsed_time: " << elapsed_secs << " sec, Freq:" << 1.0 / elapsed_secs << " Hz" << std::endl;
}


// Register this plugin
IGNITION_ADD_PLUGIN(MovaiLidar,
                    ignition::gazebo::System,
                    MovaiLidar::ISystemConfigure,
                    MovaiLidar::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(MovaiLidar,
                          "ignition::gazebo::systems::MovaiLidar")

