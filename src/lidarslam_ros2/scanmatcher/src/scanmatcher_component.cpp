#include "scanmatcher/scanmatcher_component.h"
#include <chrono>

using namespace std::chrono_literals;

namespace graphslam
{
ScanMatcherComponent::ScanMatcherComponent(const rclcpp::NodeOptions & options)
: Node("scan_matcher", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  listener_(tfbuffer_),
  broadcaster_(this)
{
  RCLCPP_INFO(get_logger(), "initialization start");
  double ndt_resolution;
  int ndt_num_threads;
  double gicp_corr_dist_threshold;

  declare_parameter("global_frame_id", "map");
  get_parameter("global_frame_id", global_frame_id_);
  declare_parameter("robot_frame_id", "base_link");
  get_parameter("robot_frame_id", robot_frame_id_);
  declare_parameter("odom_frame_id", "odom");
  get_parameter("odom_frame_id", odom_frame_id_);
  declare_parameter("registration_method", "NDT");
  get_parameter("registration_method", registration_method_);
  declare_parameter("ndt_resolution", 5.0);
  get_parameter("ndt_resolution", ndt_resolution);
  declare_parameter("ndt_num_threads", 0);
  get_parameter("ndt_num_threads", ndt_num_threads);
  declare_parameter("gicp_corr_dist_threshold", 5.0);
  get_parameter("gicp_corr_dist_threshold", gicp_corr_dist_threshold);
  declare_parameter("trans_for_mapupdate", 1.5);
  get_parameter("trans_for_mapupdate", trans_for_mapupdate_);
  declare_parameter("vg_size_for_input", 0.2);
  get_parameter("vg_size_for_input", vg_size_for_input_);
  declare_parameter("vg_size_for_map", 0.1);
  get_parameter("vg_size_for_map", vg_size_for_map_);
  declare_parameter("use_min_max_filter", false);
  get_parameter("use_min_max_filter", use_min_max_filter_);
  declare_parameter("scan_min_range", 0.1);
  get_parameter("scan_min_range", scan_min_range_);
  declare_parameter("scan_max_range", 100.0);
  get_parameter("scan_max_range", scan_max_range_);
  declare_parameter("scan_period", 0.1);
  get_parameter("scan_period", scan_period_);
  declare_parameter("map_publish_period", 15.0);
  get_parameter("map_publish_period", map_publish_period_);  
  declare_parameter("num_targeted_cloud", 10);
  get_parameter("num_targeted_cloud", num_targeted_cloud_);
  if (num_targeted_cloud_ < 1) {
    std::cout << "num_tareged_cloud should be positive" << std::endl;
    num_targeted_cloud_ = 1;
  }

  declare_parameter("initial_pose_x", 0.0);
  get_parameter("initial_pose_x", initial_pose_x_);
  declare_parameter("initial_pose_y", 0.0);
  get_parameter("initial_pose_y", initial_pose_y_);
  declare_parameter("initial_pose_z", 0.0);
  get_parameter("initial_pose_z", initial_pose_z_);
  declare_parameter("initial_pose_qx", 0.0);
  get_parameter("initial_pose_qx", initial_pose_qx_);
  declare_parameter("initial_pose_qy", 0.0);
  get_parameter("initial_pose_qy", initial_pose_qy_);
  declare_parameter("initial_pose_qz", 0.0);
  get_parameter("initial_pose_qz", initial_pose_qz_);
  declare_parameter("initial_pose_qw", 1.0);
  get_parameter("initial_pose_qw", initial_pose_qw_);

  declare_parameter("set_initial_pose", false);
  get_parameter("set_initial_pose", set_initial_pose_);
  declare_parameter("publish_tf", true);
  get_parameter("publish_tf", publish_tf_);
  declare_parameter("use_odom", false);
  get_parameter("use_odom", use_odom_);
  declare_parameter("use_imu", false);
  get_parameter("use_imu", use_imu_);
  declare_parameter("debug_flag", false);
  get_parameter("debug_flag", debug_flag_);

  std::cout << "registration_method:" << registration_method_ << std::endl;
  std::cout << "ndt_resolution[m]:" << ndt_resolution << std::endl;
  std::cout << "ndt_num_threads:" << ndt_num_threads << std::endl;
  std::cout << "gicp_corr_dist_threshold[m]:" << gicp_corr_dist_threshold << std::endl;
  std::cout << "trans_for_mapupdate[m]:" << trans_for_mapupdate_ << std::endl;
  std::cout << "vg_size_for_input[m]:" << vg_size_for_input_ << std::endl;
  std::cout << "vg_size_for_map[m]:" << vg_size_for_map_ << std::endl;
  std::cout << "use_min_max_filter:" << std::boolalpha << use_min_max_filter_ << std::endl;
  std::cout << "scan_min_range[m]:" << scan_min_range_ << std::endl;
  std::cout << "scan_max_range[m]:" << scan_max_range_ << std::endl;
  std::cout << "set_initial_pose:" << std::boolalpha << set_initial_pose_ << std::endl;
  std::cout << "publish_tf:" << std::boolalpha << publish_tf_ << std::endl;
  std::cout << "use_odom:" << std::boolalpha << use_odom_ << std::endl;
  std::cout << "use_imu:" << std::boolalpha << use_imu_ << std::endl;
  std::cout << "scan_period[sec]:" << scan_period_ << std::endl;
  std::cout << "debug_flag:" << std::boolalpha << debug_flag_ << std::endl;
  std::cout << "map_publish_period[sec]:" << map_publish_period_ << std::endl;
  std::cout << "num_targeted_cloud:" << num_targeted_cloud_ << std::endl;
  std::cout << "------------------" << std::endl;

  if (registration_method_ == "NDT") {

    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr
      ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setResolution(ndt_resolution);
    ndt->setTransformationEpsilon(0.01);
    // ndt_omp
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    if (ndt_num_threads > 0) {ndt->setNumThreads(ndt_num_threads);}

    registration_ = ndt;

  } else if (registration_method_ == "GICP") {
	  boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>
      gicp(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setMaxCorrespondenceDistance(gicp_corr_dist_threshold);
    gicp->setTransformationEpsilon(1e-8);
    registration_ = gicp;
  } else {
    RCLCPP_ERROR(get_logger(), "invalid registration method");
    exit(1);
  }

  map_array_msg_.header.frame_id = global_frame_id_;
  map_array_msg_.cloud_coordinate = map_array_msg_.LOCAL;

  path_.header.frame_id = global_frame_id_;

  lidar_undistortion_.setScanPeriod(scan_period_);

  initializePubSub();

  if (set_initial_pose_) {
    RCLCPP_INFO(get_logger(), "set initial pose");
    auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.position.x = initial_pose_x_;
    msg->pose.position.y = initial_pose_y_;
    msg->pose.position.z = initial_pose_z_;
    msg->pose.orientation.x = initial_pose_qx_;
    msg->pose.orientation.y = initial_pose_qy_;
    msg->pose.orientation.z = initial_pose_qz_;
    msg->pose.orientation.w = initial_pose_qw_;
    current_pose_stamped_ = *msg;
    pose_pub_->publish(current_pose_stamped_);
    initial_pose_received_ = true;

    path_.poses.push_back(*msg);
  }

  RCLCPP_INFO(get_logger(), "initialization end");
}

void ScanMatcherComponent::initializePubSub()
{
  RCLCPP_INFO(get_logger(), "initialize Publishers and Subscribers");
  // sub
  auto initial_pose_callback =
    [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
    {
      if (msg->header.frame_id != global_frame_id_) {
        RCLCPP_WARN(get_logger(), "This initial_pose is not in the global frame");
        return;
      }
      RCLCPP_INFO(get_logger(), "initial_pose is received");

      current_pose_stamped_ = *msg;
      previous_position_.x() = current_pose_stamped_.pose.position.x;
      previous_position_.y() = current_pose_stamped_.pose.position.y;
      previous_position_.z() = current_pose_stamped_.pose.position.z;
      initial_pose_received_ = true;

      pose_pub_->publish(current_pose_stamped_);
    };

  auto cloud_callback =
    [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
    {
      if (!initial_pose_received_)
      {
        RCLCPP_WARN(get_logger(), "initial_pose is not received");
        return;
      }

      sensor_msgs::msg::PointCloud2 transformed_msg;
      try {
        tf2::TimePoint time_point = tf2::TimePoint(
          std::chrono::seconds(msg->header.stamp.sec) +
          std::chrono::nanoseconds(msg->header.stamp.nanosec));
        const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
          robot_frame_id_, msg->header.frame_id, time_point);
        tf2::doTransform(*msg, transformed_msg, transform); // TODO:slow now(https://github.com/ros/geometry2/pull/432)
      } catch (tf2::TransformException & e) {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
        return;
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(transformed_msg, *tmp_ptr);

      if (use_imu_) {
        double scan_time = msg->header.stamp.sec +
          msg->header.stamp.nanosec * 1e-9;
        lidar_undistortion_.adjustDistortion(tmp_ptr, scan_time);
      }

      if (use_min_max_filter_) {
        double r;
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr2(new pcl::PointCloud<pcl::PointXYZI>());
        for (const auto & p : tmp_ptr->points) {
          r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
          if (scan_min_range_ < r && r < scan_max_range_) {tmp_ptr2->points.push_back(p);}
        }
        tmp_ptr = tmp_ptr2;
      }

      if (!initial_cloud_received_) {
        RCLCPP_INFO(get_logger(), "initial_cloud is received");
        initial_cloud_received_ = true;
        initializeMap(tmp_ptr, msg->header);
        last_map_time_ = clock_.now();
      }

      if (initial_cloud_received_) {receiveCloud(tmp_ptr, msg->header.stamp);}

    };

  auto imu_callback =
    [this](const typename sensor_msgs::msg::Imu::SharedPtr msg) -> void
    {
      if (initial_pose_received_) {receiveImu(*msg);}
    };

  initial_pose_sub_ =
    create_subscription<geometry_msgs::msg::PoseStamped>(
    "initial_pose", rclcpp::QoS(10), initial_pose_callback);

  imu_sub_ =
    create_subscription<sensor_msgs::msg::Imu>(
    "imu_transformed", rclcpp::SensorDataQoS(), imu_callback);

  input_cloud_sub_ =
    create_subscription<sensor_msgs::msg::PointCloud2>(
    "input_cloud", rclcpp::SensorDataQoS(), cloud_callback);

  // pub
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "current_pose",
    rclcpp::QoS(10));
  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("map", rclcpp::QoS(10));
  map_array_pub_ =
    create_publisher<lidarslam_msgs::msg::MapArray>(
    "map_array", rclcpp::QoS(
      rclcpp::KeepLast(
        1)).reliable());
  path_pub_ = create_publisher<nav_msgs::msg::Path>("path", rclcpp::QoS(10));
}

void ScanMatcherComponent::initializeMap(const pcl::PointCloud <pcl::PointXYZI>::Ptr & tmp_ptr, const std_msgs::msg::Header & header)
{
  RCLCPP_INFO(get_logger(), "create a first map");
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setLeafSize(vg_size_for_map_, vg_size_for_map_, vg_size_for_map_);
  voxel_grid.setInputCloud(tmp_ptr);
  voxel_grid.filter(*cloud_ptr);

  Eigen::Matrix4f sim_trans = getTransformation(current_pose_stamped_.pose);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(
    new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, sim_trans);
  registration_->setInputTarget(transformed_cloud_ptr);

  // map
  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*transformed_cloud_ptr, *map_msg_ptr);

  // map array
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(
    new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*cloud_ptr, *cloud_msg_ptr);
  lidarslam_msgs::msg::SubMap submap;
  submap.header = header;
  submap.distance = 0;
  submap.pose = current_pose_stamped_.pose;
  submap.cloud = *cloud_msg_ptr;
  map_array_msg_.header = header;
  map_array_msg_.submaps.push_back(submap);

  map_pub_->publish(submap.cloud);
  RCLCPP_INFO(get_logger(), "create a first map end");
}

void ScanMatcherComponent::receiveCloud(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_ptr,
  const rclcpp::Time stamp)
{
  if (mapping_flag_ && mapping_future_.valid()) {
    auto status = mapping_future_.wait_for(0s);
    if (status == std::future_status::ready) {
      if (is_map_updated_ == true) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr targeted_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(
            targeted_cloud_));
        if (registration_method_ == "NDT") {
          registration_->setInputTarget(targeted_cloud_ptr);
        } else {
          pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_targeted_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZI>());
          pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
          voxel_grid.setLeafSize(vg_size_for_input_, vg_size_for_input_, vg_size_for_input_);
          voxel_grid.setInputCloud(targeted_cloud_ptr);
          voxel_grid.filter(*filtered_targeted_cloud_ptr);
          registration_->setInputTarget(filtered_targeted_cloud_ptr);
        }
        is_map_updated_ = false;
      }
      mapping_flag_ = false;
      mapping_thread_.detach();
    }
  }
  RCLCPP_INFO(get_logger(), "map ready");
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setLeafSize(vg_size_for_input_, vg_size_for_input_, vg_size_for_input_);
  voxel_grid.setInputCloud(cloud_ptr);
  voxel_grid.filter(*filtered_cloud_ptr); 
  RCLCPP_INFO(get_logger(), "voxel filtered");

  registration_->setInputSource(filtered_cloud_ptr);
	

  Eigen::Matrix4f sim_trans = getTransformation(current_pose_stamped_.pose);

  if (use_odom_) {
    geometry_msgs::msg::TransformStamped odom_trans;
    try {
      odom_trans = tfbuffer_.lookupTransform(
        odom_frame_id_, robot_frame_id_, tf2_ros::fromMsg(
          stamp));
    } catch (tf2::TransformException & e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
    Eigen::Affine3d odom_affine = tf2::transformToEigen(odom_trans);
    Eigen::Matrix4f odom_mat = odom_affine.matrix().cast<float>();
    if (previous_odom_mat_ != Eigen::Matrix4f::Identity()) {
      sim_trans = sim_trans * previous_odom_mat_.inverse() * odom_mat;
    }
    previous_odom_mat_ = odom_mat;
  }

  
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  rclcpp::Clock system_clock;
  rclcpp::Time time_align_start = system_clock.now();
  registration_->align(*output_cloud, sim_trans);
  rclcpp::Time time_align_end = system_clock.now();

  Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();

  RCLCPP_INFO(get_logger(), "done align");

  publishMapAndPose(cloud_ptr, final_transformation, stamp);

  if (!debug_flag_) {return;}

  tf2::Quaternion quat_tf;
  double roll, pitch, yaw;
  tf2::fromMsg(current_pose_stamped_.pose.orientation, quat_tf);
  tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

  std::cout << "---------------------------------------------------------" << std::endl;
  std::cout << "nanoseconds: " << stamp.nanoseconds() << std::endl;
  std::cout << "trans: " << trans_ << std::endl;
  std::cout << "align time:" << time_align_end.seconds() - time_align_start.seconds() << "s" <<
    std::endl;
  std::cout << "number of filtered cloud points: " << filtered_cloud_ptr->size() << std::endl;
  std::cout << "initial transformation:" << std::endl;
  std::cout << sim_trans << std::endl;
  std::cout << "has converged: " << registration_->hasConverged() << std::endl;
  std::cout << "fitness score: " << registration_->getFitnessScore() << std::endl;
  std::cout << "final transformation:" << std::endl;
  std::cout << final_transformation << std::endl;
  std::cout << "rpy" << std::endl;
  std::cout << "roll:" << roll * 180 / M_PI << "," <<
    "pitch:" << pitch * 180 / M_PI << "," <<
    "yaw:" << yaw * 180 / M_PI << std::endl;
  int num_submaps = map_array_msg_.submaps.size();
  std::cout << "num_submaps:" << num_submaps << std::endl;
  std::cout << "moving distance:" << latest_distance_ << std::endl;
  std::cout << "---------------------------------------------------------" << std::endl;
}

void ScanMatcherComponent::publishMapAndPose(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & cloud_ptr,
  const Eigen::Matrix4f final_transformation, const rclcpp::Time stamp)
{

  Eigen::Vector3d position = final_transformation.block<3, 1>(0, 3).cast<double>();

  Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond quat_eig(rot_mat);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  if(publish_tf_){
    geometry_msgs::msg::TransformStamped base_to_map_msg;
    base_to_map_msg.header.stamp = stamp;
    base_to_map_msg.header.frame_id = global_frame_id_;
    base_to_map_msg.child_frame_id = robot_frame_id_;
    base_to_map_msg.transform.translation.x = position.x();
    base_to_map_msg.transform.translation.y = position.y();
    base_to_map_msg.transform.translation.z = position.z();
    base_to_map_msg.transform.rotation = quat_msg;

    if(use_odom_){
        geometry_msgs::msg::TransformStamped odom_to_map_msg;
        odom_to_map_msg = calculateMaptoOdomTransform(base_to_map_msg, stamp);
        broadcaster_.sendTransform(odom_to_map_msg);
    }
    else{
      broadcaster_.sendTransform(base_to_map_msg);
    }
  }

  current_pose_stamped_.header.stamp = stamp;
  current_pose_stamped_.pose.position.x = position.x();
  current_pose_stamped_.pose.position.y = position.y();
  current_pose_stamped_.pose.position.z = position.z();
  current_pose_stamped_.pose.orientation = quat_msg;
  pose_pub_->publish(current_pose_stamped_);

  path_.poses.push_back(current_pose_stamped_);
  path_pub_->publish(path_);

  trans_ = (position - previous_position_).norm();
  if (trans_ >= trans_for_mapupdate_ && !mapping_flag_) {
    geometry_msgs::msg::PoseStamped current_pose_stamped;
    current_pose_stamped = current_pose_stamped_;
    previous_position_ = position;
    mapping_task_ =
      std::packaged_task<void()>(
      std::bind(
        &ScanMatcherComponent::updateMap, this, cloud_ptr,
        final_transformation, current_pose_stamped));
    mapping_future_ = mapping_task_.get_future();
    mapping_thread_ = std::thread(std::move(std::ref(mapping_task_)));
    mapping_flag_ = true;
  }
}

void ScanMatcherComponent::updateMap(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_ptr,
  const Eigen::Matrix4f final_transformation,
  const geometry_msgs::msg::PoseStamped current_pose_stamped)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
  voxel_grid.setLeafSize(vg_size_for_map_, vg_size_for_map_, vg_size_for_map_);
  voxel_grid.setInputCloud(cloud_ptr);
  voxel_grid.filter(*filtered_cloud_ptr);
  RCLCPP_INFO(get_logger(), "map filter done");
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*filtered_cloud_ptr, *transformed_cloud_ptr, final_transformation);

  targeted_cloud_.clear();
  targeted_cloud_ += *transformed_cloud_ptr;
  int num_submaps = map_array_msg_.submaps.size();
  for (int i = 0; i < num_targeted_cloud_ - 1; i++) {
    if (num_submaps - 1 - i < 0) {continue;}
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(map_array_msg_.submaps[num_submaps - 1 - i].cloud, *tmp_ptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    Eigen::Affine3d submap_affine;
    tf2::fromMsg(map_array_msg_.submaps[num_submaps - 1 - i].pose, submap_affine);
    pcl::transformPointCloud(*tmp_ptr, *transformed_tmp_ptr, submap_affine.matrix());
    targeted_cloud_ += *transformed_tmp_ptr;
  }
  RCLCPP_INFO(get_logger(), "Add map done");
  /* map array */
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ptr(
    new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*filtered_cloud_ptr, *cloud_msg_ptr);

  /*
  lidarslam_msgs::msg::SubMap submap;
  submap.header.frame_id = global_frame_id_;
  submap.header.stamp = current_pose_stamped.header.stamp;
  latest_distance_ += trans_;
  submap.distance = latest_distance_;
  submap.pose = current_pose_stamped.pose;
  submap.cloud = *cloud_msg_ptr;
  submap.cloud.header.frame_id = global_frame_id_;
  map_array_msg_.header.stamp = current_pose_stamped.header.stamp;
  map_array_msg_.submaps.push_back(submap);
  map_array_pub_->publish(map_array_msg_);
  */
  
  RCLCPP_INFO(this->get_logger(), "Step 1: Creating SubMap msg");

lidarslam_msgs::msg::SubMap submap;

RCLCPP_INFO(this->get_logger(), "Step 2: Setting frame_id to %s", global_frame_id_.c_str());
submap.header.frame_id = global_frame_id_;

RCLCPP_INFO(this->get_logger(), "Step 3: Setting timestamp");
submap.header.stamp = current_pose_stamped.header.stamp;

RCLCPP_INFO(this->get_logger(), "Step 4: Updating latest_distance_ by trans_ = %.2f", trans_);
latest_distance_ += trans_;

RCLCPP_INFO(this->get_logger(), "Step 5: Setting submap distance = %.2f", latest_distance_);
submap.distance = latest_distance_;

RCLCPP_INFO(this->get_logger(), "Step 6: Assigning pose");
submap.pose = current_pose_stamped.pose;

const auto& pos = submap.pose.position;
const auto& ori = submap.pose.orientation;
RCLCPP_INFO(this->get_logger(),
  "Step 6.1: Pose Position = [%.2f, %.2f, %.2f], Orientation (quat) = [%.2f, %.2f, %.2f, %.2f]",
  pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w);

RCLCPP_INFO(this->get_logger(), "Step 7: Assigning cloud pointer");
submap.cloud = *cloud_msg_ptr;

RCLCPP_INFO(this->get_logger(), "Step 8: Setting cloud frame_id");
submap.cloud.header.frame_id = global_frame_id_;

RCLCPP_INFO(this->get_logger(), "Step 9: Updating map_array_msg_ timestamp");
map_array_msg_.header.stamp = current_pose_stamped.header.stamp;

RCLCPP_INFO(this->get_logger(), "Step 10: Pushing submap to map_array_msg_");
map_array_msg_.submaps.push_back(submap);

RCLCPP_INFO(this->get_logger(), "Step 11: Publishing map_array_msg_");

if (!map_array_pub_) {
  RCLCPP_ERROR(this->get_logger(), "map_array_pub_ is null! Did you forget to create the publisher?");
  return;
}

if (map_array_msg_.submaps.empty()) {
  RCLCPP_ERROR(this->get_logger(), "map_array_msg_ has no submaps!");
  return;
}

const auto& last_submap = map_array_msg_.submaps.back();
const auto& last_pose = last_submap.pose;
const auto& last_cloud = last_submap.cloud;

RCLCPP_INFO(this->get_logger(),
  "=== DEBUG SubMap ===\n"
  "Header: frame_id = %s, stamp = %.3f\n"
  "Pose: Position = [%.2f, %.2f, %.2f], Orientation = [%.2f, %.2f, %.2f, %.2f]\n"
  "Cloud: width = %u, height = %u, point_step = %u, row_step = %u, data size = %zu, is_dense = %d\n"
  "Fields: %zu, Frame ID = %s",
  last_submap.header.frame_id.c_str(),
  rclcpp::Time(last_submap.header.stamp).seconds(),
  last_pose.position.x, last_pose.position.y, last_pose.position.z,
  last_pose.orientation.x, last_pose.orientation.y, last_pose.orientation.z, last_pose.orientation.w,
  last_cloud.width, last_cloud.height,
  last_cloud.point_step, last_cloud.row_step,
  last_cloud.data.size(),
  last_cloud.is_dense,
  last_cloud.fields.size(),
  last_cloud.header.frame_id.c_str()
);
if (last_cloud.data.empty()) {
  RCLCPP_ERROR(this->get_logger(), "Last submap cloud has empty data!");
  return;
}

RCLCPP_INFO(this->get_logger(), "Step 11: Ready to publish map");
bool all_valid = true;
for (size_t i = 0; i < map_array_msg_.submaps.size(); ++i) {
  const auto& submap = map_array_msg_.submaps[i];

  RCLCPP_INFO(this->get_logger(),
    "Checking submap %zu: frame_id=%s cloud.width=%u cloud.height=%u cloud.data.size=%zu",
    i,
    submap.cloud.header.frame_id.c_str(),
    submap.cloud.width,
    submap.cloud.height,
    submap.cloud.data.size());

  if (submap.cloud.data.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Submap %zu has empty cloud data!", i);
    all_valid = false;
  }

  // Optional: check for NaN/Inf in pose
  if (!std::isfinite(submap.pose.position.x) ||
      !std::isfinite(submap.pose.position.y) ||
      !std::isfinite(submap.pose.position.z)) {
    RCLCPP_ERROR(this->get_logger(), "Submap %zu has non-finite position!", i);
    all_valid = false;
  }

  if (!std::isfinite(submap.pose.orientation.x) ||
      !std::isfinite(submap.pose.orientation.y) ||
      !std::isfinite(submap.pose.orientation.z) ||
      !std::isfinite(submap.pose.orientation.w)) {
    RCLCPP_ERROR(this->get_logger(), "Submap %zu has non-finite orientation!", i);
    all_valid = false;
  }
}

if (!all_valid) {
  RCLCPP_ERROR(this->get_logger(), "Validation failed. Aborting publish.");
  return;
}
try {
  map_array_pub_->publish(map_array_msg_);
} catch (const std::exception& e) {
  RCLCPP_ERROR(this->get_logger(), "Exception during publish: %s", e.what());
} catch (...) {
  RCLCPP_ERROR(this->get_logger(), "Unknown error during publish!");
}
RCLCPP_INFO(this->get_logger(),
  "Step 12: Publish complete — Total submaps: %zu, Last cloud size: %zu bytes",
  map_array_msg_.submaps.size(), submap.cloud.data.size());


  is_map_updated_ = true;
	
  rclcpp::Time map_time = clock_.now();
  double dt = map_time.seconds() - last_map_time_.seconds();
    RCLCPP_INFO(get_logger(), "map ready to pub ");
  if (dt > map_publish_period_) {
    publishMap(map_array_msg_, global_frame_id_);
    last_map_time_ = map_time;
  }
  RCLCPP_INFO(get_logger(), "map pub done ");
}

Eigen::Matrix4f ScanMatcherComponent::getTransformation(const geometry_msgs::msg::Pose pose)
{
  Eigen::Affine3d affine;
  tf2::fromMsg(pose, affine);
  Eigen::Matrix4f sim_trans = affine.matrix().cast<float>();
  return sim_trans;
}

void ScanMatcherComponent::receiveImu(const sensor_msgs::msg::Imu msg)
{
  if (!use_imu_) {return;}

  double roll, pitch, yaw;
  tf2::Quaternion orientation;
  tf2::fromMsg(msg.orientation, orientation);
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  float acc_x = static_cast<float>(msg.linear_acceleration.x) + sin(pitch) * 9.81;
  float acc_y = static_cast<float>(msg.linear_acceleration.y) - cos(pitch) * sin(roll) * 9.81;
  float acc_z = static_cast<float>(msg.linear_acceleration.z) - cos(pitch) * cos(roll) * 9.81;

  Eigen::Vector3f angular_velo{
    static_cast<float>(msg.angular_velocity.x),
    static_cast<float>(msg.angular_velocity.y),
    static_cast<float>(msg.angular_velocity.z)};
  Eigen::Vector3f acc{acc_x, acc_y, acc_z};
  Eigen::Quaternionf quat{
    static_cast<float>(msg.orientation.w),
    static_cast<float>(msg.orientation.x),
    static_cast<float>(msg.orientation.y),
    static_cast<float>(msg.orientation.z)};
  double imu_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;

  lidar_undistortion_.getImu(angular_velo, acc, quat, imu_time);

}

void ScanMatcherComponent::publishMap(const lidarslam_msgs::msg::MapArray & map_array_msg , const std::string & map_frame_id)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  for (auto & submap : map_array_msg.submaps) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_submap_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(submap.cloud, *submap_cloud_ptr);

      // Debug: Print submap.pose
  const auto & p = submap.pose.position;
  const auto & q = submap.pose.orientation;
  RCLCPP_INFO(rclcpp::get_logger("debug"), "Submap pose - Position: [%.2f, %.2f, %.2f], Orientation (quat): [%.2f, %.2f, %.2f, %.2f]",
              p.x, p.y, p.z, q.x, q.y, q.z, q.w);

    Eigen::Affine3d affine;
    tf2::fromMsg(submap.pose, affine);
    pcl::transformPointCloud(
      *submap_cloud_ptr, *transformed_submap_cloud_ptr,
      affine.matrix().cast<float>());

    *map_ptr += *transformed_submap_cloud_ptr;
     RCLCPP_INFO(rclcpp::get_logger("debug"), "Accumulated map size: %lu points", map_ptr->points.size());
  }
  RCLCPP_INFO(get_logger(), "publish a map, number of points in the map : %ld", map_ptr->size());

  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  map_msg_ptr->header.frame_id = map_frame_id;
  map_pub_->publish(*map_msg_ptr);
}

geometry_msgs::msg::TransformStamped ScanMatcherComponent::calculateMaptoOdomTransform(
  const geometry_msgs::msg::TransformStamped &base_to_map_msg,
  const rclcpp::Time stamp
)
{
  geometry_msgs::msg::TransformStamped odom_to_map_msg;
  try {
    geometry_msgs::msg::PoseStamped odom_to_map;
    geometry_msgs::msg::PoseStamped base_to_map;

    tf2::Transform odom_to_map_tf;
    tf2::Transform base_to_map_msg_tf;
    base_to_map.header.frame_id = robot_frame_id_;

    tf2::fromMsg(base_to_map_msg.transform, base_to_map_msg_tf);
    tf2::toMsg(base_to_map_msg_tf.inverse(), base_to_map.pose);
    tfbuffer_.transform(base_to_map, odom_to_map, odom_frame_id_);
    tf2::impl::Converter<true, false>::convert(odom_to_map.pose, odom_to_map_tf);
    tf2::impl::Converter<false, true>::convert(odom_to_map_tf.inverse(), odom_to_map_msg.transform);

    odom_to_map_msg.header.stamp = stamp;
    odom_to_map_msg.header.frame_id = global_frame_id_ ;
    odom_to_map_msg.child_frame_id = odom_frame_id_;
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "Transform from base_link to odom failed: %s", e.what());
  }
  return odom_to_map_msg;
}

} // namespace graphslam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(graphslam::ScanMatcherComponent)
