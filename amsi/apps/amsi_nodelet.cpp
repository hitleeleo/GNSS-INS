#include <mutex>
#include <memory>
#include <iostream>
#include <boost/circular_buffer.hpp>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>


#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>

#include <amsi/pose_estimator.hpp>
#include <amsi/gnss_tools.hpp>

#include <nmea_msgs/Sentence.h> // message from DGPS of University of California, Berkeley.
#include <sensor_msgs/NavSatFix.h>

#include <Eigen/Dense>
#include <math.h>

#include <fstream>
#include <iterator>
#include <string>
#include <vector>



namespace amsi {

class amsiNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  amsiNodelet() {
  }
  virtual ~amsiNodelet() {
  }


  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    processing_time.resize(16);
    initialize_params();

    use_imu = private_nh.param<bool>("use_imu", true);
    invert_imu = private_nh.param<bool>("invert_imu", false);
    if(use_imu) {
      NODELET_INFO("enable imu-based prediction");
      imu_sub = mt_nh.subscribe("/imu_raw", 256, &amsiNodelet::imu_callback, this);
    }

    use_dgps = private_nh.param<bool>("use_dgps", true);
    if(use_dgps) {
      NODELET_INFO("enable dgps-based correction");
      nmea_sub = nh.subscribe("/nmea_sentence", 32, &amsiNodelet::nmea_callback, this); // Berkeley's DGPS 
    }

    points_sub = mt_nh.subscribe("/points_raw", 5, &amsiNodelet::points_callback, this);
    globalmap_sub = nh.subscribe("/points_map", 1, &amsiNodelet::globalmap_callback, this);
    initialpose_sub = nh.subscribe("/initialpose", 8, &amsiNodelet::initialpose_callback, this);
    ndtpose_sub = nh.subscribe("/ndt_pose", 8, &amsiNodelet::ndtpose_callback, this);
    

    pose_pub = nh.advertise<nav_msgs::Odometry>("/gnss_ins", 5, false); // UKF-based GNSS/INS fusion
    enu_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/gnss_enu", 5, false);
    gnss_navsat_pose_pub = nh.advertise<sensor_msgs::NavSatFix>("/gnss_navsat", 5, false);
    gnss_ins_navsat_pose_pub = nh.advertise<sensor_msgs::NavSatFix>("/gnss_ins_navsat", 5, false);
    aligned_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_points", 5, false);
  }

private:
  void initialize_params() {
    // intialize scan matching method
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
    std::string ndt_neighbor_search_method = private_nh.param<std::string>("ndt_neighbor_search_method", "DIRECT7");

    double ndt_resolution = private_nh.param<double>("ndt_resolution", 1.0);

    specify_error_csv_directory = private_nh.param<std::string>("specify_error_csv_directory", "/home/wenws/amsipolyu/src/error.csv");

    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;

    pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
    ndt->setTransformationEpsilon(0.01);
    ndt->setResolution(ndt_resolution);
    if(ndt_neighbor_search_method == "DIRECT1") {
      NODELET_INFO("search_method DIRECT1 is selected");
      ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
    } else if(ndt_neighbor_search_method == "DIRECT7") {
      NODELET_INFO("search_method DIRECT7 is selected");
      ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    } else {
      if(ndt_neighbor_search_method == "KDTREE") {
        NODELET_INFO("search_method KDTREE is selected");
      } else {
        NODELET_WARN("invalid search method was given");
        NODELET_WARN("default method is selected (KDTREE)");
      }
      ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
    }
    // ndt->setStepSize(0.01);
    // ndt->setMaximumIterations(30);
    registration = ndt;

    // initialize pose estimator
    if(private_nh.param<bool>("specify_init_pose", true)) {
      NODELET_INFO("initialize pose estimator with specified parameters!!");
      pose_estimator.reset(new amsi::PoseEstimator(registration,
        ros::Time::now(),
        Eigen::Vector3f(private_nh.param<double>("init_pos_x", 0.0), private_nh.param<double>("init_pos_y", 0.0), private_nh.param<double>("init_pos_z", 0.0)),
        Eigen::Quaternionf(private_nh.param<double>("init_ori_w", 1.0), private_nh.param<double>("init_ori_x", 0.0), private_nh.param<double>("init_ori_y", 0.0), private_nh.param<double>("init_ori_z", 0.0)),
        private_nh.param<double>("cool_time_duration", 0.5)
      ));
    }
  }

private:
  /**
   * @brief callback for imu data
   * @param imu_msg
   */
  void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg) {
    std::lock_guard<std::mutex> lock(imu_data_mutex);
    imu_data.push_back(imu_msg);
  }

  /**
   * @brief callback for DGPS in nmea format
   * @param DGPS message
   */
  void nmea_callback(const nmea_msgs::SentenceConstPtr& nmea_msg) {
    std::lock_guard<std::mutex> lock(dgps_data_mutex);
    std::vector<std::string> str_vec_ptr;
    std::string token;
    std::stringstream ss(nmea_msg->sentence);
    bool find_SOL_COMPUTED =0; // the flag for Berkeley's DGPS
    while (getline(ss, token, ' '))
    {
      if(token == "SOL_COMPUTED") // solutions are computed 
      {
        // std::cout<<"message obtained"<<std::endl;
        find_SOL_COMPUTED = true;
      }
      if( find_SOL_COMPUTED ) // find flag SOL_COMPUTED
      {
        str_vec_ptr.push_back(token);
      }
    }

    if(find_SOL_COMPUTED)
    {
      sensor_msgs::NavSatFix navfix_ ;
      navfix_.header = nmea_msg->header;
      std::cout << std::setprecision(17);
      double lat = strtod((str_vec_ptr[2]).c_str(), NULL);
      double lon = strtod((str_vec_ptr[3]).c_str(), NULL);
      double alt = strtod((str_vec_ptr[4]).c_str(), NULL);
      std::cout << std::setprecision(17);

      navfix_.latitude = lat;
      navfix_.longitude = lon;
      navfix_.altitude = alt;
      gnss_navsat_pose_pub.publish(navfix_); // gnss standalone 
      if(ini_navf.latitude == NULL)
      {
        ini_navf = navfix_;
        std::cout<<"ini_navf.header  -> "<<ini_navf.header<<std::endl;
        originllh.resize(3, 1);
        originllh(0) = navfix_.longitude;
        originllh(1) = navfix_.latitude;
        originllh(2) = navfix_.altitude;
      }
      dgps_data.push_back(navfix_);
      Eigen::MatrixXd curLLh; // 
      curLLh.resize(3, 1);
      curLLh(0) = navfix_.longitude;
      curLLh(1) = navfix_.latitude;
      curLLh(2) = navfix_.altitude;

      Eigen::MatrixXd ecef; // 
      ecef.resize(3, 1);
      ecef = gnss_tools_.llh2ecef(curLLh);
      ENU_.header = navfix_.header;
      Eigen::MatrixXd eigenENU;; // 
      eigenENU.resize(3, 1);
      eigenENU = gnss_tools_.ecef2enu(originllh,ecef);

      // trans and rotation
      double prex_ = eigenENU(0);
      double prey_ = eigenENU(1);
      double theta = (68.5 )*( 3.141592 / 180.0 ); //
      eigenENU(0) = prex_ * cos(theta) - prey_ * sin(theta) ;
      eigenENU(1) = prex_ * sin(theta) + prey_ * cos(theta) ; 

      ENU_.pose.position.x = 1 * eigenENU(1);
      ENU_.pose.position.y = -1 * eigenENU(0);
      ENU_.pose.position.z = eigenENU(2);
      ENU_.pose.orientation.x = 0.23;
      ENU_.pose.orientation.y = 0.25;
      ENU_.pose.orientation.z = 0.12;
      ENU_.pose.orientation.w = 0.5;
      enu_pose_pub.publish(ENU_); // publish dgps in enu
      dgpsENU_data.push_back(ENU_);
      error_.push_back(cal_error(ENU_,ndt_pose,0));

      // std::ofstream output_file("/home/wenws/amsipolyu/src/error.csv");
      // std::ostream_iterator<std::string> output_iterator(output_file, "\n");
      // std::copy(error_.begin(), error_.end(), output_iterator);

      std::ofstream myfile(specify_error_csv_directory);
      int vsize = error_.size();
      for (int n=0; n<vsize; n++)
      {
          myfile << error_[n] << std::endl;
      }

      std::cout << std::setprecision(3);
      auto cur_tim = ros::WallTime::now();

      ecef = gnss_tools_.enu2ecef(originllh,eigenENU);
      curLLh = gnss_tools_.ecef2llh(ecef);
      navfix_.longitude = curLLh(0);
      navfix_.latitude  = curLLh(1);
      navfix_.altitude  = curLLh(2);
      gnss_ins_navsat_pose_pub.publish(navfix_); // ukf-based 

      std::cout<<"  E: "<<float(eigenENU(0))<<"  N: "<<float(eigenENU(1))<<"  U: "<<float(eigenENU(2))<<std::endl;
    }
  }

/**
   * @brief calcualte error
   * @input /gnss_enu (Ground truth) /ndt_pose (estimated pose)
   * @output /float error
   */
  float cal_error(geometry_msgs::PoseStamped ENU_,geometry_msgs::PoseStamped ndt_pose,bool D3) // dgps pose in ENU
  {
    float error_x = ENU_.pose.position.x - ndt_pose.pose.position.x;
    float error_y = ENU_.pose.position.y - ndt_pose.pose.position.y;
    float error_z = ENU_.pose.position.z - ndt_pose.pose.position.z;
    if(!D3) error_z = 0; //not 3D, but 2D
    float error = sqrt(error_x * error_x + error_y * error_y + error_z * error_z);
    return error;
  }

  /**
   * @brief callback for point cloud data
   * @param points_msg
   */
  void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);
    if(!pose_estimator) {
      NODELET_ERROR("waiting for initial pose input!!");
      return;
    }

    // if(!globalmap) {
    //   NODELET_ERROR("globalmap has not been received!!");
    //   return;
    // }

    const auto& stamp = points_msg->header.stamp;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *cloud);

    if(cloud->empty()) {
      NODELET_ERROR("cloud is empty!!");
      return;
    }

    auto filtered = downsample(cloud);

    // predict
    if(!use_imu) {
      pose_estimator->predict(stamp, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()); // no imu, constant vel model
      std::cout<<"const velocity prediction"<<std::endl;
    } else {
      std::lock_guard<std::mutex> lock(imu_data_mutex);
      auto imu_iter = imu_data.begin();
      for(imu_iter; imu_iter != imu_data.end(); imu_iter++) {
        if(stamp < (*imu_iter)->header.stamp) {
          break;
        }
        const auto& acc = (*imu_iter)->linear_acceleration;
        const auto& gyro = (*imu_iter)->angular_velocity;
        double gyro_sign = invert_imu ? -1.0 : 1.0;
        pose_estimator->predict((*imu_iter)->header.stamp, Eigen::Vector3f(acc.x, acc.y, acc.z), gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
      }
      imu_data.erase(imu_data.begin(), imu_iter);
    }

    // correct
    auto t1 = ros::WallTime::now();
    auto aligned = pose_estimator->correct(filtered,ENU_);
    auto t2 = ros::WallTime::now();

    processing_time.push_back((t2 - t1).toSec());
    double avg_processing_time = std::accumulate(processing_time.begin(), processing_time.end(), 0.0) / processing_time.size();
    // NODELET_INFO_STREAM("processing_time: " << avg_processing_time * 1000.0 << "[msec]");

    if(aligned_pub.getNumSubscribers()) {
      aligned->header.frame_id = "map";
      aligned->header.stamp = cloud->header.stamp;
      aligned_pub.publish(aligned);
    }

    publish_odometry(points_msg->header.stamp, pose_estimator->matrix());
  }

  /**
   * @brief callback for globalmap input
   * @param points_msg
   */
  void globalmap_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    NODELET_INFO("globalmap received!");
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *cloud);
    globalmap = cloud;

    registration->setInputTarget(globalmap);
  }

  /**
   * @brief callback for initial pose input ("2D Pose Estimate" on rviz)
   * @param pose_msg
   */
  void initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg) {
    NODELET_INFO("initial pose received!!");
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    const auto& p = pose_msg->pose.pose.position;
    const auto& q = pose_msg->pose.pose.orientation;
    pose_estimator.reset(
          new amsi::PoseEstimator(
            registration,
            ros::Time::now(),
            Eigen::Vector3f(p.x, p.y, p.z),
            Eigen::Quaternionf(q.w, q.x, q.y, q.z),
            private_nh.param<double>("cool_time_duration", 0.5))
    );
  }

  /**
   * @brief callback for ndt pose input ("3D Pose Estimate" from Autoware)
   * @param pose_msg
   */
  void ndtpose_callback(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
    NODELET_INFO("ndt pose received!!");
    ndt_pose = *pose_msg;
  }

  /**
   * @brief downsampling
   * @param cloud   input cloud
   * @return downsampled cloud
   */
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const ros::Time& stamp, const Eigen::Matrix4f& pose) {
    // broadcast the transform over tf
    geometry_msgs::TransformStamped odom_trans = matrix2transform(stamp, pose, "map", "velodyne");
    // pose_broadcaster.sendTransform(odom_trans);

    // publish the transform
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";

    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);
    odom.pose.pose.orientation = odom_trans.transform.rotation;

    odom.child_frame_id = "velodyne";
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    pose_pub.publish(odom);
  }

  /**
   * @brief convert a Eigen::Matrix to TransformedStamped
   * @param stamp           timestamp
   * @param pose            pose matrix
   * @param frame_id        frame_id
   * @param child_frame_id  child_frame_id
   * @return transform
   */
  geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id) {
    Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
    quat.normalize();
    geometry_msgs::Quaternion odom_quat;
    odom_quat.w = quat.w();
    odom_quat.x = quat.x();
    odom_quat.y = quat.y();
    odom_quat.z = quat.z();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;

    odom_trans.transform.translation.x = pose(0, 3);
    odom_trans.transform.translation.y = pose(1, 3);
    odom_trans.transform.translation.z = pose(2, 3);
    odom_trans.transform.rotation = odom_quat;

    return odom_trans;
  }

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  bool use_imu;
  bool invert_imu;
  bool use_dgps;

  ros::Subscriber imu_sub;
  ros::Subscriber points_sub;
  ros::Subscriber globalmap_sub;
  ros::Subscriber initialpose_sub;
  ros::Subscriber ndtpose_sub; // subscribe /ndt_pose from Autoware
  ros::Subscriber nmea_sub; // nmea sentence from DGPS

  ros::Publisher pose_pub;
  ros::Publisher enu_pose_pub; // enu publisher
  ros::Publisher gnss_navsat_pose_pub; // navsat 
  ros::Publisher gnss_ins_navsat_pose_pub; // navsat
  ros::Publisher aligned_pub;
  tf::TransformBroadcaster pose_broadcaster;

  // imu input buffer
  std::mutex imu_data_mutex;
  std::mutex dgps_data_mutex;
  std::vector<sensor_msgs::ImuConstPtr> imu_data;
  std::vector<sensor_msgs::NavSatFix> dgps_data;
  geometry_msgs::PoseStamped ENU_; // dgps pose in ENU
  std::vector<geometry_msgs::PoseStamped> dgpsENU_data;

  std::string specify_error_csv_directory;
  std::vector<float> error_;
  
  sensor_msgs::NavSatFix ini_navf ; // initial sensor msg
  Eigen::MatrixXd originllh; // origin llh
  

  // ndt pose from Autoware
  geometry_msgs::PoseStamped ndt_pose;

  // globalmap and registration method
  pcl::PointCloud<PointT>::Ptr globalmap;
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;

  // pose estimator
  std::mutex pose_estimator_mutex;
  std::unique_ptr<amsi::PoseEstimator> pose_estimator;

  // gnss_tools
  GNSS_Tools gnss_tools_;

  // processing time buffer
  boost::circular_buffer<double> processing_time;
};

}


PLUGINLIB_EXPORT_CLASS(amsi::amsiNodelet, nodelet::Nodelet)
