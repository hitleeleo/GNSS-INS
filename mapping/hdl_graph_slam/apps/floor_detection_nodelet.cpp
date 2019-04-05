#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <ros/time.h>
#include <pcl_ros/point_cloud.h>

#include <std_msgs/Time.h> 
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <hdl_graph_slam/FloorCoeffs.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <visualization_msgs/MarkerArray.h>

#include <nmea_msgs/Sentence.h> // message from DGPS of University of California, Berkeley.


using namespace std;
std::vector<double> CSV_GDOP; // save GDOP
std::vector<double> CSV_meanEle; // save mean elevation angle
std::vector<float> allMeanEle;
std::vector<float> num_points;
std::vector<float> GPSTow;
std::vector<float> allGDOP;
std::vector<float> allMeanDis;
std::vector<double> alllon;
std::vector<double> alllat;
std::vector<double> allalt;

namespace hdl_graph_slam {

class FloorDetectionNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FloorDetectionNodelet() {}
  virtual ~FloorDetectionNodelet() {}

  virtual void onInit() {
    NODELET_DEBUG("initializing floor_detection_nodelet...");
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();

    points_sub = nh.subscribe("/filtered_points", 256, &FloorDetectionNodelet::cloud_callback, this);
    floor_pub = nh.advertise<hdl_graph_slam::FloorCoeffs>("/floor_detection/floor_coeffs", 32);
    nmea_sub = nh.subscribe("/nmea_sentence", 32, &FloorDetectionNodelet::nmea_callback, this); // Berkeley's DGPS 

    read_until_pub = nh.advertise<std_msgs::Header>("/floor_detection/read_until", 32);
    floor_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/floor_detection/floor_filtered_points", 32);
    floor_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/floor_detection/floor_points", 32);

    mask_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/mask/points", 32); // mask points for traffic level evaluation
    pub_debug_marker_ = nh.advertise<visualization_msgs::MarkerArray>("debug_marker", 1, true);
    uncertainty_pub = nh.advertise<geometry_msgs::Point>("/lidarUncertainty", 1, true);
  }

public:
  typedef struct // single grid: determined in ENU coordiante system
  {
    double azimuth;
    double elevation;
    double x;
    double y;
    double z;
    double xyDis;
    //
  }maskPoint;

private:
  /**
   * @brief initialize parameters
   */
  void initialize_params() {
    tilt_deg = private_nh.param<double>("tilt_deg", 0.0);                          // approximate sensor tilt angle [deg]
    sensor_height = private_nh.param<double>("sensor_height", 2.0);                // approximate sensor height [m]
    height_clip_range= private_nh.param<double>("height_clip_range", 1.0);         // points with heights in [sensor_height - height_clip_range, sensor_height + height_clip_range] will be used for floor detection
    floor_pts_thresh = private_nh.param<int>("floor_pts_thresh", 512);             // minimum number of support points of RANSAC to accept a detected floor plane
    floor_normal_thresh = private_nh.param<double>("floor_normal_thresh", 10.0);   // verticality check thresold for the detected floor plane [deg]
    use_normal_filtering = private_nh.param<bool>("use_normal_filtering", true);   // if true, points with "non-"vertical normals will be filtered before RANSAC
    normal_filter_thresh = private_nh.param<double>("normal_filter_thresh", 20.0); // "non-"verticality check threshold [deg]
  }

  /**
   * @brief callback for point clouds
   * @param cloud_msg  point cloud msg
   */
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if(cloud->empty()) {
      return;
    }

    // floor detection
    boost::optional<Eigen::Vector4f> floor = detect(cloud);

    // publish the detected floor coefficients
    hdl_graph_slam::FloorCoeffs coeffs;
    coeffs.header = cloud_msg->header;
    if(floor) {
      coeffs.coeffs.resize(4);
      for(int i=0; i<4; i++) {
        coeffs.coeffs[i] = (*floor)[i];
      }
    }

    floor_pub.publish(coeffs);

    // for offline estimation
    std_msgs::HeaderPtr read_until(new std_msgs::Header());
    read_until->frame_id = "/velodyne_points";
    read_until->stamp = cloud_msg->header.stamp + ros::Duration(1, 0);
    read_until_pub.publish(read_until);

    read_until->frame_id = "/filtered_points";
    read_until_pub.publish(read_until);
  }

  /**
   * @brief detect the floor plane from a point cloud
   * @param cloud  input cloud
   * @return detected floor plane coefficients
   */
  boost::optional<Eigen::Vector4f> detect(const pcl::PointCloud<PointT>::Ptr& cloud) const {
    // compensate the tilt rotation
    Eigen::Matrix4f tilt_matrix = Eigen::Matrix4f::Identity();
    tilt_matrix.topLeftCorner(3, 3) = Eigen::AngleAxisf(tilt_deg * M_PI / 180.0f, Eigen::Vector3f::UnitY()).toRotationMatrix();

    // filtering before RANSAC (height and normal filtering)
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cloud, *filtered, tilt_matrix);
    
    filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height + height_clip_range), false);
    filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height - height_clip_range), true);

    if(use_normal_filtering) {
      filtered = normal_filtering(filtered);
      
      // const clock_t begin_time = clock();
      mask_points_filter(cloud);
      // std::cout << "   this mask_points_filter used  time -> " << float(clock() - begin_time) / CLOCKS_PER_SEC << "\n\n";
    }

    pcl::transformPointCloud(*filtered, *filtered, static_cast<Eigen::Matrix4f>(tilt_matrix.inverse()));

    if(floor_filtered_pub.getNumSubscribers()) {
      filtered->header = cloud->header;
      floor_filtered_pub.publish(filtered);
    }

    // too few points for RANSAC
    if(filtered->size() < floor_pts_thresh) {
      return boost::none;
    }

    // RANSAC
    pcl::SampleConsensusModelPlane<PointT>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointT>(filtered));
    pcl::RandomSampleConsensus<PointT> ransac(model_p);
    ransac.setDistanceThreshold(0.1);
    ransac.computeModel();

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    ransac.getInliers(inliers->indices);

    // too few inliers
    if(inliers->indices.size() < floor_pts_thresh) {
      return boost::none;
    }

    // verticality check of the detected floor's normal
    Eigen::Vector4f reference = tilt_matrix.inverse() * Eigen::Vector4f::UnitZ();

    Eigen::VectorXf coeffs;
    ransac.getModelCoefficients(coeffs);

    double dot = coeffs.head<3>().dot(reference.head<3>());
    if(std::abs(dot) < std::cos(floor_normal_thresh * M_PI / 180.0)) {
      // the normal is not vertical
      return boost::none;
    }

    // make the normal upward
    if(coeffs.head<3>().dot(Eigen::Vector3f::UnitZ()) < 0.0f) {
      coeffs *= -1.0f;
    }

    if(floor_points_pub.getNumSubscribers()) {
      pcl::PointCloud<PointT>::Ptr inlier_cloud(new pcl::PointCloud<PointT>);
      pcl::ExtractIndices<PointT> extract;
      extract.setInputCloud(filtered);
      extract.setIndices(inliers);
      extract.filter(*inlier_cloud);
      inlier_cloud->header = cloud->header;

      floor_points_pub.publish(inlier_cloud);
    }

    return Eigen::Vector4f(coeffs);
  }

  /**
   * @brief plane_clip
   * @param src_cloud
   * @param plane
   * @param negative
   * @return
   */
  pcl::PointCloud<PointT>::Ptr plane_clip(const pcl::PointCloud<PointT>::Ptr& src_cloud, const Eigen::Vector4f& plane, bool negative) const {
    pcl::PlaneClipper3D<PointT> clipper(plane);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);

    clipper.clipPointCloud3D(*src_cloud, indices->indices);

    pcl::PointCloud<PointT>::Ptr dst_cloud(new pcl::PointCloud<PointT>);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(src_cloud);
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.filter(*dst_cloud);

    return dst_cloud;
  }

  /**
   * @brief filter points with non-vertical normals
   * @param cloud  input cloud
   * @return filtered cloud
   */
  pcl::PointCloud<PointT>::Ptr normal_filtering(const pcl::PointCloud<PointT>::Ptr& cloud) const {
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(10);
    ne.setViewPoint(0.0f, 0.0f, sensor_height);
    ne.compute(*normals);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);

    filtered->reserve(cloud->size());

    for (int i = 0; i < cloud->size(); i++) {
      float dot = normals->at(i).getNormalVector3fMap().normalized().dot(Eigen::Vector3f::UnitZ());
      if (std::abs(dot) > std::cos(normal_filter_thresh * M_PI / 180.0)) {
        filtered->push_back(cloud->at(i));
      }
    }

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    return filtered;
  }

  /* A utility function to calculate area of  
   triangle formed by (x1, y1), (x2, y2) and 
  (x3, y3) */
  float area(double x1, double y1, double x2, double y2, 
                              double x3, double y3) 
  const { 
      return abs((x1 * (y2 - y3) + x2 * (y3 - y1) +  
                  x3 * (y1 - y2)) / 2.0); 
  } 
    
  /* A function to check whether point P(x, y)  
     lies inside the rectangle formed by A(x1, y1),  
     B(x2, y2), C(x3, y3) and D(x4, y4) */
  bool check(double x1, double y1, double x2, double y2, double x3,  
               double y3, double x4, double y4, double x, double y) 
  const { 
      /* Calculate area of rectangle ABCD */
      float A = area(x1, y1, x2, y2, x3, y3) +  
                area(x1, y1, x4, y4, x3, y3); 
    
      /* Calculate area of triangle PAB */
      float A1 = area(x, y, x1, y1, x2, y2); 
    
      /* Calculate area of triangle PBC */
      float A2 = area(x, y, x2, y2, x3, y3); 
    
      /* Calculate area of triangle PCD */
      float A3 = area(x, y, x3, y3, x4, y4); 
    
      /* Calculate area of triangle PAD */
      float A4 = area(x, y, x1, y1, x4, y4); 
    
      /* Check if sum of A1, A2, A3 and A4  
         is same as A */
      return (A == A1 + A2 + A3 + A4); 
  } 


/**
   * @brief callback for DGPS in nmea format
   * @param DGPS message
   */
  void nmea_callback(const nmea_msgs::SentenceConstPtr& nmea_msg) {
    std::vector<std::string> str_vec_ptr,str_vec_ptr2;
    std::string token;
    std::stringstream ss(nmea_msg->sentence);
    std::stringstream ss_(nmea_msg->sentence);
    bool find_BestPos =0; // the flag for Berkeley's DGPS
    bool find_SOL_COMPUTED =0; // the flag for Berkeley's DGPS
    while (getline(ss, token, ' '))
    {
      if(token == "FINESTEERING") // solutions are computed 
      {
             std::cout<<"message obtained"<<std::endl;

        // std::cout<<"message obtained"<<std::endl;
        find_BestPos = true;
      }
      if( find_BestPos ) // find flag find_BestPos
      {
        str_vec_ptr.push_back(token);
      }
    }

    if(find_BestPos)
    {
      double lat = strtod((str_vec_ptr[2]).c_str(), NULL);
      nmeaDGPSTime = lat;
      // std::cout<<"message obtained at time -> "<<nmeaDGPSTime <<std::endl;

    }

    // obtain groudn truth from DGPS
    while (getline(ss_, token, ' '))
    {
      if(token == "SOL_COMPUTED") // solutions are computed 
      {
        std::cout<<"message SOL_COMPUTED obtained"<<std::endl;
        find_SOL_COMPUTED = true;
      }

      if( find_SOL_COMPUTED ) // find flag SOL_COMPUTED
      {
        str_vec_ptr2.push_back(token);
      }
    }
    if(find_SOL_COMPUTED)
    {
      std::cout << std::setprecision(17);
      double lat = strtod((str_vec_ptr2[2]).c_str(), NULL);
      double lon = strtod((str_vec_ptr2[3]).c_str(), NULL);
      double alt = strtod((str_vec_ptr2[4]).c_str(), NULL);
      nmeaLon = lon ;
      nmeaLat = lat ;
      nmeaAlt =alt ;
      std::cout << std::setprecision(17);

      std::cout<<"  lat: "<<lat<<"  lon: "<<lon<<"  alt: "<<alt<<std::endl;
    }

  }



/**
   * @brief filter points of mask points
   * @param cloud  input cloud
   * @return filtered cloud
   */
  void mask_points_filter(const pcl::PointCloud<PointT>::Ptr& cloud) const {

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloudinitial(new pcl::PointCloud<PointT>);
    cloudinitial =cloud;
    filtered->header = cloud->header;
    filtered->reserve(cloud->size());

    std::vector<maskPoint> maskPoints; // 360 mask points 
    std::vector<maskPoint> maskPoints_; // used mask points (points with low elevation not used)
    maskPoint maskPoint_;
    geometry_msgs::Point uncertainty_;
    float unZeroNum =0; // number of un zero elevation angle 
    float maskEleThres =5;
    uncertainty_.x = 0.0;
    float meanDistance = 0;

    Eigen::MatrixXd G_Matrix; // G matrix for GDOP Calculation

    // variable to save R, azimuth and elevation angle
    std::vector<float> CSV_R; // save R
    std::vector<float> CSV_azimuth; // save azimuth angle
    std::vector<float> CSV_elevation; // save elevation angle

    for(int i = 0; i<361; i++)
    {
      maskPoints.push_back(maskPoint_);
    }
    // std::cout<<"atan2(cloud->at(i).z, xyDis)  "<<atan2(2,1) * 180/M_PI <<std::endl;

    // obtain rectangle
    visualization_msgs::MarkerArray markers;
    vector<PointT> rectangle_p;
    float longitudeL=25;
    float lateralLeft=5.5;
    float lateralRight=6.2;
    PointT p1;
    p1.x = longitudeL; 
    p1.y = lateralLeft; 
    rectangle_p.push_back(p1); 
    p1.x = -longitudeL; 
    p1.y = lateralLeft; 
    rectangle_p.push_back(p1); 
    p1.x = -longitudeL; 
    p1.y = -lateralRight; 
    rectangle_p.push_back(p1); 
    p1.x = longitudeL; 
    p1.y = -lateralRight; 
    rectangle_p.push_back(p1); 
    for (int m = 0; m < rectangle_p.size(); m++) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "velodyne";
      // marker.header.stamp = filtered->header.stamp;
      marker.ns = "Rays";
      marker.id = markers.markers.size();
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = 0;
      marker.pose.position.x = 0.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
      marker.lifetime = ros::Duration(0.2);
      marker.frame_locked = true;
      marker.points.resize(2);
      marker.points[0].x = rectangle_p[m].x;
      marker.points[0].y = rectangle_p[m].y;
      marker.points[0].z = 0;
      if(m ==3)
      {
        marker.points[1].x = rectangle_p[0].x;
        marker.points[1].y = rectangle_p[0].y;
        marker.points[1].z = 0;
      }
      else
      {
        marker.points[1].x = rectangle_p[m+1].x;
        marker.points[1].y = rectangle_p[m+1].y;
        marker.points[1].z = 0;
      }
      
      // marker.points[0].x = 0;
      // marker.points[0].y = 0;
      // marker.points[0].z = 0;
      // marker.points[1].x = 10;
      // marker.points[1].y = 5;
      // marker.points[1].z = 0;
      marker.colors.resize(2);
      marker.colors[0].a = 1;
      marker.colors[0].r = 0.0;
      marker.colors[0].g = 1.0;
      marker.colors[0].b = 0.0;

      marker.colors[1].a = 1;
      marker.colors[1].r = 0.0;
      marker.colors[1].g = 1.0;
      marker.colors[1].b = 0.0;

      markers.markers.push_back(marker); // rectangle
    }

    for (int i = 0; i < cloud->size(); i++) {
      bool inside_ = check(rectangle_p[0].x,rectangle_p[0].y,rectangle_p[1].x,rectangle_p[1].y,rectangle_p[2].x,rectangle_p[2].y,rectangle_p[3].x,rectangle_p[3].y,cloud->at(i).x,cloud->at(i).y);
      // bool inside_ = check(0, 10, 10, 0, 0, -10, -10, 0, 10, 15);
      if(inside_)
      {
        cloud->at(i).z = cloud->at(i).z + sensor_height; // move the points
        double xyDis = sqrt(cloud->at(i).x * cloud->at(i).x + cloud->at(i).y * cloud->at(i).y);
        double xyAzimuth = atan2(cloud->at(i).y, cloud->at(i).x) * 180/M_PI ;
        if(xyAzimuth <0) xyAzimuth = xyAzimuth +360;
        double eleAngle = atan2(cloud->at(i).z, xyDis) * 180/M_PI;
        if(eleAngle>maskEleThres) eleAngle = atan2(2.51, xyDis) * 180/M_PI; // 2+2.11 is height of truck
        // std::cout<<"xyAzimuth->  "<<xyAzimuth <<"eleAngle->  "<<eleAngle <<std::endl;
        if( (maskPoints[(int)xyAzimuth].elevation < eleAngle) &&(xyDis > 2))
          {
            maskPoints[(int)xyAzimuth].azimuth = xyAzimuth;
            
            maskPoints[(int)xyAzimuth].elevation = eleAngle;
            maskPoints[(int)xyAzimuth].x = cloud->at(i).x;
            maskPoints[(int)xyAzimuth].y = cloud->at(i).y;
            maskPoints[(int)xyAzimuth].z = cloud->at(i).z;
            maskPoints[(int)xyAzimuth].xyDis = xyDis;
            // std::cout<<"cloud->at(i).z->  "<<cloud->at(i).z <<std::endl;
          }
          // filtered->push_back(cloud->at(i));
      }
    }

    for (int k = 0; k < maskPoints.size(); k++) {
      PointT dynamicP;
      dynamicP.x = maskPoints[k].x;
      dynamicP.y = maskPoints[k].y;
      dynamicP.z = maskPoints[k].z - sensor_height;
      if(maskPoints[k].elevation>maskEleThres)
      {
        // std::cout<<"maskPoints[k].elevation->  "<<maskPoints[k].elevation <<std::endl;
        unZeroNum ++;
        uncertainty_.x = uncertainty_.x +maskPoints[k].elevation;
        meanDistance = meanDistance + maskPoints[k].xyDis;
        maskPoints_.push_back(maskPoints[k]);
        // std::cout<<"uncertainty_.x->  "<<uncertainty_.x <<"maskPoints[k].elevation->  "<<maskPoints[k].elevation <<std::endl;
      }
      if(maskPoints[k].elevation > maskEleThres)
      {
        filtered->push_back(dynamicP);
        // visaulization 
        visualization_msgs::Marker marker;
        marker.header.frame_id = "velodyne";
        // marker.header.stamp = filtered->header.stamp;
        marker.ns = "Rays";
        marker.id = markers.markers.size();
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = 0;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1; // 0.1
        marker.lifetime = ros::Duration(0.2);
        marker.frame_locked = true;
        marker.points.resize(2);
        marker.points[0].x = 0;
        marker.points[0].y = 0;
        marker.points[0].z = -1* sensor_height;
        marker.points[1].x = dynamicP.x;
        marker.points[1].y = dynamicP.y;
        marker.points[1].z = dynamicP.z;
        marker.colors.resize(2);
        marker.colors[0].a = 0.3;
        marker.colors[0].r = 1.0;
        marker.colors[0].g = 0.0;
        marker.colors[0].b = 0.0;

        marker.colors[1].a = 0.3;
        marker.colors[1].r = 1.0;
        marker.colors[1].g = 0.0;
        marker.colors[1].b = 0.0;

        markers.markers.push_back(marker);
      }

      // save Skyplot to CSV
      float d1=0,d2=0,d3=0;
      d1 = dynamicP.x;
      d2 = dynamicP.y;
      d3 = dynamicP.z;
      CSV_R.push_back(sqrt(d1 * d1 + d2 * d2 + d3 * d3));
      CSV_elevation.push_back(maskPoints[k].elevation);
      CSV_azimuth.push_back(maskPoints[k].azimuth);

    }

    if(unZeroNum ==0 ) unZeroNum=1;
    uncertainty_.x = uncertainty_.x / unZeroNum;
    uncertainty_.y = 0;
    uncertainty_.z = 0;
    meanDistance = meanDistance / unZeroNum;
    if(meanDistance>25) meanDistance=25;
    // std::cout<<"uncertainty_.x->  "<<uncertainty_.x <<std::endl;

    /* *****************calculate GDOP (refer to Dr. Li-ta Hsu's slide in Avionics)********************/
    // Eigen::Matrix4f tilt_matrix = Eigen::Matrix4f::Identity();

    G_Matrix.resize(maskPoints_.size(), 3);
    float GDOP_=0;
    if(maskPoints_.size()>8)
    {
      for (int k = 0; k < maskPoints_.size(); k++) {
      G_Matrix(k,0) = cos(maskPoints_[k].elevation * M_PI/180.0) * cos(maskPoints_[k].azimuth * M_PI/180.0);
      G_Matrix(k,1) = cos(maskPoints_[k].elevation * M_PI/180.0) * sin(maskPoints_[k].azimuth * M_PI/180.0);
      G_Matrix(k,2) = sin(maskPoints_[k].elevation * M_PI/180.0);
      }
      Eigen::MatrixXd H_Matrix = (G_Matrix.transpose() * G_Matrix); // H matrix for GDOP Calculation
      H_Matrix = H_Matrix.inverse();
      // std::cout<<"uncertainty_.x->  "<<uncertainty_.x<<"         GDOP-> "<<sqrt( H_Matrix(0,0) + H_Matrix(1,1))  <<
      // "\n"<<std::endl;
      CSV_GDOP.push_back(sqrt( H_Matrix(0,0) + H_Matrix(1,1)));
      CSV_meanEle.push_back(uncertainty_.x);
      GDOP_=sqrt( H_Matrix(0,0) + H_Matrix(1,1));
      if(GDOP_!=GDOP_) std::cout<<"-nan in matrix "<<std::endl;
      if(isnan(GDOP_)) std::cout<<"-nan in matrix using isnan"<<std::endl;

    }
    else GDOP_=0;
    

    filtered->width = filtered->size();
    // filtered->height = 1;
    filtered->is_dense = false;

    // Draw the uncertainty ellipse
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time();
    marker.ns = "ellipse";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = -1* sensor_height;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = uncertainty_.x/1.3; // /10
    marker.scale.y = uncertainty_.x/1.3; // /10
    marker.scale.z = uncertainty_.x/1.3; // /10
    marker.color.a = 0.99; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    // markers.markers.push_back(marker);

    mask_points_pub.publish(filtered);
    pub_debug_marker_.publish(markers);
    uncertainty_pub.publish(uncertainty_);
    GPSTow.push_back(nmeaDGPSTime);
    alllon.push_back(nmeaLon);
    alllat.push_back(nmeaLat);
    allalt.push_back(nmeaAlt);
    allMeanEle.push_back(uncertainty_.x);
    allGDOP.push_back(GDOP_);
    allMeanDis.push_back(meanDistance);
    num_points.push_back(maskPoints_.size());

    // std::cout<<"markers.size  "<<markers.markers.size() <<std::endl;

    // save Skyplot
    // std::ofstream CSV_R_file("/home/wenws/amsipolyu/src/LiDAROdometry/hdl_graph_slam/data/2/CSV_R.csv");
    int vsize = CSV_R.size();
    // for (int n=0; n<vsize; n++)
    // {
    //     CSV_R_file << CSV_R[n] << std::endl;
    // }
    // std::ofstream CSV_elevation_file("/home/wenws/amsipolyu/src/LiDAROdometry/hdl_graph_slam/data/2/CSV_elevation.csv");
    // vsize = CSV_elevation.size();
    // for (int n=0; n<vsize; n++)
    // {
    //     CSV_elevation_file << CSV_elevation[n] << std::endl;
    // }
    // std::ofstream CSV_azimuth_file("/home/wenws/amsipolyu/src/LiDAROdometry/hdl_graph_slam/data/2/CSV_azimuth_file.csv");
    // vsize = CSV_azimuth.size();
    // for (int n=0; n<vsize; n++)
    // {
    //     CSV_azimuth_file << CSV_azimuth[n] << std::endl;
    // }

    // std::ofstream CSV_GDOP_file("/home/wenws/amsipolyu/src/LiDAROdometry/hdl_graph_slam/data/2/CSV_GDOP_file.csv");
    // std::ofstream CSV_meanele_file("/home/wenws/amsipolyu/src/LiDAROdometry/hdl_graph_slam/data/2/CSV_meanele_file.csv");
    // vsize = CSV_GDOP.size();
    // for (int n=0; n<vsize; n++)
    // {
    //     CSV_GDOP_file << CSV_GDOP[n] << std::endl;
    //     CSV_meanele_file << CSV_meanEle[n] << std::endl;

    // }

    std::ofstream LiDARFeatures("/home/wenws/amsipolyu/src/LiDAROdometry/hdl_graph_slam/data/2/LiDARFeatures.csv");
    std::ofstream allmeanelefile("/home/wenws/amsipolyu/src/LiDAROdometry/hdl_graph_slam/data/2/allmeanele.csv");
    std::ofstream allgdopfile("/home/wenws/amsipolyu/src/LiDAROdometry/hdl_graph_slam/data/2/allGDOP.csv");
    std::ofstream allmeanDisfile("/home/wenws/amsipolyu/src/LiDAROdometry/hdl_graph_slam/data/2/allmeanDisfile.csv");
    vsize = GPSTow.size();
    std::ostringstream data_ss;
    data_ss.precision(12);
    data_ss<<"GPSTime"<<","<<"meanElevation"<<","<<"GDOP"<<","<<"num_points"<<","<<"meanDis"<<"nmea_lon"<<"nmea_lat"<<"nmea_alt"<<"\n";
    // data_ss  nmeaAlt
    double preTime=0;
    for (int n=0; n<vsize; n++)
    {
      double isOk=(int)GPSTow[n] - (int)preTime;

      if(fabs(isOk))
      {
        // std::cout<<"preTime->"  <<preTime<< "  GPSTow[n]" <<GPSTow[n]<<"  fabs(isOk)"<<fabs(isOk)<<std::endl;
        data_ss<<GPSTow[n]<<",";
        data_ss<<allMeanEle[n]<<",";
        data_ss<<allGDOP[n]<<",";
        data_ss<<num_points[n]<<",";
        data_ss<<allMeanDis[n]<<",";
        data_ss<<alllon[n]<<",";
        data_ss<<(double)alllat[n]<<",";
        data_ss<<allalt[n]<<",";
        data_ss<<"\n";
      }
      
      preTime=GPSTow[n];

      // GPSTowfile << GPSTow[n] << std::endl;
      // allmeanelefile << allMeanEle[n] << std::endl;
      // allgdopfile << allGDOP[n] << std::endl;
      // allmeanDisfile << allMeanDis[n] << std::endl;
    }
    string data_(data_ss.str());
    LiDARFeatures << data_<< std::endl;


  }


  

private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  // ROS topics
  ros::Subscriber points_sub;
  ros::Subscriber nmea_sub;

  ros::Publisher floor_pub;
  ros::Publisher floor_points_pub;
  ros::Publisher floor_filtered_pub;

  ros::Publisher mask_points_pub; // mask points publisher
  ros::Publisher pub_debug_marker_; // marker publisher
  ros::Publisher uncertainty_pub; // uncertainty publisher

  ros::Publisher read_until_pub;

  // floor detection parameters
  // see initialize_params() for the details
  double tilt_deg;
  double sensor_height;
  double height_clip_range;

  int floor_pts_thresh;
  double floor_normal_thresh;

  bool use_normal_filtering;
  double normal_filter_thresh;

  double nmeaDGPSTime;
  double nmeaLat;
  double nmeaLon;
  double nmeaAlt;
 

 
};

}

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::FloorDetectionNodelet, nodelet::Nodelet)
