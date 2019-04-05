/* ----------------------------------------------------------------------------

 * amsi Copyright 2019, Positioning and Navigation Laboratory,
 * Hong Kong Polytechnic University
 * All Rights Reserved
 * Authors: Weisong Wen, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file gps_imu_loose_ekf.cpp
 * @brief fuse gps (ENU) and imu (ENU) using ekf (loosely coupling)
 * @author Weisong Wen (weisong.wen@connect.polyu.hk)
 */

/**
 * Example of use of the imuFactors (imuFactor and combinedImuFactor) in conjunction with GPS
 
 *  - we read IMU and GPS data from rosbag, with the following format:
 *  A topic with "/imu/data" is an imu measurement
 *  linAccN, linAccE, linAccD, angVelN, angVelE, angVelD
 *  A topic with "/ublox_gps_node/fix" is a gps correction formatted with
 *  lat, lon, altitude
 */
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <iostream>

#include <Eigen/Dense>
#include<Eigen/Core>
#include<Eigen/Geometry>
// fstream
#include <fstream>
#include<sstream>
#include <stdlib.h>
#include <iomanip>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


// math
#include <math.h>
//time 
#include <time.h>
//algorithm 
#include <algorithm>
// Define Infinite (Using INT_MAX caused overflow problems)

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <novatel_msgs/BESTPOS.h> // novatel_msgs/INSPVAX

#include <amsi/gnss_tools.hpp>


#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/PointCloud2.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <visualization_msgs/MarkerArray.h>


#include <amsi/gnss_tools.hpp>



#define INF 10000
#define pi 3.1415926

using namespace Eigen;

using namespace std;

FILE* fp_out_loose_ekf = fopen("/home/husai/gps_imu_loose_ekf.csv", "w+");

// FILE* generateData = fopen("/home/wenws/libRSF/datasets/smartLoc/data_kowloon_rang3.txt", "w+");

// FILE* generateData_odom = fopen("/home/wenws/libRSF/datasets/smartLoc/data_kowloon_odom.txt", "w+");

// FILE* generateData_gt3 = fopen("/home/wenws/libRSF/datasets/smartLoc/data_kowloon_gt3.txt", "w+");

// FILE* generateData_imu = fopen("/home/wenws/libRSF/datasets/smartLoc/data_kowloon_imu.txt", "w+");







class ekf_loose
{
  ros::NodeHandle nh;

public:

  typedef struct
  {
    double wx;
    double wy;
    double wz;
    double ax;
    double ay;
    double az;
  }imu_bias; // imu bias

  typedef struct
  {
    double wx;
    double wy;
    double wz;
    double ax;
    double ay;
    double az;
  }imu_noise; // imu bias

  typedef struct
  {
    imu_bias bias;
    imu_noise noise;
  }imu_para; // imu bias

  typedef struct
  {
    double mean;
    double std;
  }statistical_para; // imu bias || positioning evaluation


  typedef struct
  {
    double x;
    double y;
    double z;
    double ID;
  }bp; // building point 

  ros::Subscriber imu_sub;
  ros::Subscriber gps_sub;
  ros::Subscriber gps_raw_sub;
  ros::Subscriber span_BP_sub, dgps_nmea_sub;

  ros::Publisher ekf_loose_Pose_pub;
  ros::Publisher ekf_GNSS_odom_pub;
  ros::Publisher ekf_span_odom_pub;
  ros::Publisher gnss_navsat_pose_pub;

  sensor_msgs::Imu imu_track;
  bool imu_up =0 ;
  int imu_co=0;

  bool gps_up = 0;
  nav_msgs::Odometry odom_track; // GNSS SPP in ENU
  nav_msgs::Odometry span_odom; // span_cpt in ENU
  double span_gps_week_sec = 0;
  nav_msgs::Odometry ekf_pose_odom; // ekf pose in ENU
  // gnss_tools
  GNSS_Tools gnss_tools_;
  Eigen::MatrixXd originllh,originllh_span; // origin llh
  Eigen::MatrixXd referencellh; // origin llh
  sensor_msgs::NavSatFix ini_navf,ini_navf_span ; // initial sensor msg

  imu_bias offline_cal= {-0.00231128, 0.0019349, -0.000309033, 
                        -0.0000563799, -0.0004587, 0.0979159}; // offiline calibrated imu bias and noise

  imu_para imu_parameter; // online calibrated imu parameters using begin 2000 frames of imu raw measurements
  bool use_online_imu_cal = 1;
  bool online_cal_success = 0;
  vector<sensor_msgs::Imu> imu_queue;
  int imu_queue_size = 100;

  // EKF related parameters
  Eigen::MatrixXd imu_noise_matrix;

  Eigen::MatrixXd Q_matrix, G_matrix, sigma_matrix, R_matrix, K_matrix, H_matrix, orientation_matrix, acceleration_rotation;
 
  VectorXd ekf_state; // state:  px, py, pz, vx, vy, vz, bax, bzy, baz
  VectorXd ekf_u_t; // ax, ay, az
  VectorXd ekf_z_t; // px, py, pz

  double prediction_pre_t=0;

  vector<double> posi_err, posi_err_gps;

  Eigen::MatrixXd span_ecef; // 

  double nstamps_ = 0;

  double x,y,z,w;



public:
  /**
   * @brief constructor
   * @param imu_msg
   */
  ekf_loose(bool state)
  {
    std::cout<<"----------------constructor-----------------"<<std::endl;
    // imu_sub = nh.subscribe("/imu/data", 50, &ekf_loose::imu_callback,this); // imu_rt
    imu_sub = nh.subscribe("/imu_raw", 50, &ekf_loose::imu_callback,this); // imu_rt
    gps_sub = nh.subscribe("/ublox_gps_node/fix", 50, &ekf_loose::ubloxFix_callback,this); 
    dgps_nmea_sub =nh.subscribe("/nmea_sentence", 50, &ekf_loose::nmea_callback,this);

    gnss_navsat_pose_pub = nh.advertise<sensor_msgs::NavSatFix>("/ublox_gps_node/fix2", 10);

    ekf_loose_Pose_pub = nh.advertise<nav_msgs::Odometry>("/ekf_loose_Pose", 10);
    ekf_GNSS_odom_pub = nh.advertise<nav_msgs::Odometry>("/ekf_gnss_odom", 10);
    ekf_span_odom_pub = nh.advertise<nav_msgs::Odometry>("/ekf_span_odom", 10);

    onInitEKF(); // initialize EKF related parameters
    // fprintf(fp_out_loose_ekf, "%s ,%s ,%s ,%s ,%s ,%s, %s, %s, %s \n", "epoch", "GPS_Eror", "GPS_IMU_loose__Eror", "gps_E", "gps_N", "loose_E", "loose_N", "Gt_E", "Gt_N");
    // fprintf(generateData, "%s %s %s %s %s %s %s %s %s \n", "epoch", "GPS_Eror", "GPS_IMU_loose__Eror", "gps_E", "gps_N", "loose_E", "loose_N", "Gt_E", "Gt_N");
    

  }

  ~ekf_loose()
  {
  }


  /**
   * @brief imu callback
   * @param imu msg
   * @return void
   @ 
   */
  void onInitEKF(void)
  {
    ekf_state.resize(9);
    ekf_state << 0,0,0,
                 0,0,0,
                 0,0,0;
    
    ekf_u_t.resize(3);
    ekf_u_t << 0,0,0;

    ekf_z_t.resize(3);
    ekf_z_t << 0,0,0;

    Q_matrix.resize(9,9);
    Q_matrix << 0.0002, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0.0002, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0.0002, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0.0001, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0.0001, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0.0001, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0.05, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0.05, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0.05; 
    Q_matrix = Q_matrix * 1;

    sigma_matrix.resize(9,9);
    sigma_matrix << 3.0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 3.0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 3.2, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0.3, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0.3, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0.3, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0.07, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0.07, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0.07; 
    
    G_matrix.resize(9,9);

    R_matrix.resize(3,3);
    R_matrix << 50,0,0,
                0,50,0,
                0,0,50;

    K_matrix.resize(9,3);
    H_matrix.resize(3,9);
    H_matrix << 1, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0, 0, 0, 0;
  }

  /**
   * @brief imu callback
   * @param imu msg
   * @return void
   @ 
   */
  void imu_callback(const sensor_msgs::Imu::Ptr& input)
  {
    // std::cout << " IMU data call back" << input->angular_velocity.x << std::endl;
    imu_track = * input;
    x = imu_track.orientation.x;
    y = imu_track.orientation.y;
    z = imu_track.orientation.z;
    w = imu_track.orientation.w;

    double imu_roll, imu_pitch, imu_yaw;
    tf::Quaternion imu_orientation;
    tf::quaternionMsgToTF(input->orientation, imu_orientation);
    tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);
    cout<<"yaw : = " <<imu_yaw*(180/3.14)<<endl;
    // cout<<imu_track.orientation.x<<endl;
    cout<<"imu_track.linear_acceleration.x : = " <<endl<<imu_track.linear_acceleration.x<<endl;
    
    Eigen::Quaterniond q(w,x,y,z);
    q.normalized();
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix=q.toRotationMatrix();
    orientation_matrix.resize(3,1);
    orientation_matrix<<imu_track.linear_acceleration.x,
                        imu_track.linear_acceleration.y,
                        imu_track.linear_acceleration.z;
    acceleration_rotation = rotation_matrix.transpose() * orientation_matrix;

    cout<<"acceleration_rotation : = " <<endl<<acceleration_rotation<<endl;
    imu_track.linear_acceleration.x = acceleration_rotation(0,0);
    imu_track.linear_acceleration.y = acceleration_rotation(1,0);
    imu_track.linear_acceleration.z = acceleration_rotation(2,0);
    cout<<"imu_track.linear_acceleration.x after rotation: = " <<endl<<imu_track.linear_acceleration.x<<endl;
    cout<<"imu_track.linear_acceleration.y after rotation: = " <<endl<<imu_track.linear_acceleration.y<<endl;
    cout<<"imu_track.linear_acceleration.z after rotation: = " <<endl<<imu_track.linear_acceleration.z<<endl;
    cout<<"-----------------------------" <<endl;


    imu_queue.push_back(imu_track);
    // cout<<"imu_queue_size -> "<<imu_queue.size()<<endl;
    if( (imu_queue.size() > imu_queue_size) && (online_cal_success ==0) && (use_online_imu_cal ==1))
    {
      online_cal_success = 1;
      const clock_t begin_time = clock();
      cout<<"-------------------start calibrate imu------------------------ "<<endl;
      imu_parameter = imu_calibtation(imu_queue);
      std::cout << "imu Calibration used  time -> " << double(clock() - begin_time) / CLOCKS_PER_SEC << "\n\n";
      // cout<<"bias ax ->" <<imu_parameter.bias.ax<<endl;
      // cout<<"bias ay ->" <<imu_parameter.bias.ay<<endl;
      // cout<<"bias az ->" <<imu_parameter.bias.az<<endl;

      // cout<<"bias wx ->" <<imu_parameter.bias.wx<<endl;
      // cout<<"bias wy ->" <<imu_parameter.bias.wy<<endl;
      // cout<<"bias wz ->" <<imu_parameter.bias.wz<<endl;
    }
    if(use_online_imu_cal)
    {
        // decrease the bias 
      imu_track.angular_velocity.x = imu_track.angular_velocity.x - (imu_parameter.bias.wx);
      imu_track.angular_velocity.y = imu_track.angular_velocity.y - (imu_parameter.bias.wy);
      imu_track.angular_velocity.z = imu_track.angular_velocity.z - (imu_parameter.bias.wz);

      imu_track.linear_acceleration.x = imu_track.linear_acceleration.x - (imu_parameter.bias.ax);
      imu_track.linear_acceleration.y = imu_track.linear_acceleration.y - (imu_parameter.bias.ay);
      imu_track.linear_acceleration.z = imu_track.linear_acceleration.z - (imu_parameter.bias.az);
    }
    else if(!use_online_imu_cal) // if do not online calibrate, use offiline calibration (1h statistical static data)
    {
        // decrease the bias 
      imu_track.angular_velocity.x = imu_track.angular_velocity.x - (-0.00231128);
      imu_track.angular_velocity.y = imu_track.angular_velocity.y - (0.0019349);
      imu_track.angular_velocity.z = imu_track.angular_velocity.z - (-0.000309033);

      imu_track.linear_acceleration.x = imu_track.linear_acceleration.x - (-0.0000563799);
      imu_track.linear_acceleration.y = imu_track.linear_acceleration.y - (-0.0004587);
      imu_track.linear_acceleration.z = imu_track.linear_acceleration.z - (0.0979159);
    }
    
    

    imu_up =1;
    imu_co++;

    ekf_u_t(0) = imu_track.linear_acceleration.x;
    ekf_u_t(1) = imu_track.linear_acceleration.y;
    ekf_u_t(2) = imu_track.linear_acceleration.z;

    if(prediction_pre_t == 0)
    {
      prediction_pre_t = imu_track.header.stamp.toSec();
    }

    double delta_t = imu_track.header.stamp.toSec() -  prediction_pre_t;


    if(online_cal_success)
    {
      // cout<< "delta_t-> "<<delta_t;
        // position prediction
      ekf_state(0) = ekf_state(0) + ekf_state(3) * delta_t + 1/2 * (ekf_u_t(0) - 
        ekf_state(6)) * pow(delta_t, 2);
      ekf_state(1) = ekf_state(1) + ekf_state(4) * delta_t + 1/2 * (ekf_u_t(1) - 
        ekf_state(7)) * pow(delta_t, 2);
      ekf_state(2) = ekf_state(2) + ekf_state(5) * delta_t + 1/2 * (ekf_u_t(2) - 
        ekf_state(8)) * pow(delta_t, 2);

      // velocity estimation
      ekf_state(3) = ekf_state(3) + (ekf_u_t(0) - ekf_state(6)) * delta_t;
      ekf_state(4) = ekf_state(4) + (ekf_u_t(1) - ekf_state(7)) * delta_t;
      ekf_state(5) = ekf_state(5) + (ekf_u_t(2) - ekf_state(8)) * delta_t;

      // bias prediction
      ekf_state(6) = ekf_state(6);
      ekf_state(7) = ekf_state(7);
      ekf_state(8) = ekf_state(8);

      G_matrix << 1, 0, 0, delta_t, 0, 0, -1/2 * pow(delta_t,2), 0, 0,
                  0, 1, 0, 0, delta_t, 0, 0, -1/2 * pow(delta_t,2), 0,
                  0, 0, 1, 0, 0, delta_t, 0, 0, -1/2 * pow(delta_t,2),
                  0, 0, 0, 1, 0, 0, -delta_t, 0, 0,
                  0, 0, 0, 0, 1, 0, 0, -delta_t, 0,
                  0, 0, 0, 0, 0, 1, 0, 0, -delta_t,
                  0, 0, 0, 0, 0, 0, 1, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 1, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 1; 

      // sigma matrix prediction
      sigma_matrix = G_matrix * sigma_matrix * G_matrix.transpose() + Q_matrix;
      // cout<< "sigma_matrix-> "<<sigma_matrix<<endl;

      ekf_pose_odom.header = imu_track.header;
      ekf_pose_odom.header.frame_id = "odom";
      ekf_pose_odom.pose.pose.position.x = ekf_state(0);
      ekf_pose_odom.pose.pose.position.y = ekf_state(1);
      ekf_pose_odom.pose.pose.position.z = ekf_state(2);
      // ekf_loose_Pose_pub.publish(ekf_pose_odom);
    }

    prediction_pre_t = imu_track.header.stamp.toSec();
  }

   /**
   * @brief imu bias, noise calculation (calibration)
   * @param imu queue 
   * @return imu_para
   @ 
   */
  imu_para imu_calibtation(vector<sensor_msgs::Imu> input)
  {
    imu_para para; // bias + noise
    imu_bias bias; // bias of imu 
    imu_noise noise; // noise of imu

    // cout<<"imu bias calculation (calibration)"<<endl;
    vector<double> wx_queue;
    double wx_sum = 0, wx_mean = 0;

    vector<double> wy_queue;
    double wy_sum = 0, wy_mean = 0;

    vector<double> wz_queue;
    double wz_sum = 0, wz_mean = 0;

    vector<double> ax_queue;
    double ax_sum = 0, ax_mean = 0;

    vector<double> ay_queue;
    double ay_sum = 0, ay_mean = 0;

    vector<double> az_queue;
    double az_sum = 0, az_mean = 0;

    for(int i = 0; i < input.size()-1; i ++)
    {
      wx_queue.push_back(input[i].angular_velocity.x);
      wy_queue.push_back(input[i].angular_velocity.y);
      wz_queue.push_back(input[i].angular_velocity.z);

      ax_queue.push_back(input[i].linear_acceleration.x);
      ay_queue.push_back(input[i].linear_acceleration.y);
      az_queue.push_back(input[i].linear_acceleration.z);
    }

    statistical_para result = statis_cal(wx_queue);
    para.bias.wx = result.mean;
    para.noise.wx = result.std;

    result = statis_cal(wy_queue);
    para.bias.wy = result.mean;
    para.noise.wy = result.std;

    result = statis_cal(wz_queue);
    para.bias.wz = result.mean;
    para.noise.wz = result.std;

    result = statis_cal(ax_queue);
    para.bias.ax = result.mean;
    para.noise.ax = result.std;

    result = statis_cal(ay_queue);
    para.bias.ay = result.mean;
    para.noise.ay = result.std;

    result = statis_cal(az_queue);
    para.bias.az = result.mean;
    para.noise.az = result.std;

    return para;
  }

  /**
   * @brief statistical parameters inference
   * @param vector<double>  
   * @return statistical_para
   @ 
   */
  statistical_para statis_cal(vector<double> input)
  {
    statistical_para result;
    double sum = std::accumulate(std::begin(input), std::end(input), 0.0);
    double mean =  sum / input.size(); 
   
    double accum  = 0.0;
    std::for_each (std::begin(input), std::end(input), [&](const double d) {
      accum  += (d-mean)*(d-mean);
    });
   
    double stdev = sqrt(accum/(input.size()-1));

    result.mean = mean;
    result.std = stdev;
    return result;
  }

 

void nmea_callback(const nmea_msgs::SentenceConstPtr& nmea_msg) {
    std::vector<std::string> str_vec_ptr;
    std::string token;
    std::stringstream ss(nmea_msg->sentence);
    bool find_SOL_COMPUTED =0;
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
      if(ini_navf.latitude == NULL)
      {
        ini_navf = navfix_;
        std::cout<<"ini_navf.header  -> "<<ini_navf.header<<std::endl;
        originllh_span.resize(3, 1); 
        originllh_span(0) = navfix_.longitude;
        originllh_span(1) = navfix_.latitude;
        originllh_span(2) = navfix_.altitude;
        std::cout<<"reference longitude: "<<navfix_.longitude<<std::endl;
        std::cout<<"reference latitude: "<<navfix_.latitude<<std::endl;
      }
      Eigen::MatrixXd curLLh; // 
      curLLh.resize(3, 1);
      curLLh(0) = navfix_.longitude;
      curLLh(1) = navfix_.latitude;
      curLLh(2) = navfix_.altitude;

      Eigen::MatrixXd ecef; // 
      ecef.resize(3, 1);
      ecef = gnss_tools_.llh2ecef(curLLh);
      Eigen::MatrixXd eigenENU;; // 
      eigenENU.resize(3, 1);
      eigenENU = gnss_tools_.ecef2enu(originllh_span,ecef);

      span_odom.header.frame_id = "odom";
      span_odom.pose.pose.position.x = eigenENU(0);
      span_odom.pose.pose.position.y = eigenENU(1);
      span_odom.pose.pose.position.z = eigenENU(2);
      span_odom.pose.pose.orientation.x = 0;
      span_odom.pose.pose.orientation.y = 0;
      span_odom.pose.pose.orientation.z = 0;
      span_odom.pose.pose.orientation.w = 0;
      ekf_span_odom_pub.publish(span_odom);

      std::cout<<"push back message to gps_queue..."<<std::endl;
    }
  }



  /**
   * @brief gps callback
   * @param gps fix msg
   * @return void
   @ 
   */
  void ubloxFix_callback(const sensor_msgs::NavSatFixConstPtr& fix_msg) // update 
  {
    cout<<"ubloxFix_callback received "<<endl;
    sensor_msgs::NavSatFix navfix_ ;
    navfix_.header = fix_msg->header;
    navfix_.latitude = fix_msg->latitude;
    navfix_.longitude = fix_msg->longitude;
    navfix_.altitude = fix_msg->altitude;

    if(ini_navf.latitude == NULL)
      {
        ini_navf = navfix_;
        // std::cout<<"ini_navf.header  -> "<<ini_navf.header<<std::endl;
        originllh.resize(3, 1);
        originllh(0) = navfix_.longitude;
        originllh(1) = navfix_.latitude;
        originllh(2) = navfix_.altitude;
        // std::cout<<"reference longitude: "<<navfix_.longitude<<std::endl;
        // std::cout<<"reference latitude: "<<navfix_.latitude<<std::endl;
      }
      Eigen::MatrixXd curLLh; // 
      curLLh.resize(3, 1);
      curLLh(0) = navfix_.longitude;
      curLLh(1) = navfix_.latitude;
      curLLh(2) = navfix_.altitude;

      Eigen::MatrixXd ecef; // 
      ecef.resize(3, 1);
      ecef = gnss_tools_.llh2ecef(curLLh);
      Eigen::MatrixXd eigenENU;; // 
      eigenENU.resize(3, 1);
      eigenENU = gnss_tools_.ecef2enu(originllh_span,ecef);
      /*
      gps(0) = odom_track.pose.pose.position.x;
      gps(1) = odom_track.pose.pose.position.y;
      gps(2) = odom_track.pose.pose.position.z;
      gps(3) = odom_track.pose.pose.orientation.x;
      gps(4) = odom_track.pose.pose.orientation.y;
      gps(5) = odom_track.pose.pose.orientation.z;
      gps(6) = 1;
      */
      odom_track.header.frame_id = "odom";
      odom_track.pose.pose.position.x = eigenENU(0);
      odom_track.pose.pose.position.y = eigenENU(1);
      odom_track.pose.pose.position.z = 0;
      odom_track.pose.pose.orientation.x = 0;
      odom_track.pose.pose.orientation.y = 0;
      odom_track.pose.pose.orientation.z = 0;
      odom_track.pose.pose.orientation.w = 0;
      if(originllh_span.size()) // initial llh from span-cpt is available 
      {
        gps_up = 1;
      }
      
      ekf_GNSS_odom_pub.publish(odom_track);

      ekf_z_t(0) = odom_track.pose.pose.position.x; 
      ekf_z_t(1) = odom_track.pose.pose.position.y; 
      ekf_z_t(2) = odom_track.pose.pose.position.z; 

      if(online_cal_success) // imu calibration ready
      {
        // update 

        // update K_matrix
        Eigen::MatrixXd I_matrix;
        I_matrix.resize(9,9);
        I_matrix.setIdentity();
        K_matrix = sigma_matrix * H_matrix.transpose() * (H_matrix * sigma_matrix * 
          H_matrix.transpose() + R_matrix).inverse();
        ekf_state = ekf_state + K_matrix * (ekf_z_t - H_matrix * ekf_state);
        sigma_matrix = (I_matrix - K_matrix * H_matrix) * sigma_matrix;
        cout<<"ekf_state-> " <<ekf_state<<endl;
        
        ekf_pose_odom.header = navfix_.header;
        ekf_pose_odom.header.frame_id = "odom";
        ekf_pose_odom.pose.pose.position.x = ekf_state(0);
        ekf_pose_odom.pose.pose.position.y = ekf_state(1);
        ekf_pose_odom.pose.pose.position.z = ekf_state(2);
        ekf_loose_Pose_pub.publish(ekf_pose_odom);

        //2D position error between loose ekf and span
        double error_ekf = sqrt(pow(ekf_pose_odom.pose.pose.position.x - span_odom.pose.pose.position.x,2) + pow(ekf_pose_odom.pose.pose.position.y - span_odom.pose.pose.position.y,2));
        posi_err.push_back(error_ekf);
        statistical_para result = statis_cal(posi_err);
        cout<<" positioning error (loose ekf) mean:->" << result.mean 
          << " positioning error (loose ekf) std: ->" << result.std;
        //2D position error between gps and span
        double error_gps = sqrt(pow(odom_track.pose.pose.position.x - span_odom.pose.pose.position.x,2) + pow(odom_track.pose.pose.position.y - span_odom.pose.pose.position.y,2));
        posi_err_gps.push_back(error_gps);
        result = statis_cal(posi_err_gps);
        cout<<" positioning error (gps) mean:->" << result.mean <<
         " positioning error (gps) std: ->" << result.std;

        fprintf(fp_out_loose_ekf, "%d ,%3.2f ,%3.2f ,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f \n", posi_err.size(), error_gps, error_ekf, 
          odom_track.pose.pose.position.x, odom_track.pose.pose.position.y, ekf_pose_odom.pose.pose.position.x, ekf_pose_odom.pose.pose.position.y, 
          span_odom.pose.pose.position.x , span_odom.pose.pose.position.y);

      }
      
  }

  /**
   * @brief  least square for signle point positioning
   * @param eAllSVPositions ((n,4) prn, sx, sy, sz, )     eAllSVPositions ((n,3) PRN CNO Pseudorange)
   * @return eWLSSolution 5 unknowns with two clock bias variables
   @ 
  */
  Eigen::MatrixXd LeastSquare(Eigen::MatrixXd eAllSVPositions, Eigen::MatrixXd eAllMeasurement){
  
    Eigen::MatrixXd eWLSSolution;
    eWLSSolution.resize(5, 1);

    /**after read the obs file, one measure is not right**/
    int validNumMeasure=0;
    std::vector<int> validMeasure;
    for (int idx = 0; idx < eAllMeasurement.rows(); idx++){
      for (int jdx = 0; jdx < eAllSVPositions.rows(); jdx++){
        if (int(eAllMeasurement(idx, 0)) == int(eAllSVPositions(jdx, 0))){
          validNumMeasure++;
          validMeasure.push_back(int(eAllMeasurement(idx, 0)));
        }
      }
    }

    Eigen::MatrixXd validMeasurement; // for WLS 
    validMeasurement.resize(validNumMeasure,eAllMeasurement.cols());
    for (int idx = 0; idx < eAllMeasurement.rows(); idx++){
      for (int jdx = 0; jdx < eAllSVPositions.rows(); jdx++){
        if (int(eAllMeasurement(idx, 0)) == int(eAllSVPositions(jdx, 0))){
          for (int kdx = 0; kdx < eAllMeasurement.cols(); kdx++){
            // std::cout<<"satellite prn -> "<<eAllMeasurement(idx, 0)<<"\n"<<std::endl;
            validMeasurement(idx, kdx) = eAllMeasurement(idx, kdx);
            
          }
        }
      }
    }



    int iNumSV = validMeasurement.rows();

    /*Find the received SV and Sort based on the order of Measurement matrix*/
    Eigen::MatrixXd eExistingSVPositions; // for WLS
    eExistingSVPositions.resize(iNumSV, eAllSVPositions.cols());

    for (int idx = 0; idx < validMeasurement.rows(); idx++){
      for (int jdx = 0; jdx < eAllSVPositions.rows(); jdx++){
        if (int(validMeasurement(idx, 0)) == int(eAllSVPositions(jdx, 0))){
          for (int kdx = 0; kdx < eAllSVPositions.cols(); kdx++){
            // std::cout<<"satellite prn -> "<<eAllMeasurement(idx, 0)<<"\n"<<std::endl;
            eExistingSVPositions(idx, kdx) = eAllSVPositions(jdx, kdx);
            
          }
        }
      }
    } 
    //for (int idx = 0; idx < eExistingSVPositions.rows(); idx++){
    //  printf("%2d-[%3d] - (%10.2f,%10.2f,%10.2f) %f\n", idx, int(eExistingSVPositions(idx, 0)), eExistingSVPositions(idx, 1), eExistingSVPositions(idx, 2), eExistingSVPositions(idx, 3), eExistingSVPositions(idx, 4)*CLIGHT);
    //}

    //Intialize the result by guessing.
    for (int idx = 0; idx < eWLSSolution.rows(); idx++){
      eWLSSolution(idx, 0) = 0;
    }
    
    // for the case of insufficient satellite
    if (iNumSV < 5){
      return eWLSSolution;
    }

    bool bWLSConverge = false;

    int count = 0;
    while (!bWLSConverge)
    {
      Eigen::MatrixXd eH_Matrix;
      eH_Matrix.resize(iNumSV, eWLSSolution.rows());

      Eigen::MatrixXd eDeltaPr;
      eDeltaPr.resize(iNumSV, 1);

      Eigen::MatrixXd eDeltaPos;
      eDeltaPos.resize(eWLSSolution.rows(), 1);

      for (int idx = 0; idx < iNumSV; idx++){

        int prn = int(validMeasurement(idx, 0));
        double pr = validMeasurement(idx, 2);
        
        // Calculating Geometric Distance
        double rs[3], rr[3], e[3];
        double dGeoDistance;

        rs[0] = eExistingSVPositions(idx, 1);
        rs[1] = eExistingSVPositions(idx, 2);
        rs[2] = eExistingSVPositions(idx, 3);

        rr[0] = eWLSSolution(0);
        rr[1] = eWLSSolution(1);
        rr[2] = eWLSSolution(2);

        // dGeoDistance = geodist(rs, rr, e);
        dGeoDistance = sqrt(pow((rs[0] - rr[0]),2) + pow((rs[1] - rr[1]),2) +pow((rs[2] - rr[2]),2));

        // Making H matrix      
        eH_Matrix(idx, 0) = -(rs[0] - rr[0]) / dGeoDistance;
        eH_Matrix(idx, 1) = -(rs[1] - rr[1]) / dGeoDistance;
        eH_Matrix(idx, 2) = -(rs[2] - rr[2]) / dGeoDistance;

        if (PRNisGPS(prn)){
          eH_Matrix(idx, 3) = 1;
          eH_Matrix(idx, 4) = 0;
        }
        else if (PRNisBeidou(prn))
        {
          eH_Matrix(idx, 3) = 1;
          eH_Matrix(idx, 4) = 1;
        }

        // Making delta pseudorange
        double rcv_clk_bias;
        if (PRNisGPS(prn)){
          rcv_clk_bias = eWLSSolution(3);       
        }
        else if (PRNisBeidou(prn))
        {
          rcv_clk_bias = eWLSSolution(4);
        }
        // double sv_clk_bias = eExistingSVPositions(idx, 4) * CLIGHT;
        eDeltaPr(idx, 0) = pr - dGeoDistance + rcv_clk_bias;
        //printf("%2d - %f %f %f %f \n", prn, pr, dGeoDistance, eDeltaPr(idx, 0), rcv_clk_bias);
      }

      // Least Square Estimation 
      eDeltaPos = (eH_Matrix.transpose() * eH_Matrix).ldlt().solve(eH_Matrix.transpose() *  eDeltaPr);
      //eDeltaPos = (eH_Matrix.transpose() * eH_Matrix).inverse() * eH_Matrix.transpose() *  eDeltaPr;
      //eDeltaPos = eH_Matrix.householderQr().solve(eDeltaPr);

      //for (int idx = 0; idx < eDeltaPos.rows(); idx++)
      //  printf("%f ", eDeltaPos(idx));
      //printf("\n");

      eWLSSolution(0) += eDeltaPos(0);
      eWLSSolution(1) += eDeltaPos(1);
      eWLSSolution(2) += eDeltaPos(2);
      eWLSSolution(3) += eDeltaPos(3);
      eWLSSolution(4) += eDeltaPos(4);

      for (int i = 0; i < 3; ++i){
        //printf("%f\n", fabs(eDeltaPos(i)));
        if (fabs(eDeltaPos(i)) >1e-4)
        {
          bWLSConverge = false;
        }
        else { 
          bWLSConverge = true;
        };
        
      }
      count += 1;
      if (count > 6)
        bWLSConverge = true;
    }
    // printf("WLS -> (%11.2f,%11.2f,%11.2f)\n\n", eWLSSolution(0), eWLSSolution(1), eWLSSolution(2));
    std::cout << std::setprecision(12);
    // cout<< "---------------WLS (ECEF) x, y, z, bias_gps, bias_beidou-----------------  \n"<<eWLSSolution<<endl;

    return eWLSSolution;
  }
  
 
  /**
   * @brief satellite set validation
   * @param prn
   * @return ture/false
   @ 
   */
  bool PRNisGPS(int prn)
  {
    if (prn <= 32 || prn == 84)
      return true;
    else{
      return false;
    } 
  }

  /**
   * @brief satellite set validation
   * @param prn
   * @return ture/false
   @ 
   */
  bool PRNisGLONASS(int prn)
  {
    if (prn > 32 && prn <= 56)
      return true;
    else{
      return false;
    }
  }

  /**
   * @brief satellite set validation
   * @param prn
   * @return ture/false
   @ 
   */
  bool PRNisBeidou(int prn)
  {
    if ((prn <= 121) && (prn >= 87))
      return true;
    else{
      return false;
    }
  }

  /**
   * @brief covariance estimation
   * @param nlosExclusion::GNSS_Raw_Array GNSS_data
   * @return weight_matrix
   @ 
   */
  double cofactorMatrixCal_single_satellite(double ele, double snr)
  {
    double cofactor_ = 0;
    double snr_1 = 50.0; // T = 50
    double snr_A = 30.0; // A = 30
    double snr_a = 30.0;// a = 30
    double snr_0 = 10.0; // F = 10

    double snr_R = snr;
    double elR = ele;
    double q_R_1 = 1 / (pow(( sin(elR * pi/180.0 )),2));
    double q_R_2 = pow(10,(-(snr_R - snr_1) / snr_a));
    double q_R_3 = (((snr_A / (pow(10,(-(snr_0 - snr_1) / snr_a))) - 1) / (snr_0 - snr_1)) * (snr_R - snr_1) + 1);
    double q_R = q_R_1* (q_R_2 * q_R_3);
    cofactor_ = (float(q_R)); // uncertainty: cofactor_[i] larger, larger uncertainty

    return cofactor_;
  }

 
  /**
   * @brief delay function
   * @param seconds for delay
   * @return void
   @ 
   */
  void wait(int seconds) // delay function
  {
    clock_t endwait,start;
    start = clock();
    endwait = clock() + seconds * CLOCKS_PER_SEC;
    while (clock() < endwait) {
      if(clock() - start > CLOCKS_PER_SEC)
      {
        start = clock();
        std::cout<<".......1 s"<<std::endl;
      }
    }
  }  

private:
  int reserve1;

private:
  ros::Publisher pub_debug_marker_; // marker publisher
  ros::Publisher marker_pub;
  visualization_msgs::MarkerArray markers; // markers for building models

  

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ekf_loose");
  std::cout<<"ekf_loose......"<<std::endl;

  // printf("obss.n = %d\n", obss.n);
  ekf_loose ekf_loose_(1);
  ros::spin();
  while (ros::ok()) {
  }
  return 0;
}