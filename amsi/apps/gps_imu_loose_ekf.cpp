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
// fstream
#include <fstream>
#include<sstream>
#include <stdlib.h>
#include <iomanip>

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

#include <nlosExclusion/GNSS_Raw_Array.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <novatel_msgs/INSPVAX.h> // novatel_msgs/INSPVAX
#include <geometry_msgs/Point32.h>


#define INF 10000
#define pi 3.1415926

using namespace Eigen;

using namespace std;

FILE* fp_out_loose_ekf = fopen("/home/wenws/amsipolyu/IONGNSS2019_result/gps_imu_loose_ekf.csv", "w+");

FILE* generateData = fopen("/home/wenws/libRSF/datasets/smartLoc/data_kowloon_rang3.txt", "w+");

FILE* generateData_odom = fopen("/home/wenws/libRSF/datasets/smartLoc/data_kowloon_odom.txt", "w+");

FILE* generateData_gt3 = fopen("/home/wenws/libRSF/datasets/smartLoc/data_kowloon_gt3.txt", "w+");

FILE* generateData_imu = fopen("/home/wenws/libRSF/datasets/smartLoc/data_kowloon_imu.txt", "w+");







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
  ros::Subscriber gps_raw_sub, heading_sub;
  ros::Subscriber span_BP_sub;

  ros::Publisher ekf_loose_Pose_pub;
  ros::Publisher ekf_GNSS_odom_pub;
  ros::Publisher ekf_span_odom_pub;
  ros::Publisher gnss_navsat_pose_pub, imu_v_pub, span_v_pub;

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
  int imu_queue_size = 5;

  // EKF related parameters
  Eigen::MatrixXd imu_noise_matrix;

  Eigen::MatrixXd Q_matrix, G_matrix, sigma_matrix, R_matrix, K_matrix, H_matrix;
 
  VectorXd ekf_state; // state:  px, py, pz, vx, vy, vz, bax, bzy, baz
  VectorXd ekf_u_t; // ax, ay, az
  VectorXd ekf_z_t; // px, py, pz

  double prediction_pre_t=0;

  vector<double> posi_err, posi_err_gps;

  Eigen::MatrixXd span_ecef; // 

  double nstamps_ = 0;

  novatel_msgs::INSPVAX head_track;

  geometry_msgs::Point32 imu_v, span_v; 


public:
  /**
   * @brief constructor
   * @param imu_msg
   */
  ekf_loose(bool state)
  {
    std::cout<<"----------------constructor-----------------"<<std::endl;
    // imu_sub = nh.subscribe("/imu/data", 50, &ekf_loose::imu_callback,this); // imu_rt
    imu_sub = nh.subscribe("/imu/data", 50, &ekf_loose::imu_callback,this); // imu_rt
    gps_sub = nh.subscribe("/ublox_gps_node/fix", 50, &ekf_loose::ubloxFix_callback,this);  // subscribe the result from WLS
    span_BP_sub =nh.subscribe("/novatel_data/bestpos", 50, &ekf_loose::span_bp_callback,this);

    gps_raw_sub = nh.subscribe("/GNSS_CV", 50, &ekf_loose::GNSS_raw_callback,this);

    heading_sub = nh.subscribe("/novatel_data/inspvax", 500
      , &ekf_loose::heading_callback, this); // heading from span-cpt 

    gnss_navsat_pose_pub = nh.advertise<sensor_msgs::NavSatFix>("/ublox_gps_node/fix", 10); 

    ekf_loose_Pose_pub = nh.advertise<nav_msgs::Odometry>("/ekf_loose_Pose", 10);
    ekf_GNSS_odom_pub = nh.advertise<nav_msgs::Odometry>("/ekf_ublox_odom", 10);
    ekf_span_odom_pub = nh.advertise<nav_msgs::Odometry>("/ekf_span_odom", 10);

    imu_v_pub = nh.advertise<geometry_msgs::Point32>("/imu_der_velocity", 10);
    span_v_pub = nh.advertise<geometry_msgs::Point32>("/span_der_velocity", 10);

    onInitEKF(); // initialize EKF related parameters
    // fprintf(fp_out_loose_ekf, "%s ,%s ,%s ,%s ,%s ,%s, %s, %s, %s \n", "epoch", "GPS_Eror", "GPS_IMU_loose__Eror", "gps_E", "gps_N", "loose_E", "loose_N", "Gt_E", "Gt_N");
    // fprintf(generateData, "%s %s %s %s %s %s %s %s %s \n", "epoch", "GPS_Eror", "GPS_IMU_loose__Eror", "gps_E", "gps_N", "loose_E", "loose_N", "Gt_E", "Gt_N");
    

  }

  ~ekf_loose()
  {
  }


void heading_callback(const novatel_msgs::INSPVAXConstPtr& msg) // subscribe the GNSS SPP topic
  {
    head_track  = *msg;
    // std::cout<< "heading = " <<head_track<<std::endl;
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
    double g_ = 9.8;
    // std::cout << " IMU data call back" << input->angular_velocity.x << std::endl;
    imu_track = * input;
    imu_track.linear_acceleration.x = imu_track.linear_acceleration.x * g_;
    imu_track.linear_acceleration.y = imu_track.linear_acceleration.y * g_;
    imu_track.linear_acceleration.z = imu_track.linear_acceleration.z * g_;

    double imu_roll, imu_pitch, imu_yaw;
    tf::Quaternion imu_orientation;
    tf::quaternionMsgToTF(input->orientation, imu_orientation);
    tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);
    imu_roll = imu_roll * 180.0 / pi;
    imu_pitch = imu_pitch * 180.0 / pi;
    imu_yaw = imu_yaw * 180.0 / pi;
    
    // std::cout<< "    imu_roll -> " <<imu_roll << "    imu_pitch-> "<< imu_pitch << "   imu_yaw-> " << imu_yaw << std::endl;

    // Eigen::Quaterniond q(imu_track.orientation.w,imu_track.orientation.x,imu_track.orientation.y,imu_track.orientation.z);
    // q.normalized();
    // Eigen::Matrix3d l2b_rotation;
    // l2b_rotation=q.toRotationMatrix();
    // Eigen::MatrixXd lineAcc_body;
    // Eigen::MatrixXd lineAcc_local;
    // lineAcc_body.resize(3,1);
    // lineAcc_body<<imu_track.linear_acceleration.x,
    //          imu_track.linear_acceleration.y,
    //          imu_track.linear_acceleration.z;
    // lineAcc_local = l2b_rotation.transpose() * lineAcc_body;
    // imu_track.linear_acceleration.x = lineAcc_local(0,0);
    // imu_track.linear_acceleration.y = lineAcc_local(1,0);
    // imu_track.linear_acceleration.z = lineAcc_local(2,0);
    // std::cout<< "    imu_track.linear_acceleration.x -> " <<imu_track.linear_acceleration.x << "    imu_track.linear_acceleration.y-> " <<imu_track.linear_acceleration.y << std::endl;


    imu_queue.push_back(imu_track);
    // cout<<"imu_queue_size -> "<<imu_queue.size()<<endl;
    if( (imu_queue.size() > imu_queue_size) && (online_cal_success ==0) && (use_online_imu_cal ==1))
    {
      online_cal_success = 1;
      const clock_t begin_time = clock();
      cout<<"-------------------start calibrate imu------------------------ "<<endl;
      imu_parameter = imu_calibtation(imu_queue);
      std::cout << "imu Calibration used  time -> " << double(clock() - begin_time) / CLOCKS_PER_SEC << "\n\n";
      cout<<"bias ax ->" <<imu_parameter.bias.ax<<endl;
      cout<<"bias ay ->" <<imu_parameter.bias.ay<<endl;
      cout<<"bias az ->" <<imu_parameter.bias.az<<endl;

      cout<<"bias wx ->" <<imu_parameter.bias.wx<<endl;
      cout<<"bias wy ->" <<imu_parameter.bias.wy<<endl;
      cout<<"bias wz ->" <<imu_parameter.bias.wz<<endl;
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

      imu_v.x = imu_v.x + ekf_u_t(0) * delta_t;
      imu_v.y = imu_v.y + ekf_u_t(1) * delta_t;
      imu_v.z = sqrt( pow(imu_v.x, 2) + pow(imu_v.y, 2));
      imu_v_pub.publish(imu_v);

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

      // fprintf(generateData_imu, "%s %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f \n", "imu", nstamps_, 
      //       imu_track.linear_acceleration.x, imu_track.linear_acceleration.y, imu_track.linear_acceleration.z,
      //       imu_track.angular_velocity.x, imu_track.angular_velocity.y,imu_track.angular_velocity.z,
      //       imu_track.linear_acceleration_covariance[3], imu_track.linear_acceleration_covariance[4],imu_track.linear_acceleration_covariance[5],
      //       imu_track.linear_acceleration_covariance[6],imu_track.linear_acceleration_covariance[7],imu_track.linear_acceleration_covariance[8]); // generateData_gt3    not always span_ecef available at the first epoch


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

  /**
   * @brief GNSS raw callback
   * @param GNSS msg
   * @return void
    std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
    nlosExclusion/GNSS_Raw[] GNSS_Raws
    float64 GNSS_time
    float64 total_sv
    float64 prn_satellites_index
    float64 pseudorange
    float64 snr
    float64 elevation
    float64 azimuth
    float64 err_tropo
    float64 err_iono
    float64 sat_clk_err
    float64 sat_pos_x
    float64 sat_pos_y
    float64 sat_pos_z
    int64 visable
    string sat_system
    BeiDou: 88->88+37
    GPS: 1->32
   @ 
   */
  void GNSS_raw_callback(const nlosExclusion::GNSS_Raw_Array::Ptr& input)
  {
    nstamps_++;
    // cout<<"GNSS raw data"<<endl;
    nlosExclusion::GNSS_Raw_Array GNSS_data = *input; 


    /**
   * @brief weighted least square for signle point positioning
   * @param eAllSVPositions ((n,4) prn, sx, sy, sz, )     eAllSVPositions ((n,3) PRN CNO Pseudorange)
   * @return eWLSSolution 5 unknowns with two clock bias variables
   @ 
    */
    Eigen::MatrixXd eAllSVPositions; // satellite positions   
    Eigen::MatrixXd eAllMeasurement; // pseudorange measurements
    eAllSVPositions.resize(GNSS_data.GNSS_Raws.size(), 4);
    eAllMeasurement.resize(GNSS_data.GNSS_Raws.size(), 3);
    // double stamp_ = (ros::Time::now()).toSec() - 1551496923;
    double stamp_ = nstamps_;
    for(int i =0; i < GNSS_data.GNSS_Raws.size(); i++) // for weighted least square
    {
      eAllSVPositions(i,0) = GNSS_data.GNSS_Raws[i].prn_satellites_index;
      eAllSVPositions(i,1) = GNSS_data.GNSS_Raws[i].sat_pos_x;
      eAllSVPositions(i,2) = GNSS_data.GNSS_Raws[i].sat_pos_y;
      eAllSVPositions(i,3) = GNSS_data.GNSS_Raws[i].sat_pos_z;

      eAllMeasurement(i,0) = GNSS_data.GNSS_Raws[i].prn_satellites_index;
      eAllMeasurement(i,1) = GNSS_data.GNSS_Raws[i].snr;
      eAllMeasurement(i,2) = GNSS_data.GNSS_Raws[i].pseudorange;

      // if(PRNisGPS(GNSS_data.GNSS_Raws[i].prn_satellites_index))

      if(1)
      {
        bool use_CV_GNSS =1;
        double psr_cov = cofactorMatrixCal_single_satellite(GNSS_data.GNSS_Raws[i].elevation, GNSS_data.GNSS_Raws[i].snr);
        if(use_CV_GNSS == 0)
        {
          fprintf(generateData, "%s %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f \n", "range3", stamp_, GNSS_data.GNSS_Raws[i].pseudorange, 
            psr_cov , GNSS_data.GNSS_Raws[i].sat_pos_x, GNSS_data.GNSS_Raws[i].sat_pos_y, GNSS_data.GNSS_Raws[i].sat_pos_z, 
            GNSS_data.GNSS_Raws[i].prn_satellites_index , GNSS_data.GNSS_Raws[i].elevation, GNSS_data.GNSS_Raws[i].snr);
        }
        

        /* if the satellite is NLOS, the strainght forward method is to exclude all the satellites. 
          `however, we can also increase the covariance of the NLOS satellite measurements
        */
        // std::vector<double> NLOS_list = {12, 25, 31, 99};
        if(use_CV_GNSS == 1)
        {
          if(GNSS_data.GNSS_Raws[i].visable == 0)
          // if(std::find(NLOS_list.begin(), NLOS_list.end(), GNSS_data.GNSS_Raws[i].prn_satellites_index) != NLOS_list.end())
          {
            // std::cout<<"find NLOS satellite ->  " << GNSS_data.GNSS_Raws[i].prn_satellites_index <<std::endl;
            psr_cov = psr_cov * 1.2;
            fprintf(generateData, "%s %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f \n", "range3", stamp_, GNSS_data.GNSS_Raws[i].pseudorange, 
              psr_cov , GNSS_data.GNSS_Raws[i].sat_pos_x, GNSS_data.GNSS_Raws[i].sat_pos_y, GNSS_data.GNSS_Raws[i].sat_pos_z, 
              GNSS_data.GNSS_Raws[i].prn_satellites_index , GNSS_data.GNSS_Raws[i].elevation, GNSS_data.GNSS_Raws[i].snr);
          }
          else
          {
            fprintf(generateData, "%s %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f \n", "range3", stamp_, GNSS_data.GNSS_Raws[i].pseudorange, 
              psr_cov , GNSS_data.GNSS_Raws[i].sat_pos_x, GNSS_data.GNSS_Raws[i].sat_pos_y, GNSS_data.GNSS_Raws[i].sat_pos_z, 
              GNSS_data.GNSS_Raws[i].prn_satellites_index , GNSS_data.GNSS_Raws[i].elevation, GNSS_data.GNSS_Raws[i].snr);
          }
        }
        
        
      }
      
      double odom3_vx = sqrt(pow(ekf_state(3),2) + pow(ekf_state(4),2));

      std::cout<<"odom3_vx -> " << odom3_vx << std::endl;
      std::cout<<"ekf_state -> " << ekf_state << std::endl;

      // odom3_vx = sqrt(pow(head_track.north_velocity,2) + pow(head_track.east_velocity,2));

      span_v.x = head_track.north_velocity;
      span_v.y = head_track.east_velocity;
      span_v.z = odom3_vx;
      span_v_pub.publish(span_v);

      // double odom3_vx = 0.0;
      double odom3_vy = 0;
      double odom3_vz = 0;
      
      double odom3_wx = 0;
      double odom3_wy = 0;
      double odom3_wz = imu_track.angular_velocity.z;
      // double odom3_wz = 0.0;

      double odom3_vx_std = 10;
      double odom3_vy_std = 0.1;
      double odom3_vz_std = 0.1;
      
      double odom3_wx_std = 0.1;
      double odom3_wy_std = 0.1;
      double odom3_wz_std = 0.1;

      fprintf(generateData_odom, "%s %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f\n", "odom3", stamp_, 
            odom3_vx, odom3_vy, odom3_vz, odom3_wx, odom3_wy, odom3_wz, 
            odom3_vx_std, odom3_vy_std, odom3_vz_std, odom3_wx_std, odom3_wy_std, odom3_wz_std); // generateData_gt3
      fprintf(generateData_gt3, "%s %3.5f %3.5f %3.5f %3.5f \n", "gt3", stamp_, 
            span_ecef(0), span_ecef(1), span_ecef(2)); // generateData_gt3    not always span_ecef available at the first epoch
      // generateData_imu
      // fprintf(generateData_imu, "%s %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f %3.5f \n", "imu", stamp_, 
      //       imu_track.linear_acceleration.x, imu_track.linear_acceleration.y, imu_track.linear_acceleration.z,
      //       imu_track.angular_velocity.x, imu_track.angular_velocity.y,imu_track.angular_velocity.z,
      //       imu_track.linear_acceleration_covariance[3], imu_track.linear_acceleration_covariance[4],imu_track.linear_acceleration_covariance[5],
      //       imu_track.linear_acceleration_covariance[6],imu_track.linear_acceleration_covariance[7],imu_track.linear_acceleration_covariance[8]); // generateData_gt3    not always span_ecef available at the first epoch

      std::cout<<"span_ecef -> "<<span_ecef<<std::endl; 
    }
    Eigen::MatrixXd  eWLSSolutionECEF; // 5 unknowns with two clock bias variables
    // eWLSSolutionECEF = LeastSquare(eAllSVPositions, eAllMeasurement);  //
    eWLSSolutionECEF = WeightedLeastSquare(eAllSVPositions, eAllMeasurement, GNSS_data);  //WeightedLeastSquare

    Eigen::MatrixXd ecef;
    Eigen::MatrixXd llh;
    llh.resize(3, 1);
    ecef.resize(3, 1);
    ecef(0) = eWLSSolutionECEF(0);
    ecef(1) = eWLSSolutionECEF(1);
    ecef(2) = eWLSSolutionECEF(2);
    llh = gnss_tools_.ecef2llh(ecef);

    sensor_msgs::NavSatFix navfix_ ;
    navfix_.header = imu_track.header;
    navfix_.latitude = llh(1);
    navfix_.longitude = llh(0);
    navfix_.altitude = llh(2);
    gnss_navsat_pose_pub.publish(navfix_); // gnss standalone   
    // cout<<"llh-> \n"<< llh<<endl;  


  }


  /**
   * @brief gps callback
   * @param gps fix msg
   * @return void
   @ 
   */
  void ubloxFix_callback(const sensor_msgs::NavSatFixConstPtr& fix_msg) // update 
  {
    // cout<<"ubloxFix_callback received "<<endl;
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
        // cout<<"ekf_state-> " <<ekf_state<<endl;
        
        ekf_pose_odom.header = navfix_.header;
        ekf_pose_odom.header.frame_id = "odom";
        ekf_pose_odom.pose.pose.position.x = ekf_state(0);
        ekf_pose_odom.pose.pose.position.y = ekf_state(1);
        ekf_pose_odom.pose.pose.position.z = ekf_state(2);
        ekf_loose_Pose_pub.publish(ekf_pose_odom);

        double error_ekf = sqrt(pow(ekf_pose_odom.pose.pose.position.x - span_odom.pose.pose.position.x,2) + pow(ekf_pose_odom.pose.pose.position.y - span_odom.pose.pose.position.y,2));
        posi_err.push_back(error_ekf);
        statistical_para result = statis_cal(posi_err);
        // cout<<" positioning error (loose ekf) mean:->" << result.mean 
        //   << " positioning error (loose ekf) std: ->" << result.std;

        double error_gps = sqrt(pow(odom_track.pose.pose.position.x - span_odom.pose.pose.position.x,2) + pow(odom_track.pose.pose.position.y - span_odom.pose.pose.position.y,2));
        posi_err_gps.push_back(error_gps);
        result = statis_cal(posi_err_gps);
        // cout<<" positioning error (gps) mean:->" << result.mean <<
        //  " positioning error (gps) std: ->" << result.std;

        fprintf(fp_out_loose_ekf, "%d ,%3.2f ,%3.2f ,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f \n", posi_err.size(), error_gps, error_ekf, 
          odom_track.pose.pose.position.x, odom_track.pose.pose.position.y, ekf_pose_odom.pose.pose.position.x, ekf_pose_odom.pose.pose.position.y, 
          span_odom.pose.pose.position.x , span_odom.pose.pose.position.y);

      }
      
  }

  /**
   * @brief span_cpt callback
   * @param span_cpt bestpos msg
   * @return void
   @ 
   */
  void span_bp_callback(const novatel_msgs::BESTPOSConstPtr& fix_msg)
  {
    // cout<<"jd /fix received "<<endl;
    sensor_msgs::NavSatFix navfix_ ;
    navfix_.latitude = fix_msg->latitude;
    navfix_.longitude = fix_msg->longitude;
    navfix_.altitude = fix_msg->altitude;

    if(ini_navf_span.latitude == NULL)
      {
        ini_navf_span = navfix_;
        // std::cout<<"ini_navf_span.header  -> "<<ini_navf_span.header<<std::endl;
        originllh_span.resize(3, 1);
        originllh_span(0) = navfix_.longitude;
        originllh_span(1) = navfix_.latitude;
        originllh_span(2) = navfix_.altitude;
        // std::cout<<"reference longitude (span_cpt): "<<navfix_.longitude<<std::endl;
        // std::cout<<"reference latitude (span_cpt): "<<navfix_.latitude<<std::endl;
      }
      Eigen::MatrixXd curLLh; // 
      curLLh.resize(3, 1);
      curLLh(0) = navfix_.longitude;
      curLLh(1) = navfix_.latitude;
      curLLh(2) = navfix_.altitude;

      
      

      Eigen::MatrixXd ecef; // 
      ecef.resize(3, 1);
      ecef = gnss_tools_.llh2ecef(curLLh);
      span_ecef.resize(3, 1);
      span_ecef = ecef;
      Eigen::MatrixXd eigenENU;; // 
      eigenENU.resize(3, 1);
      eigenENU = gnss_tools_.ecef2enu(originllh_span,ecef);
      
      span_gps_week_sec = fix_msg->header.gps_week_seconds;
      span_odom.header.frame_id = "odom";
      span_odom.pose.pose.position.x = eigenENU(0);
      span_odom.pose.pose.position.y = eigenENU(1);
      span_odom.pose.pose.position.z = eigenENU(2);
      span_odom.pose.pose.orientation.x = 0;
      span_odom.pose.pose.orientation.y = 0;
      span_odom.pose.pose.orientation.z = 0;
      span_odom.pose.pose.orientation.w = 0;
      ekf_span_odom_pub.publish(span_odom);
      
      
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
   * @brief  weighted least square for signle point positioning
   * @param eAllSVPositions ((n,4) prn, sx, sy, sz, )     eAllSVPositions ((n,3) PRN CNO Pseudorange)
   * @return eWLSSolution 5 unknowns with two clock bias variables
   @ 
  */
  Eigen::MatrixXd WeightedLeastSquare(Eigen::MatrixXd eAllSVPositions, Eigen::MatrixXd eAllMeasurement, nlosExclusion::GNSS_Raw_Array GNSS_data){
  
    Eigen::MatrixXd eWLSSolution;
    eWLSSolution.resize(5, 1);

    MatrixXd weight_matrix = cofactorMatrixCal_WLS(GNSS_data);

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
      // eDeltaPos = (eH_Matrix.transpose() * weight_matrix * eH_Matrix).ldlt().solve(eH_Matrix.transpose() * weight_matrix *  eDeltaPr);
      eDeltaPos = (eH_Matrix.transpose() * weight_matrix * eH_Matrix).inverse() * eH_Matrix.transpose() * weight_matrix * eDeltaPr;
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
    cout<< "---------------WLS (ECEF) x, y, z, bias_gps, bias_beidou-----------------  \n"<<eWLSSolution<<endl;

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
  MatrixXd cofactorMatrixCal_EKF(nlosExclusion::GNSS_Raw_Array GNSS_data)
  {
    double snr_1 = 50.0; // T = 50
    double snr_A = 30.0; // A = 30
    double snr_a = 30.0;// a = 30
    double snr_0 = 10.0; // F = 10
    VectorXd cofactor_;  // cofactor of satellite
    cofactor_.resize(GNSS_data.GNSS_Raws.size());
    for(int i = 0; i < GNSS_data.GNSS_Raws.size(); i++)
    {
      if( (PRNisGPS(GNSS_data.GNSS_Raws[i].prn_satellites_index)) || (PRNisBeidou(GNSS_data.GNSS_Raws[i].prn_satellites_index)) )
      {
        double snr_R = GNSS_data.GNSS_Raws[i].snr;
        double elR = GNSS_data.GNSS_Raws[i].elevation;
        double q_R_1 = 1 / (pow(( sin(elR * pi/180.0 )),2));
        double q_R_2 = pow(10,(-(snr_R - snr_1) / snr_a));
        double q_R_3 = (((snr_A / (pow(10,(-(snr_0 - snr_1) / snr_a))) - 1) / (snr_0 - snr_1)) * (snr_R - snr_1) + 1);
        double q_R = q_R_1* (q_R_2 * q_R_3);
        cofactor_[i]=(float(q_R)); // uncertainty: cofactor_[i] larger, larger uncertainty
      }
    }
    // cout<<"cofactor_ -> "<<cofactor_<<endl;

    MatrixXd weight_matrix;
    weight_matrix.resize(GNSS_data.GNSS_Raws.size(),GNSS_data.GNSS_Raws.size());
    weight_matrix.setIdentity();
    for(int k = 0; k < weight_matrix.rows(); k++)
    {
      weight_matrix.row(k) = weight_matrix.row(k) * cofactor_(k);
    }

    return weight_matrix;
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
   * @brief covariance estimation
   * @param nlosExclusion::GNSS_Raw_Array GNSS_data
   * @return weight_matrix
   @ 
   */
  MatrixXd cofactorMatrixCal_WLS(nlosExclusion::GNSS_Raw_Array GNSS_data)
  {
    double snr_1 = 50.0; // T = 50
    double snr_A = 30.0; // A = 30
    double snr_a = 30.0;// a = 30
    double snr_0 = 10.0; // F = 10
    VectorXd cofactor_;  // cofactor of satellite
    cofactor_.resize(GNSS_data.GNSS_Raws.size());
    for(int i = 0; i < GNSS_data.GNSS_Raws.size(); i++)
    {
      if( (PRNisGPS(GNSS_data.GNSS_Raws[i].prn_satellites_index)) || (PRNisBeidou(GNSS_data.GNSS_Raws[i].prn_satellites_index)) )
      {
        double snr_R = GNSS_data.GNSS_Raws[i].snr;
        double elR = GNSS_data.GNSS_Raws[i].elevation;
        double q_R_1 = 1 / (pow(( sin(elR * pi/180.0 )),2));
        double q_R_2 = pow(10,(-(snr_R - snr_1) / snr_a));
        double q_R_3 = (((snr_A / (pow(10,(-(snr_0 - snr_1) / snr_a))) - 1) / (snr_0 - snr_1)) * (snr_R - snr_1) + 1);
        double q_R = q_R_1* (q_R_2 * q_R_3);
        cofactor_[i]=(1.0/float(q_R)); // uncertainty: cofactor_[i] larger, larger uncertainty
      }
    }
    // cout<<"cofactor_ -> "<<cofactor_<<endl;

    MatrixXd weight_matrix;
    weight_matrix.resize(GNSS_data.GNSS_Raws.size(),GNSS_data.GNSS_Raws.size());
    weight_matrix.setIdentity();
    for(int k = 0; k < weight_matrix.rows(); k++)
    {
      weight_matrix.row(k) = weight_matrix.row(k) * cofactor_(k);
    }

    return weight_matrix;
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