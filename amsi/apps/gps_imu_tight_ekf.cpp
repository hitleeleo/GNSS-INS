/* ----------------------------------------------------------------------------

 * amsi Copyright 2019, Positioning and Navigation Laboratory,
 * Hong Kong Polytechnic University
 * All Rights Reserved
 * Authors: Weisong Wen, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file gps_imu_tight_ekf.cpp
 * @brief fuse gps (ENU) and imu (ENU) using ekf (tight coupling)
 * @author Weisong Wen (weisong.wen@connect.polyu.hk)
 */

/**
 * Example of use of the imuFactors (imuFactor and combinedImuFactor) in conjunction with GPS
 
 *  - we read IMU and GPS data from rosbag, with the following format:
 *  A topic with "/imu/data" is an imu measurement
 *  linAccN, linAccE, linAccD, angVelN, angVelE, angVelD
 *  A topic with "/GNSS_" is a gps correction formatted with format nlosExclusion/GNSS_Raw_Array
 *  
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


#define INF 10000
#define D2R 3.1415926/180.0
#define pi 3.1415926
using namespace Eigen;

using namespace std;


FILE* fp_out_tight_ekf = fopen("/home/wenws/amsipolyu/IONGNSS2019_result/gps_imu_tight_ekf.csv", "w+");
// fprintf(fp_out_tight_ekf, "Sequence\n");

FILE* fp_out_loose_ekf_residual = fopen("/home/wenws/amsipolyu/IONGNSS2019_result/gps_imu_loose_ekf_residual.csv", "w+");

FILE* fp_out_llh_trajectory = fopen("/home/wenws/amsipolyu/IONGNSS2019_result/GNSS_imu_tight_llh_trajectory.csv", "w+");




class ekf_tight
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
  ros::Subscriber span_BP_sub;

  ros::Publisher ekf_tight_Pose_pub;
  ros::Publisher ekf_span_odom_pub;

  ros::Publisher WLS_pub;

  sensor_msgs::Imu imu_track;
  bool imu_up =0 ;
  int imu_co=0;

  bool gps_up = 0;
  nav_msgs::Odometry span_odom; // span_cpt in ENU
  double span_gps_week_sec = 0;
  nav_msgs::Odometry ekf_pose_odom; // ekf pose in ENU

  nav_msgs::Odometry WLS_pose_odom; // WLS pose in ENU
  // gnss_tools
  GNSS_Tools gnss_tools_;
  Eigen::MatrixXd originllh,originllh_span; // origin llh
  sensor_msgs::NavSatFix ini_navf,ini_navf_span ; // initial sensor msg

  imu_bias offline_cal= {-0.00231128, 0.0019349, -0.000309033, 
                        -0.0000563799, -0.0004587, 0.0979159}; // offiline calibrated imu bias and noise

  imu_para imu_parameter; // online calibrated imu parameters using begin 2000 frames of imu raw measurements
  bool use_online_imu_cal = 1;
  bool online_cal_success = 0;
  vector<sensor_msgs::Imu> imu_queue;
  int imu_queue_size = 5; // 2000

  // EKF related parameters
  Eigen::MatrixXd imu_noise_matrix;

  Eigen::MatrixXd Q_matrix, G_matrix, sigma_matrix, R_matrix, K_matrix, H_matrix;
 
  VectorXd ekf_state; // state:  px, py, pz, vx, vy, vz, bax, bzy, baz, bclo_gps, bclo_BeiDow
  VectorXd ekf_u_t; // ax, ay, az
  VectorXd ekf_z_t; // pseudoranges commonly 6~20 satellites

  double prediction_pre_t=0;

  vector<double> posi_err_WLS, posi_err_Tight;

  double usr_clk = 0;
  int gps_count = 0;


public:
  /**
   * @brief constructor
   * @param imu_msg
   */
  ekf_tight(bool state)
  {
    std::cout<<"----------------constructor-----------------"<<std::endl;
    imu_sub = nh.subscribe("/imu/data", 50, &ekf_tight::imu_callback,this);
    span_BP_sub =nh.subscribe("/novatel_data/bestpos", 50, &ekf_tight::span_bp_callback,this);
    gps_sub = nh.subscribe("/GNSS_", 50, &ekf_tight::GNSS_raw_callback,this);

    ekf_tight_Pose_pub = nh.advertise<nav_msgs::Odometry>("/ekf_tight_Pose", 10);
    ekf_span_odom_pub = nh.advertise<nav_msgs::Odometry>("/ekf_span_odom", 10);

    WLS_pub = nh.advertise<nav_msgs::Odometry>("/ekf_ublox_odom", 10);

    onInitEKF(); // initialize EKF related parameters
    // fprintf(fp_out_tight_ekf, "%s ,%s ,%s ,%s ,%s ,%s, %s, %s, %s \n", "epoch", "GPS_WLS_Eror", "GPS_IMU_Tight__Eror", "WLS_E", "WLS_N", "Tight_E", "Tight_N", "Gt_E", "Gt_N");


  }

  ~ekf_tight()
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
    ekf_state.resize(11);
    // ekf_state << -2417727,5384829,2408312, // p  -2417727,5384829,2408312   s:-10969201.8753,37049414.6844,16649050.0512
    //              0,0,0, // v
    //              0,0,0, // imu bias
    //              869339.640254,869340.258166; // satellites (GPS/BeiDou) clock bias

    ekf_state << -2418961.5793,5384681.00529,2407394.51534, // p  -2417727,5384829,2408312   s:-10969201.8753,37049414.6844,16649050.0512
                 0,0,0, // v
                 0,0,0, // imu bias
                 869339.640254,869340.258166; // satellites (GPS/BeiDou) clock bias
    
    ekf_u_t.resize(3);
    ekf_u_t << 0,0,0;

    Q_matrix.resize(11,11);
    // Q_matrix << 0.0002, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //             0, 0.0002, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    //             0, 0, 0.0002, 0, 0, 0, 0, 0, 0, 0, 0,
    //             0, 0, 0, 0.0001, 0, 0, 0, 0, 0, 0, 0,
    //             0, 0, 0, 0, 0.0001, 0, 0, 0, 0, 0, 0,
    //             0, 0, 0, 0, 0, 0.0001, 0, 0, 0, 0, 0,
    //             0, 0, 0, 0, 0, 0, 0.0005, 0, 0, 0, 0,
    //             0, 0, 0, 0, 0, 0, 0, 0.0005, 0, 0, 0,
    //             0, 0, 0, 0, 0, 0, 0, 0, 0.0005,  0, 0,
    //             0, 0, 0, 0, 0, 0, 0, 0, 0,  0.05, 0,
    //             0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0.05; //871744
    Q_matrix << 0.0002, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0.0002, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0.0002, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0.0001, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0.0001, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0.0001, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0.05, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0.05, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0.05, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3;

    Q_matrix = Q_matrix * 10;

    sigma_matrix.resize(11,11);
    sigma_matrix << 3.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 3.0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 3.2, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0.3, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0.3, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0.3, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0.007, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0.007, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0.007, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3;
    // sigma_matrix = sigma_matrix ;
    
    G_matrix.resize(11,11);
    cout<<"-----------------initial G_matrix-------------------  \n"<<G_matrix<<endl;

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

    // imu_track.linear_acceleration.x = imu_track.linear_acceleration.x * g_;
    // imu_track.linear_acceleration.y = imu_track.linear_acceleration.y * g_;
    // imu_track.linear_acceleration.z = imu_track.linear_acceleration.z * g_;

    imu_queue.push_back(imu_track);
    // cout<<"imu_queue_size -> "<<imu_queue.size()<<endl;
    if( (imu_queue.size() > imu_queue_size) && (online_cal_success ==0) && (use_online_imu_cal ==1))
    {
      online_cal_success = 1;
      const clock_t begin_time = clock();
      cout<<"-------------------start calibrate imu------------------------ "<<endl;
      imu_parameter = imu_calibtation(imu_queue);
      std::cout << "imu Calibration used  time -> " << double(clock() - begin_time)
       / CLOCKS_PER_SEC << "\n\n";
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


    if(online_cal_success && (originllh_span.rows() == 3)) // imu calibration, span origin available
    {
      // cout<< "delta_t-> "<<delta_t;
        // position prediction
      double lon = (double)originllh_span(0) * D2R;
      double lat = (double)originllh_span(1) * D2R;

      double e = ekf_u_t(0) - ekf_state(6); // acc in enu minus bias
      double n = ekf_u_t(1) - ekf_state(7);
      double u = ekf_u_t(2) - ekf_state(8);

      // double e = ekf_u_t(0) ; // acc in enu minus bias
      // double n = ekf_u_t(1) ;
      // double u = ekf_u_t(2) ;

      ekf_u_t(0) = 0 - sin(lon) * e - cos(lon) * sin(lat) * n + cos(lon) * cos(lat) * u; // transfrom the acc from enu to ecef
      ekf_u_t(1) = 0 + cos(lon) * e - sin(lon) * sin(lat) * n + cos(lat) * sin(lon) * u;
      ekf_u_t(2) = 0 + cos(lat) * n + sin(lat) * u;

      

      ekf_state(0) = ekf_state(0) + ekf_state(3) * delta_t + 1/2 * (ekf_u_t(0)) * pow(delta_t, 2);
      ekf_state(1) = ekf_state(1) + ekf_state(4) * delta_t + 1/2 * (ekf_u_t(1)) * pow(delta_t, 2);
      ekf_state(2) = ekf_state(2) + ekf_state(5) * delta_t + 1/2 * (ekf_u_t(2)) * pow(delta_t, 2);

      // velocity estimation
      ekf_state(3) = ekf_state(3) + (ekf_u_t(0)) * delta_t;
      ekf_state(4) = ekf_state(4) + (ekf_u_t(1)) * delta_t;
      ekf_state(5) = ekf_state(5) + (ekf_u_t(2)) * delta_t;

      // imu bias prediction
      ekf_state(6) = ekf_state(6);
      ekf_state(7) = ekf_state(7);
      ekf_state(8) = ekf_state(8);

      // satellite VS receiver clock bias
      ekf_state(9) = ekf_state(9); // GPS
      ekf_state(10) = ekf_state(10); // BeiDou

      if(imu_co > 100)
      {
        std::cout << std::setprecision(12);
        imu_co = 0;
        // cout<<"prediction: ekf_u_t_-------- \n"<<ekf_u_t<<endl;
        // cout<<"prediction: ekf_state-------- \n"<<ekf_state<<endl;
      }

      G_matrix << 1, 0, 0, delta_t, 0, 0, 1/2*sin(lon)*pow(delta_t,2), 1/2*sin(lat)*cos(lon)*pow(delta_t,2), -1/2*cos(lon)*cos(lat)*pow(delta_t,2), 0, 0,
                  0, 1, 0, 0, delta_t, 0, -1/2*cos(lon)*pow(delta_t,2), 1/2 *sin(lon)*sin(lat)*pow(delta_t,2), -1/2*sin(lon)*cos(lat)*pow(delta_t,2), 0, 0,
                  0, 0, 1, 0, 0, delta_t, 0, -1/2*cos(lat)*pow(delta_t,2), -1/2 * sin(lat)*pow(delta_t,2), 0, 0,
                  0, 0, 0, 1, 0, 0, sin(lon)*delta_t, sin(lat)*cos(lon)*delta_t, -1*cos(lon)*cos(lat)*delta_t, 0, 0,
                  0, 0, 0, 0, 1, 0, -1*cos(lon)*delta_t, sin(lon)*sin(lat)*delta_t, -1*sin(lon)*cos(lat)*delta_t, 0, 0,
                  0, 0, 0, 0, 0, 1, 0, -1*cos(lat)*delta_t, -1 * sin(lat)*delta_t, 0, 0,
                  0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
      // G_matrix << 1, 0, 0, delta_t, 0, 0, 0, 0, 0, 0, 0,
      //             0, 1, 0, 0, delta_t, 0, 0, 0, 0, 0, 0,
      //             0, 0, 1, 0, 0, delta_t, 0, 0, 0, 0, 0,
      //             0, 0, 0, 1, 0, 0, -1/2*delta_t, 0, 0, 0, 0,
      //             0, 0, 0, 0, 1, 0, 0, -1/2*delta_t, 0, 0, 0,
      //             0, 0, 0, 0, 0, 1, 0, 0, -1/2*delta_t, 0, 0,
      //             0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      //             0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      //             0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      //             0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
      //             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

      // sigma matrix prediction
      sigma_matrix = G_matrix * sigma_matrix * G_matrix.transpose() + Q_matrix;
      // sigma_matrix = sigma_matrix  + Q_matrix;
      std::cout << std::setprecision(5);
      // cout<< "---------------sigma_matrix-----------------  \n"<<sigma_matrix<<endl;

      Eigen::MatrixXd enu;
      Eigen::MatrixXd ecef;
      ecef.resize(3, 1);
      ecef(0) = ekf_state(0);
      ecef(1) = ekf_state(1);
      ecef(2) = ekf_state(2);
      enu.resize(3, 1);
      std::cout << std::setprecision(12);
      enu = gnss_tools_.ecef2enu(originllh_span, ecef);

      ekf_pose_odom.header = imu_track.header;
      ekf_pose_odom.header.frame_id = "odom";
      ekf_pose_odom.pose.pose.position.x = enu(0);
      ekf_pose_odom.pose.pose.position.y = enu(1);
      ekf_pose_odom.pose.pose.position.z = enu(2);
      ekf_tight_Pose_pub.publish(ekf_pose_odom); // high frequency pose 
    }

    prediction_pre_t = imu_track.header.stamp.toSec();
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
    gps_count ++;
    double ps_std = 10;
    // cout<<"GNSS raw data"<<endl;
    nlosExclusion::GNSS_Raw_Array GNSS_data = *input; 

    MatrixXd weight_matrix = cofactorMatrixCal_EKF(GNSS_data);

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
    for(int i =0; i < GNSS_data.GNSS_Raws.size(); i++) // for weighted least square
    {
            // add Gaussian noise
      std::default_random_engine generator;

      auto dist_E = std::bind(std::normal_distribution<double>{0, ps_std},
                      std::mt19937(std::random_device{}()));
      double random_psr = dist_E();

      eAllSVPositions(i,0) = GNSS_data.GNSS_Raws[i].prn_satellites_index;
      eAllSVPositions(i,1) = GNSS_data.GNSS_Raws[i].sat_pos_x;
      eAllSVPositions(i,2) = GNSS_data.GNSS_Raws[i].sat_pos_y;
      eAllSVPositions(i,3) = GNSS_data.GNSS_Raws[i].sat_pos_z;

      eAllMeasurement(i,0) = GNSS_data.GNSS_Raws[i].prn_satellites_index;
      eAllMeasurement(i,1) = GNSS_data.GNSS_Raws[i].snr;
      eAllMeasurement(i,2) = GNSS_data.GNSS_Raws[i].pseudorange ;
      // eAllMeasurement(i,2) = GNSS_data.GNSS_Raws[i].pseudorange + random_psr;

    }
    Eigen::MatrixXd  eWLSSolutionECEF; // 5 unknowns with two clock bias variables
    // eWLSSolutionECEF = LeastSquare(eAllSVPositions, eAllMeasurement);  //
    // eWLSSolutionECEF = WeightedLeastSquare(eAllSVPositions, eAllMeasurement, GNSS_data);  //WeightedLeastSquare  

    eWLSSolutionECEF = WeightedLeastSquare(eAllSVPositions, eAllMeasurement, GNSS_data);  //WeightedLeastSquare  WeightedLeastSquare_GPS

    ekf_z_t.resize(GNSS_data.GNSS_Raws.size());
    for(int i =0; i < GNSS_data.GNSS_Raws.size(); i++)
    {
      // add Gaussian noise
      std::default_random_engine generator;

      auto dist_E = std::bind(std::normal_distribution<double>{0, ps_std},
                      std::mt19937(std::random_device{}()));
      double random_psr = dist_E();

      if(PRNisGPS(GNSS_data.GNSS_Raws[i].prn_satellites_index))
      {
        // ekf_z_t(i) = GNSS_data.GNSS_Raws[i].pseudorange + ekf_state(9);

        ekf_z_t(i) = GNSS_data.GNSS_Raws[i].pseudorange;
        // ekf_z_t(i) = GNSS_data.GNSS_Raws[i].pseudorange + random_psr;

        // ekf_z_t(i) = GNSS_data.GNSS_Raws[i].pseudorange + 49801104.0936; //49801104.0936,-89240828.5583
      }
      else if( PRNisBeidou(GNSS_data.GNSS_Raws[i].prn_satellites_index))
      {
        // ekf_z_t(i) = GNSS_data.GNSS_Raws[i].pseudorange + ekf_state(10);
        ekf_z_t(i) = GNSS_data.GNSS_Raws[i].pseudorange;
        // ekf_z_t(i) = GNSS_data.GNSS_Raws[i].pseudorange + random_psr;
        // ekf_z_t(i) = GNSS_data.GNSS_Raws[i].pseudorange + (-89240828.5583); //49801104.0936,-89240828.5583
      }
    }

    // ekf_z_t << 0,0,0;
    if(online_cal_success) // imu calibration ready
    {
      VectorXd predicted_ekf_z_t; // predicted observation
      predicted_ekf_z_t.resize(GNSS_data.GNSS_Raws.size());
      H_matrix.resize(GNSS_data.GNSS_Raws.size(),11);
      //ekf_state(9) = usr_clk;
      for(int i =0; i < GNSS_data.GNSS_Raws.size(); i++) // tranverse all the satellites 
      {
        VectorXd H_row; // state:  px, py, pz, vx, vy, vz, bax, bzy, baz, bclo_GPS, bclo_BeiDou
        H_row.resize(11);
        // cout<<"construct observation matrxi"<<endl;

        double dis_x = GNSS_data.GNSS_Raws[i].sat_pos_x - ekf_state(0);
        double dis_y = GNSS_data.GNSS_Raws[i].sat_pos_y - ekf_state(1);
        double dis_z = GNSS_data.GNSS_Raws[i].sat_pos_z - ekf_state(2);


        double dis_r_s = sqrt( pow(dis_x,2) + pow(dis_y,2) + pow(dis_z,2));
        // cout<<"guessed pseudorange -> "<<dis_r_s<<"GNSS_data.GNSS_Raws[i].pseudorange-> "<<GNSS_data.GNSS_Raws[i].pseudorange <<endl;

        
        H_row(0) = -1 * (dis_x) / dis_r_s;
        H_row(1) = -1 * (dis_y) / dis_r_s;
        H_row(2) = -1 * (dis_z) / dis_r_s;

        H_row(3) = 0; // velocity 
        H_row(4) = 0;
        H_row(5) = 0;

        H_row(6) = 0; // imu bias 
        H_row(7) = 0;
        H_row(8) = 0;

        if(PRNisGPS(GNSS_data.GNSS_Raws[i].prn_satellites_index)) //PRNisGPS
        {
          H_row(9) = 1;
          H_row(10) =0;

          predicted_ekf_z_t(i) = dis_r_s + ekf_state(9);
        }
        else if( PRNisBeidou(GNSS_data.GNSS_Raws[i].prn_satellites_index)) //PRNisGPS
        {
          H_row(9) = 0;
          H_row(10) = 1;

          predicted_ekf_z_t(i) = dis_r_s + ekf_state(10);
        }

        H_matrix.row(i) = H_row;        
      }

      
      // H_matrix = (H_matrix.transpose() * weight_matrix * H_matrix).inverse()* H_matrix.transpose() * weight_matrix;

      R_matrix.resize(GNSS_data.GNSS_Raws.size(),GNSS_data.GNSS_Raws.size());
      R_matrix.setIdentity();
      R_matrix = weight_matrix ;
      // cout<<"R_matrix  -> "<<R_matrix<<endl;
      // cout<< "---------------H_matrix-----------------  \n"<<H_matrix<<endl;

      K_matrix.resize(11,GNSS_data.GNSS_Raws.size());

      // update K_matrix
      Eigen::MatrixXd I_matrix;
      I_matrix.resize(11,11);
      I_matrix.setIdentity();


      K_matrix = sigma_matrix * H_matrix.transpose() * (H_matrix * sigma_matrix * H_matrix.transpose() + R_matrix).inverse();
      // ekf_state = ekf_state + K_matrix * (ekf_z_t - H_matrix * ekf_state);
      ekf_state = ekf_state + K_matrix * (ekf_z_t -predicted_ekf_z_t);

      std::vector<double> res_vec;
      for(int res_i = 0; res_i <ekf_z_t.rows(); res_i ++)
      {
        VectorXd ekf_z_t_residual = ekf_z_t -predicted_ekf_z_t;
        res_vec.push_back(ekf_z_t_residual(res_i));
      }
      fprintf(fp_out_loose_ekf_residual, "%d ,%3.2f ,%3.2f  \n", gps_count, statis_cal(res_vec).mean, statis_cal(res_vec).std);
      // std::cout<< "statis_cal(res_vec).mean" << statis_cal(res_vec).mean <<std::endl;

      
      
      //usr_clk = usr_clk + ekf_state(9);
      sigma_matrix = (I_matrix - K_matrix * H_matrix) * sigma_matrix;
      
      std::cout << std::setprecision(12);
      // cout<< "---------------K_matrix * (ekf_z_t -predicted_ekf_z_t)-----------------  \n"<<K_matrix * (ekf_z_t -predicted_ekf_z_t)<<endl;
      // cout<< "--------------- (ekf_z_t -predicted_ekf_z_t)-----------------  \n"<< (ekf_z_t -predicted_ekf_z_t)<<endl;

      Eigen::MatrixXd enu;
      Eigen::MatrixXd ecef;
      ecef.resize(3, 1);
      ecef(0) = eWLSSolutionECEF(0);
      ecef(1) = eWLSSolutionECEF(1);
      ecef(2) = eWLSSolutionECEF(2);
      enu.resize(3, 1);
      std::cout << std::setprecision(12);
      enu = gnss_tools_.ecef2enu(originllh_span, ecef);

      //WLS GNSS LLH 
      Eigen::MatrixXd WLS_llh;
      WLS_llh.resize(3, 1);
      WLS_llh = gnss_tools_.ecef2llh(ecef);

      Eigen::MatrixXd enu2;
      Eigen::MatrixXd ecef2;
      ecef2.resize(3, 1);
      ecef2(0) = ekf_state(0); 
      ecef2(1) = ekf_state(1);
      ecef2(2) = ekf_state(2);
      enu2.resize(3, 1);
      std::cout << std::setprecision(12);
      enu2 = gnss_tools_.ecef2enu(originllh_span, ecef2);

      //Tight GNSS/IMU LLH 
      Eigen::MatrixXd GNSS_IMU_llh;
      GNSS_IMU_llh.resize(3, 1);
      GNSS_IMU_llh = gnss_tools_.ecef2llh(ecef2);

      cout<< "---------------WLS (ENU)-----------------  \n"<<enu<<endl;
      cout<< "---------------EKF (ENU)-----------------  \n"<<enu2<<endl;
      // cout<< "---------------ekf tight coupling State (ecef)--px, py, pz, vx, vy, vz, imu_bias_x, imu_bias_y, imu_bias_z, bias_clock_gps, bias_clock_beidou---------------  \n"<<ekf_state<<endl;
      std::cout << std::setprecision(3);
      // cout<< "---------------K_matrix-----------------  \n"<<K_matrix<<endl;
      // cout<< "---------------sigma_matrix-----------------  \n"<<sigma_matrix<<endl;
      //cout<< "---------------b_gps1-----------------  \n"<<ekf_state<<endl;
      std::cout << std::setprecision(12);
      // cout<<"ekf_z_t -predicted_ekf_z_t)"<<ekf_z_t -predicted_ekf_z_t<<endl;

      ekf_pose_odom.header = imu_track.header;
      ekf_pose_odom.header.frame_id = "odom";
      ekf_pose_odom.pose.pose.position.x = enu2(0);
      ekf_pose_odom.pose.pose.position.y = enu2(1);
      ekf_pose_odom.pose.pose.position.z = enu2(2);
      ekf_tight_Pose_pub.publish(ekf_pose_odom); // high frequency pose 

      WLS_pose_odom.header = imu_track.header;
      WLS_pose_odom.header.frame_id = "odom";
      WLS_pose_odom.pose.pose.position.x = enu(0);
      WLS_pose_odom.pose.pose.position.y = enu(1);
      // WLS_pose_odom.pose.pose.position.z = enu(2);
      WLS_pub.publish(WLS_pose_odom); // high frequency pose 

      if(gps_count > 10)
      {
        double error_WLS = sqrt( pow(enu(0) - span_odom.pose.pose.position.x, 2) + pow(enu(1) - span_odom.pose.pose.position.y, 2) );
        posi_err_WLS.push_back(error_WLS);
        statistical_para result_WLS = statis_cal(posi_err_WLS);
        // cout<<"result_WLS mean->  "<<result_WLS.mean<<endl;
        // cout<<"result_WLS std->  "<<result_WLS.std<<endl;
        cout<<"result_WLS mean->  "<<error_WLS<<endl;

        double error_Tight = sqrt( pow(enu2(0) - span_odom.pose.pose.position.x, 2) +  pow(enu2(1) - span_odom.pose.pose.position.y, 2) );
        posi_err_Tight.push_back(error_Tight);
        statistical_para result_Tight = statis_cal(posi_err_Tight);
        // cout<<"result_Tight mean->  "<<result_Tight.mean<<endl;
        // cout<<"result_Tight std->  "<<result_Tight.std<<endl;
        cout<<"result_Tight mean->  "<< error_Tight <<endl;

        // FILE* fp_out_tight_ekf = fopen("/home/wenws/amsipolyu/gps_imu_tight_ekf.csv", "w+");
        // fprintf(fp_out_tight_ekf, "%s ,%s ,%s ,%s\n", "epoch", "GPS_WLS_Eror", "GPS_IMU_Tight__Eror");
        //  for (int n=0; n<posi_err_WLS.size(); n++)
        // {
        //   fprintf(fp_out_tight_ekf, "%d ,%3.2f ,%3.2f \n", posi_err_WLS.size(), posi_err_WLS[n], posi_err_Tight[n]);
        // }
        // fclose(fp_out_tight_ekf);

        fprintf(fp_out_tight_ekf, "%d,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f,%3.2f \n", gps_count, error_WLS, error_Tight, span_odom.pose.pose.position.x , span_odom.pose.pose.position.y, enu(0), enu(1), enu2(0), enu2(1));
        
        std::cout << std::setprecision(12);
        fprintf(fp_out_llh_trajectory, "%d,%3.7f,%3.7f,%3.7f,%3.7f,%3.7f,%3.7f \n", gps_count, span_odom.pose.pose.orientation.x , span_odom.pose.pose.orientation.y, 
          GNSS_IMU_llh(0), GNSS_IMU_llh(1), WLS_llh(0), WLS_llh(1));

        // cout<< "posi_err_WLS.size()   "<<posi_err_WLS.size() <<endl;
        // cout<< "error_WLS   "<<error_WLS <<endl;
        // cout<< "error_Tight   "<<error_Tight <<endl;
        // cout<< "ekf_state   "<<ekf_state <<endl;

      }
      
    }
    
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
      Eigen::MatrixXd eigenENU;; // 
      eigenENU.resize(3, 1);
      eigenENU = gnss_tools_.ecef2enu(originllh_span,ecef);
      
      span_gps_week_sec = fix_msg->header.gps_week_seconds;
      span_odom.header.frame_id = "odom";
      span_odom.pose.pose.position.x = eigenENU(0);
      span_odom.pose.pose.position.y = eigenENU(1);
      span_odom.pose.pose.position.z = eigenENU(2);
      span_odom.pose.pose.orientation.x = fix_msg->latitude;
      span_odom.pose.pose.orientation.y = fix_msg->longitude;
      span_odom.pose.pose.orientation.z = fix_msg->altitude;
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
  Eigen::MatrixXd WeightedLeastSquare_GPS(Eigen::MatrixXd eAllSVPositions, Eigen::MatrixXd eAllMeasurement, nlosExclusion::GNSS_Raw_Array GNSS_data){
  
    Eigen::MatrixXd eWLSSolution;
    eWLSSolution.resize(4, 1);

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
      std::cout<<"satellite number is not enough" <<std::endl;
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
        }
        // Making delta pseudorange
        double rcv_clk_bias;
        if (PRNisGPS(prn)){
          rcv_clk_bias = eWLSSolution(3);       
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
      {
        bWLSConverge = true;
        std::cout<<" more than 6 times in iterations"<<std::endl;
      }
    }
    // printf("WLS -> (%11.2f,%11.2f,%11.2f)\n\n", eWLSSolution(0), eWLSSolution(1), eWLSSolution(2));
    std::cout << std::setprecision(12);
    cout<< "---------------WLS (ECEF) x, y, z, bias_gps, bias_beidou-----------------  \n"<<eWLSSolution<<endl;

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
  ros::init(argc, argv, "ekf_tight");
  std::cout<<"ekf_tight......"<<std::endl;

  // printf("obss.n = %d\n", obss.n);
  ekf_tight ekf_tight_(1);
  ros::spin();
  while (ros::ok()) {
  }
  return 0;
}