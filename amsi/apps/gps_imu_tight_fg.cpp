/* ----------------------------------------------------------------------------

 * amsi Copyright 2019, Positioning and Navigation Laboratory,
 * Hong Kong Polytechnic University
 * All Rights Reserved
 * Authors: Weisong Wen, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file imuFactorsExample
 * @brief Test example for using GTSAM ImuFactor and ImuCombinedFactor navigation code.
 * @author Weisong Wen (weisong.wen@connect.polyu.hk)
 */

/**
 * Example of use of the imuFactors (imuFactor and combinedImuFactor) in conjunction with GPS
 *  - you can test imuFactor (resp. combinedImuFactor) by commenting (resp. uncommenting)
 *  the line #define USE_COMBINED (few lines below)
 *  - we read IMU and GPS data from a CSV file, with the following format:
 *  A row starting with "i" is the first initial position formatted with
 *  N, E, D, qx, qY, qZ, qW, velN, velE, velD
 *  A row starting with "0" is an imu measurement
 *  linAccN, linAccE, linAccD, angVelN, angVelE, angVelD
 *  A row starting with "1" is a gps correction formatted with
 *  N, E, D, qX, qY, qZ, qW
 *  Note that for GPS correction, we're only using the position not the rotation. The
 *  rotation is provided in the file for ground truth comparison.
 */

// GTSAM related includes.
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h> // DoglegOptimizer
#include <gtsam/nonlinear/DoglegOptimizer.h> // DoglegOptimizer

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/geometry/Point2.h>


#include <gtsam/nonlinear/ISAM2.h>


#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

//time 
#include <time.h>

// ros and math related head files
#include <vector>
#include <ros/ros.h>


#include <amsi/gnss_tools.hpp>

#include <nmea_msgs/Sentence.h> // message from DGPS of University of California, Berkeley.
#include <sensor_msgs/NavSatFix.h>
#include <Eigen/Dense>
#include <novatel_msgs/BESTPOS.h> // novatel_msgs/INSPVAX

#include <random> // gaussian noise

#include <amsi/gnss_tools.hpp>

#include <nlosExclusion/GNSS_Raw_Array.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>
#include <gtsam/gnssNavigation/PseudorangeFactor_spp.h>




// Uncomment line below to use the CombinedIMUFactor as opposed to the standard ImuFactor.
// #define USE_COMBINED

using namespace gtsam;
using namespace std;
// using namespace Eigen;
#define INF 10000
#define pi 3.1415926
#define D2R 3.1415926/180.0

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::C; // Point2 receiver clock bias  (cb_g,cb_b)

const string output_filename = "imuFactorExampleResults.csv";

FILE* gps_imu_loose_fg_result = fopen("/home/wenws/amsipolyu/IONGNSS2019_result/gps_imu_loose_fg_result.csv", "w+");


// This will either be PreintegratedImuMeasurements (for ImuFactor) or
// PreintegratedCombinedMeasurements (for CombinedImuFactor).
PreintegrationType *imu_preintegrated_;
sensor_msgs::Imu imu_track;
bool imu_up =0 ;
int imu_co=0;

bool gps_up = 0;
nav_msgs::Odometry odom_track,span_odom, odom_track_ENU, span_odom_ecef;

// gnss_tools
GNSS_Tools gnss_tools_;
Eigen::MatrixXd originllh,originllh_span; // origin llh
Eigen::MatrixXd referencellh; // origin llh
sensor_msgs::NavSatFix ini_navf,ini_navf_span ; // initial sensor msg
// referencellh.resize(3, 1);
// // start point of robot
// referencellh(0) = 116.4986266; //116.4986357 (pre-test) 116.4986266 (semi-final compet ) 116.4986473 (semi-final test )
// referencellh(1) = 39.7917427; // 39.7917481 ( pre-test) 39.7917427 (semi-final  compet ) 39.7917502 (semi-final  test )
// referencellh(2) = 22.1009979248; // 116.4986266 (semi-final )

ros::Subscriber span_BP_sub;
ros::Publisher span_odom_pub;

nlosExclusion::GNSS_Raw_Array _GNSS_data; // save the GNSS data 

Point3 nomXYZ(-2418954.90428,5384679.60663,2407391.40139);


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


imu_para imu_parameter; // online calibrated imu parameters using begin 2000 frames of imu raw measurements
bool use_online_imu_cal = 0;
bool online_cal_success = 0;
bool offline_cal_success = 0;

vector<sensor_msgs::Imu> imu_queue;
int imu_queue_size = 10;

double gps_cov = 0;
double psr_cov = 0;

double nstamp = 0;

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
   * @brief imu callback
   * @param imu msg
   * @return void
   * calibrate using the first several epochs, the gravity can be minused
   @ 
   */
  void imu_callback(const sensor_msgs::Imu::Ptr& input) // 
  {
    double g_ = 1;
    // std::cout << " IMU data call back" << input->angular_velocity.x << std::endl;
    imu_track = * input;

    // imu_track.linear_acceleration.x = imu_track.linear_acceleration.x * g_;
    // imu_track.linear_acceleration.y = imu_track.linear_acceleration.y * g_;
    // imu_track.linear_acceleration.z = imu_track.linear_acceleration.z * g_;

    /****in the tightly coupled GNSS/INS integration, navigation frame is ECEF***/
    if(originllh_span.rows() > 1)
    {
      double lon = (double)originllh_span(0) * D2R;
      double lat = (double)originllh_span(1) * D2R;


      double e = imu_track.linear_acceleration.x; // acc in enu 
      double n = imu_track.linear_acceleration.y;
      double u = imu_track.linear_acceleration.z;


      imu_track.linear_acceleration.x = 0 - sin(lon) * e - cos(lon) * sin(lat) * n + cos(lon) * cos(lat) * u; // transfrom the acc from enu to ecef
      imu_track.linear_acceleration.y = 0 + cos(lon) * e - sin(lon) * sin(lat) * n + cos(lat) * sin(lon) * u;
      imu_track.linear_acceleration.z = 0 + cos(lat) * n + sin(lat) * u;
    }
    
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
      imu_track.angular_velocity.z = imu_track.angular_velocity.z - (-0.00109033);

      imu_track.linear_acceleration.x = imu_track.linear_acceleration.x - (-0.0223799);
      imu_track.linear_acceleration.y = imu_track.linear_acceleration.y - (-0.1048587);
      imu_track.linear_acceleration.z = imu_track.linear_acceleration.z - (9.791944);
      offline_cal_success = 1;
    }
    imu_up =1;
    imu_co++;
  }

void imu_callback1(const sensor_msgs::Imu::Ptr& input) // the pre-integration needs the data in NED, but the imu data is in ENU
{
  // std::cout << " IMU data call back" << input->angular_velocity.x << std::endl;
  imu_track = * input;
  // decrease the bias 
  // imu_track.angular_velocity.x = imu_track.angular_velocity.x - (-0.00231128);
  // imu_track.angular_velocity.y = imu_track.angular_velocity.y - (0.0019349);
  // imu_track.angular_velocity.z = imu_track.angular_velocity.z - (-0.000309033);

  // imu_track.linear_acceleration.x = imu_track.linear_acceleration.x - (-0.0000563799);
  // imu_track.linear_acceleration.y = imu_track.linear_acceleration.y - (-0.0004587);
  // imu_track.linear_acceleration.z = imu_track.linear_acceleration.z - (0.0979159);

  // in ENU
  imu_track.angular_velocity.x = imu_track.angular_velocity.x - (-0.00303897);
  imu_track.angular_velocity.y = imu_track.angular_velocity.y - (0.00170488);
  imu_track.angular_velocity.z = imu_track.angular_velocity.z - (0.000641023);

  imu_track.linear_acceleration.x = imu_track.linear_acceleration.x - (-0.00224483);
  imu_track.linear_acceleration.y = imu_track.linear_acceleration.y - (0.000396544);
  // imu_track.linear_acceleration.z = imu_track.linear_acceleration.z - (0.0978841);
  imu_track.linear_acceleration.z = 0;


  // in NED
  // imu_track.angular_velocity.y = imu_track.angular_velocity.x - (-0.00303897);
  // imu_track.angular_velocity.x = imu_track.angular_velocity.y - (0.00170488);
  // imu_track.angular_velocity.z = -1 * (imu_track.angular_velocity.z - (0.000641023));

  // imu_track.linear_acceleration.y = imu_track.linear_acceleration.x - (-0.00224483);
  // imu_track.linear_acceleration.x = imu_track.linear_acceleration.y - (0.000396544);
  // imu_track.linear_acceleration.z = -1 * (imu_track.linear_acceleration.z - (0.0978841));



  imu_up =1;
  imu_co++;

}

void odom_callback(const nav_msgs::Odometry::ConstPtr& input)
{
  //std::cout << __func__ << std::endl;

  // odom_track = *input;
  // gps_up = 1;
}

void ubloxFix_callback(const sensor_msgs::NavSatFixConstPtr& fix_msg)
  {
    // cout<<"jd /fix received "<<endl;
    sensor_msgs::NavSatFix navfix_ ;
    navfix_.header = fix_msg->header;
    navfix_.latitude = fix_msg->latitude;
    navfix_.longitude = fix_msg->longitude;
    navfix_.altitude = fix_msg->altitude;

    if(ini_navf.latitude == NULL)
      {
        ini_navf = navfix_;
        std::cout<<"ini_navf.header  -> "<<ini_navf.header<<std::endl;
        originllh.resize(3, 1);
        originllh(0) = navfix_.longitude;
        originllh(1) = navfix_.latitude;
        originllh(2) = navfix_.altitude;
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
      // odom_track.pose.pose.position.x = eigenENU(0);
      // odom_track.pose.pose.position.y = eigenENU(1);
      // add Gaussian noise
      std::default_random_engine generator;


      auto dist_E = std::bind(std::normal_distribution<double>{0, 3},
                      std::mt19937(std::random_device{}()));
      double random_E = dist_E();

      auto dist_N = std::bind(std::normal_distribution<double>{0, 4},
                      std::mt19937(std::random_device{}()));
      double random_N = dist_N();

      // odom_track.pose.pose.position.x = eigenENU(0) + random_E;
      // odom_track.pose.pose.position.y = eigenENU(1) + random_N;
      
      std::cout<<"distribution_E -> "<< random_E<<std::endl;

      odom_track.pose.pose.position.z = 0;
      odom_track.pose.pose.orientation.x = 0.03;
      odom_track.pose.pose.orientation.y = 0.01;
      odom_track.pose.pose.orientation.z = 0.04;
      odom_track.pose.pose.orientation.w = 0.012;
      if(originllh_span.size()) // initial llh from span-cpt is available 
        gps_up = 1;
      
      
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
  Eigen::MatrixXd cofactorMatrixCal_EKF(nlosExclusion::GNSS_Raw_Array GNSS_data)
  {
    double snr_1 = 50.0; // T = 50
    double snr_A = 30.0; // A = 30
    double snr_a = 30.0;// a = 30
    double snr_0 = 10.0; // F = 10
    Eigen::VectorXd cofactor_;  // cofactor of satellite
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

    Eigen::MatrixXd weight_matrix;
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
  Eigen::MatrixXd cofactorMatrixCal_WLS(nlosExclusion::GNSS_Raw_Array GNSS_data)
  {
    double snr_1 = 50.0; // T = 50
    double snr_A = 30.0; // A = 30
    double snr_a = 30.0;// a = 30
    double snr_0 = 10.0; // F = 10
    Eigen::VectorXd cofactor_;  // cofactor of satellite
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

    Eigen::MatrixXd weight_matrix;
    weight_matrix.resize(GNSS_data.GNSS_Raws.size(),GNSS_data.GNSS_Raws.size());
    weight_matrix.setIdentity();
    for(int k = 0; k < weight_matrix.rows(); k++)
    {
      weight_matrix.row(k) = weight_matrix.row(k) * cofactor_(k);
    }

    return weight_matrix;
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

    Eigen::MatrixXd weight_matrix = cofactorMatrixCal_WLS(GNSS_data);

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
    nstamp ++;
    double ps_std = 10;
    // cout<<"GNSS raw data"<<endl;
    nlosExclusion::GNSS_Raw_Array GNSS_data = *input; 
    _GNSS_data = GNSS_data;
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
      // eAllMeasurement(i,2) = GNSS_data.GNSS_Raws[i].pseudorange + random_psr;
      eAllMeasurement(i,2) = GNSS_data.GNSS_Raws[i].pseudorange ;

      // std::cout<<"span_ecef -> "<<span_ecef<<std::endl; 
    }
    Eigen::MatrixXd  eWLSSolutionECEF; // 5 unknowns with two clock bias variables
    // eWLSSolutionECEF = LeastSquare(eAllSVPositions, eAllMeasurement);  //
    eWLSSolutionECEF = WeightedLeastSquare(eAllSVPositions, eAllMeasurement, GNSS_data);  //WeightedLeastSquare

    Eigen::MatrixXd ecef;
    Eigen::MatrixXd enu_wls;
    enu_wls.resize(3, 1);
    ecef.resize(3, 1);
    ecef(0) = eWLSSolutionECEF(0);
    ecef(1) = eWLSSolutionECEF(1);
    ecef(2) = eWLSSolutionECEF(2);
    enu_wls = gnss_tools_.ecef2enu(originllh_span, ecef);

    odom_track.header.frame_id = "odom";

    
    odom_track_ENU.header.frame_id = "odom";

    // if(((int)nstamp%100) == 0)
    // {
    //   random_N = 35;
    //   random_E = 35;
    // }

    Eigen::MatrixXd weight_matrix = cofactorMatrixCal_WLS(GNSS_data);
    psr_cov = 0;
    for(int m = 0;  m< weight_matrix.cols(); m++)
    {
      psr_cov = psr_cov + 1.0 / weight_matrix(m,m);
    }
    psr_cov = psr_cov / weight_matrix.cols();
    psr_cov = psr_cov/ 100.0;

    odom_track.pose.pose.position.x = ecef(0);
    odom_track.pose.pose.position.y = ecef(1);
    odom_track.pose.pose.position.z = ecef(2);

    odom_track_ENU.pose.pose.position.x = enu_wls(0);
    odom_track_ENU.pose.pose.position.y = enu_wls(1);
    odom_track_ENU.pose.pose.position.z = enu_wls(2);

    // odom_track.pose.pose.position.x = enu_wls(0) + random_E;
    // odom_track.pose.pose.position.y = enu_wls(1) + random_N;


    std::cout<< "enu_wls(0) -> "<< enu_wls(0) <<"enu_wls(1)-> "<< enu_wls(1) <<"psr_cov -> " <<psr_cov <<  std::endl;

    if(originllh_span.size()) // initial llh from span-cpt is available 
        gps_up = 1;
  }

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
        std::cout<<"ini_navf_span.header  -> "<<ini_navf_span.header<<std::endl;
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
      span_odom.pose.pose.position.z = 0;
      span_odom.pose.pose.orientation.x = 0.03;
      span_odom.pose.pose.orientation.y = 0.01;
      span_odom.pose.pose.orientation.z = 0.04;
      span_odom.pose.pose.orientation.w = 0.012;
      span_odom_pub.publish(span_odom);

      span_odom_ecef = span_odom;
      span_odom_ecef.pose.pose.position.x = ecef(0);
      span_odom_ecef.pose.pose.position.y = ecef(1);
      span_odom_ecef.pose.pose.position.z = ecef(2);
      span_odom_ecef.pose.pose.orientation.x = 0.03;
      span_odom_ecef.pose.pose.orientation.y = 0.01;
      span_odom_ecef.pose.pose.orientation.z = 0.04;
      span_odom_ecef.pose.pose.orientation.w = 0.012;

      
      
  }


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "GNSSINS2NonLinear");
  std::cout<<"GNSSINS2NonLinear node......"<<std::endl;

  ros::NodeHandle nh;

  ros::Subscriber param_sub = nh.subscribe("/imu/data", 50, imu_callback); // imu data from Xsens Mti 10
  ros::Subscriber odom_sub = nh.subscribe("/Odom_0", 50, odom_callback); //  odom information from Zhixingzhe
  ros::Publisher factorGraphPose_pub = nh.advertise<nav_msgs::Odometry>("/factorGraphPose", 10); // fused result
  ros::Publisher GNSS_odom_pub = nh.advertise<nav_msgs::Odometry>("/gnss_odom", 10); // GNSS in ENU from WLS
  ros::Subscriber jdFix_sub =nh.subscribe("/ublox_gps_node/fix2", 50, ubloxFix_callback); // subscribe data from ublox solution
  span_BP_sub =nh.subscribe("/novatel_data/bestpos", 50, span_bp_callback); // ground truth position from span-cpt
  span_odom_pub = nh.advertise<nav_msgs::Odometry>("/span_odom", 10); // publish the span-cpt solution in ENU.

  ros::Subscriber gps_raw_sub = nh.subscribe("/GNSS_", 50, GNSS_raw_callback); // subscribe the raw GNSS data

  ros::Rate rate(20);
  // while(ros::ok()){
  if(1){
        // ros::spin();
        // rate.sleep();
      string data_filename;
  if (argc < 2) {
    // printf("using default CSV file\n");
    data_filename = findExampleDataFile("imuAndGPSdata.csv");
  } else {
    data_filename = argv[1];
  }

  // Set up output file for plotting errors
  FILE* fp_out = fopen(output_filename.c_str(), "w+");
  fprintf(fp_out, "#time(s),x(m),y(m),z(m),qx,qy,qz,qw,gt_x(m),gt_y(m),gt_z(m),gt_qx,gt_qy,gt_qz,gt_qw\n");

  // Begin parsing the CSV file.  Input the first line for initialization.
  // From there, we'll iterate through the file and we'll preintegrate the IMU
  // or add in the GPS given the input.
  ifstream file(data_filename.c_str());
  string value;

  /************Step 1: Get initial state***************/
  // Format is (N,E,D,qX,qY,qZ,qW,velN,velE,velD)
  Eigen::Matrix<double,10,1> initial_state = Eigen::Matrix<double,10,1>::Zero();
  getline(file, value, ','); // i
  for (int i=0; i<9; i++) {
    getline(file, value, ',');
    initial_state(i) = atof(value.c_str());
  }
  getline(file, value, '\n');
  initial_state(9) = atof(value.c_str());
  // cout << "initial state:\n" << initial_state.transpose() << "\n\n";

  // Assemble initial quaternion through gtsam constructor ::quaternion(w,x,y,z);
  Rot3 prior_rotation = Rot3::Quaternion(initial_state(6), initial_state(3), 
                                         initial_state(4), initial_state(5));
  // Point3 prior_point(initial_state.head<3>());
  Point3 prior_point = nomXYZ;
  Pose3 prior_pose(prior_rotation, prior_point);
  Vector3 prior_velocity(initial_state.tail<3>());
  imuBias::ConstantBias prior_imu_bias; // assume zero initial bias
  Point2 prior_cb(0,0); // assume zero initial clock bias/
  // Point2 prior_cb(56098484.0491,-98362089.033); // assume zero initial clock bias

  Values initial_values;
  int correction_count = 0;
  initial_values.insert(X(correction_count), prior_pose);
  initial_values.insert(V(correction_count), prior_velocity);
  initial_values.insert(B(correction_count), prior_imu_bias);  
  initial_values.insert(C(correction_count), prior_cb);  

  // Assemble prior noise model and add it the graph.
  noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
  noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.1); // m/s
  noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);
  noiseModel::Diagonal::shared_ptr cb_noise_model = noiseModel::Diagonal::Sigmas((Vector(2) << 1, 1).finished()); //  m, m noise model for clock noise

  // Add all prior factors (pose, velocity, bias) to the graph.
  NonlinearFactorGraph *graph = new NonlinearFactorGraph();

  // Create an iSAM2 object. Unlike iSAM1, which performs periodic batch steps to maintain proper linearization
  // and efficient variable ordering, iSAM2 performs partial relinearization/reordering at each step. A parameter
  // structure is available that allows the user to set various properties, such as the relinearization threshold
  // and type of linear solver. For this example, we we set the relinearization threshold small so the iSAM2 result
  // will approach the batch result.
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  ISAM2 isam(parameters);

  graph->add(PriorFactor<Pose3>(X(correction_count), prior_pose, pose_noise_model));
  graph->add(PriorFactor<Vector3>(V(correction_count), prior_velocity,velocity_noise_model));
  graph->add(PriorFactor<imuBias::ConstantBias>(B(correction_count), prior_imu_bias,bias_noise_model));
  graph->add(PriorFactor<Point2>(C(correction_count), prior_cb,cb_noise_model)); // add prior for clock bias factor

  // We use the sensor specs to build the noise model for the IMU factor.
  
  // gievn in GTSAM example
  // double accel_noise_sigma = 0.0003924;
  // double gyro_noise_sigma = 0.000205689024915;
  // double accel_bias_rw_sigma = 0.004905;
  // double gyro_bias_rw_sigma = 0.000001454441043;

  // Xsens 100 Ti
  // double accel_noise_sigma = 0.03924;
  // double gyro_noise_sigma = 0.0205689024915;

  double accel_noise_sigma = 0.1624;
  double gyro_noise_sigma = 0.205689024915;

  double accel_bias_rw_sigma = 0.004905;
  double gyro_bias_rw_sigma = 0.000001454441043;

  Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
  Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
  Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-8; // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
  Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
  Matrix66 bias_acc_omega_int = Matrix::Identity(6,6)*1e-5; // error in the bias used for preintegration

  boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
  // PreintegrationBase params:
  p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
  p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_int;
  
#ifdef USE_COMBINED
  imu_preintegrated_ = new PreintegratedCombinedMeasurements(p, prior_imu_bias);
#else
  imu_preintegrated_ = new PreintegratedImuMeasurements(p, prior_imu_bias);
#endif 

  // Store previous state for the imu integration and the latest predicted outcome.
  NavState prev_state(prior_pose, prior_velocity);
  NavState prop_state = prev_state;
  imuBias::ConstantBias prev_bias = prior_imu_bias;
  Point2 prev_cb = prior_cb;

  // Keep track of the total error over the entire run for a simple performance metric.
  double current_position_error = 0.0, current_orientation_error = 0.0;

  double output_time = 0.0;
  double dt = 0.005;  // The real system has noise, but here, results are nearly 
                      // exactly the same, so keeping this for simplicity.

  // All priors have been set up, now iterate through the data file.
  ros::Rate rate(10); // usually ok when rate(10)
  while (ros::ok()) { //ros::ok()
    ros::spinOnce();
    rate.sleep();
    GNSS_odom_pub.publish(odom_track_ENU);
    if ((imu_up == 1) && ( (online_cal_success ==1) || (offline_cal_success ==1))) { // IMU measurement obtained and do pre-integration
      // cout<<"imu_up-> "<<imu_up<<endl;
      Eigen::Matrix<double,6,1> imu = Eigen::Matrix<double,6,1>::Zero();

      imu(0) = imu_track.linear_acceleration.x ;
      imu(1) = imu_track.linear_acceleration.y ;
      imu(2) = imu_track.linear_acceleration.z ;

      imu(3) = imu_track.angular_velocity.x ;
      imu(4) = imu_track.angular_velocity.y ;
      imu(5) = imu_track.angular_velocity.z ;

      // Adding the IMU preintegration.
      imu_preintegrated_->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);
      imu_up = 0;

    } 
    // GPS measurement obtained and add gps factor and do optimization
     if ((gps_up == 1) && ( (online_cal_success ==1) || (offline_cal_success ==1))) { 
     // if (imu_co > 20) { // imu  measurement
      // cout<<"gps_up-> "<<gps_up<<endl;
      Eigen::Matrix<double,7,1> gps = Eigen::Matrix<double,7,1>::Zero();
      
      gps(0) = odom_track.pose.pose.position.x;
      gps(1) = odom_track.pose.pose.position.y;
      gps(2) = odom_track.pose.pose.position.z;
      gps(3) = odom_track.pose.pose.orientation.x;
      gps(4) = odom_track.pose.pose.orientation.y;
      gps(5) = odom_track.pose.pose.orientation.z;
      gps(6) = 1;

      // gps(0) = span_odom_ecef.pose.pose.position.x;
      // gps(1) = span_odom_ecef.pose.pose.position.y;
      // gps(2) = span_odom_ecef.pose.pose.position.z;
      // gps(3) = span_odom_ecef.pose.pose.orientation.x;
      // gps(4) = span_odom_ecef.pose.pose.orientation.y;
      // gps(5) = span_odom_ecef.pose.pose.orientation.z;
      // gps(6) = 1;


      correction_count++;
      gps_up = 0;
      imu_co = 0;

      // Adding IMU factor and GPS factor and optimizing.
#ifdef USE_COMBINED
      PreintegratedCombinedMeasurements *preint_imu_combined = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated_);
      CombinedImuFactor imu_factor(X(correction_count-1), V(correction_count-1),
                                   X(correction_count  ), V(correction_count  ),
                                   B(correction_count-1), B(correction_count  ),
                                   *preint_imu_combined);
      graph->add(imu_factor);
#else
      PreintegratedImuMeasurements *preint_imu = dynamic_cast<PreintegratedImuMeasurements*>(imu_preintegrated_);
      ImuFactor imu_factor(X(correction_count-1), V(correction_count-1),
                           X(correction_count  ), V(correction_count  ),
                           B(correction_count-1),
                           *preint_imu);
      graph->add(imu_factor);
      imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
      graph->add(BetweenFactor<imuBias::ConstantBias>(B(correction_count-1), 
                                                      B(correction_count  ), 
                                                      zero_bias, bias_noise_model));
#endif

      //noiseModel::Isotropic::Sigma(3,0.01)
      // noiseModel::Diagonal::shared_ptr correction_noise = noiseModel::Isotropic::Sigma(3, 0.1); // 3,0.01   gps(2) good and the performance is improved
      // noiseModel::Diagonal::shared_ptr correction_noise = noiseModel::Isotropic::Sigma(3, 0.1); // 3,0.01   gps(2)
      // std::cout<<"cov-> " << gps_cov/10 <<std::endl;
      noiseModel::Diagonal::shared_ptr correction_noise = noiseModel::Isotropic::Sigma(3, psr_cov ); // 3,0.01   gps(2)
      GPSFactor gps_factor(X(correction_count),
                           Point3(gps(0),  // N,  
                                  gps(1),  // E,
                                  gps(2)), // D,
                           correction_noise);
      // graph->add(gps_factor);

      for(int i =0; i < _GNSS_data.GNSS_Raws.size(); i++) // for weighted least square
    {
      // add Gaussian noise
      std::default_random_engine generator;


      auto dist_E = std::bind(std::normal_distribution<double>{0, 10},
                      std::mt19937(std::random_device{}()));
      double random_psr = dist_E();

      // std::cout<<"span_ecef -> "<<span_ecef<<std::endl; 

      // Add constraint for pseudorange observable
      double sv_prn = _GNSS_data.GNSS_Raws[i].prn_satellites_index;
      Point3 satXYZ(_GNSS_data.GNSS_Raws[i].sat_pos_x, _GNSS_data.GNSS_Raws[i].sat_pos_y, _GNSS_data.GNSS_Raws[i].sat_pos_z);
      double range = _GNSS_data.GNSS_Raws[i].pseudorange;
      double SV_weight = 1.0 / cofactorMatrixCal_single_satellite(_GNSS_data.GNSS_Raws[i].elevation, _GNSS_data.GNSS_Raws[i].snr);
      SV_weight = SV_weight* 10; // 10 

      // Add clock bias prior factor for pseudorange observable
      // graph->add(PriorFactor<Point2>(C(correction_count), prev_cb,cb_noise_model)); // add prior for clock bias factor

      PseudorangeFactor_spp pseudorange_Factor_spp(X(correction_count), C(correction_count),range, satXYZ, sv_prn, nomXYZ, noiseModel::Diagonal::Sigmas( (gtsam::Vector(1) << SV_weight).finished()));
      graph->add(pseudorange_Factor_spp);
      std::cout<< "pseudorange weighting-> "<< SV_weight << std::endl;
    }
      

      /***********add clock bias factor************/
      Point2 zero_cb(0,0); // assume zero initial clock bias
      // graph->add(BetweenFactor<Point2>(C(correction_count-1), 
      //                                                 C(correction_count  ), 
      //                                                 zero_cb, cb_noise_model));
      
      
      // Now optimize and compare results.
      prop_state = imu_preintegrated_->predict(prev_state, prev_bias);
      initial_values.insert(X(correction_count), prop_state.pose());
      initial_values.insert(V(correction_count), prop_state.v());
      initial_values.insert(B(correction_count), prev_bias);
      initial_values.insert(C(correction_count), prev_cb); // clock bias 

      // GaussNewtonOptimizer optimizer(*graph, initial_values);
      // const clock_t begin_time = clock();
      // Values result = optimizer.optimize();
      // std::cout << "this optimizer used  time -> " << float(clock() - begin_time) / CLOCKS_PER_SEC << "\n\n";

      
      


      // LevenbergMarquardtOptimizer optimizer(*graph, initial_values); // LevenbergMarquardtOptimizer
      // const clock_t begin_time = clock(); 
      // DoglegOptimizer optimizer(*graph, initial_values); // DoglegOptimizer
      // Values result = optimizer.optimize();
      // cout << "****************************************************" << endl;
      // std::cout << "this optimizer used  time -> " << float(clock() - begin_time) / CLOCKS_PER_SEC << "\n\n";


      const clock_t begin_time = clock();
      isam.update(*graph,initial_values);
      isam.update();
      Values result = isam.calculateEstimate();
      
      // result.print("Current estimate: ");
      graph->resize(0);
      initial_values.clear();
      std::cout << "this optimizer used  time -> " << float(clock() - begin_time) / CLOCKS_PER_SEC << "\n\n";



      // Overwrite the beginning of the preintegration for the next step.
      prev_state = NavState(result.at<Pose3>(X(correction_count)),
                            result.at<Vector3>(V(correction_count)));
      prev_bias = result.at<imuBias::ConstantBias>(B(correction_count));

      prev_cb = result.at<Point2>(C(correction_count));
      std::cout<< "prev_cb-> " << prev_cb << std::endl;

      // Reset the preintegration object.
      imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);

      // Print out the position and orientation error for comparison.
      Vector3 gtsam_position = prev_state.pose().translation();

      Eigen::MatrixXd ecef_fg;
      Eigen::MatrixXd enu_fg;
      enu_fg.resize(3, 1);
      ecef_fg.resize(3, 1);
      ecef_fg(0) = gtsam_position(0);
      ecef_fg(1) = gtsam_position(1);
      ecef_fg(2) = gtsam_position(2);
      enu_fg = gnss_tools_.ecef2enu(originllh_span, ecef_fg);
      gtsam_position = enu_fg;

      // cout<<"current state ->" <<gtsam_position<<endl;
      nav_msgs::Odometry facgrapose = odom_track;
      facgrapose.header.frame_id = "odom";
      facgrapose.pose.pose.position.x = gtsam_position(0);
      facgrapose.pose.pose.position.y = gtsam_position(1);
      facgrapose.pose.pose.position.z = gtsam_position(2);
      factorGraphPose_pub.publish(facgrapose);
      Vector3 position_error = gtsam_position - gps.head<3>();
      current_position_error = position_error.norm();

      Quaternion gtsam_quat = prev_state.pose().rotation().toQuaternion();
      Quaternion gps_quat(gps(6), gps(3), gps(4), gps(5));
      Quaternion quat_error = gtsam_quat * gps_quat.inverse();
      quat_error.normalize();
      Vector3 euler_angle_error(quat_error.x()*2,
                                 quat_error.y()*2,
                                 quat_error.z()*2);
      current_orientation_error = euler_angle_error.norm();

      // display statistics
      // cout << "Position error:" << current_position_error << "\t " << "Angular error:" << current_orientation_error << "\n";
      // cout<<"size of the graph ->"<<graph->size()<<endl;
      fprintf(fp_out, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", 
              output_time, gtsam_position(0), gtsam_position(1), gtsam_position(2),
              gtsam_quat.x(), gtsam_quat.y(), gtsam_quat.z(), gtsam_quat.w(),
              gps(0), gps(1), gps(2), 
              gps_quat.x(), gps_quat.y(), gps_quat.z(), gps_quat.w());
      // print the result 
      double del_x = span_odom.pose.pose.position.x - gtsam_position(0);
      double del_y = span_odom.pose.pose.position.y - gtsam_position(1);
      double del_z = span_odom.pose.pose.position.z - gtsam_position(2);
      double error_2D_fg = sqrt(pow(del_x, 2) + pow(del_y, 2));
      std::cout<<"error_2D_fg -> "<< error_2D_fg << std::endl;

      double del_x_fix = span_odom.pose.pose.position.x - odom_track_ENU.pose.pose.position.x;
      double del_y_fix = span_odom.pose.pose.position.y - odom_track_ENU.pose.pose.position.y;
      double del_z_fix = span_odom.pose.pose.position.z - odom_track_ENU.pose.pose.position.z;
      double error_2D_fix = sqrt(pow(del_x_fix, 2) + pow(del_y_fix, 2));
      std::cout<<"error_2D_fix -> "<< error_2D_fix << std::endl;

      fprintf(gps_imu_loose_fg_result, "%3.2f,%3.5f,%3.5f,%3.5f,%3.5f,%3.5f,%3.5f,%3.5f,%3.5f\n", output_time, 
            error_2D_fix,error_2D_fg, span_odom.pose.pose.position.x, span_odom.pose.pose.position.y, odom_track_ENU.pose.pose.position.x,odom_track_ENU.pose.pose.position.y,gtsam_position(0), gtsam_position(1)); // generateData_gt3    not always span_ecef available at the first epoch

      output_time += 1.0; 

      cout << "****************************************************" << endl << endl << endl;

    } 
    // else {
    //   cerr << "waiting for gps and imu data\n";
    //   // return 1;
    // }
  }
  fclose(fp_out);
  // cout << "Complete, results written to " << output_filename << "\n\n";;
  }
  

  
  return 0;
}
