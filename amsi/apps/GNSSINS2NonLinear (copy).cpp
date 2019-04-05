/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file imuFactorsExample
 * @brief Test example for using GTSAM ImuFactor and ImuCombinedFactor navigation code.
 * @author Garrett (ghemann@gmail.com), Luca Carlone
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
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

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

// Uncomment line below to use the CombinedIMUFactor as opposed to the standard ImuFactor.
// #define USE_COMBINED

using namespace gtsam;
using namespace std;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

const string output_filename = "imuFactorExampleResults.csv";

// This will either be PreintegratedImuMeasurements (for ImuFactor) or
// PreintegratedCombinedMeasurements (for CombinedImuFactor).
PreintegrationType *imu_preintegrated_;
sensor_msgs::Imu imu_track;
bool imu_up =0 ;

bool gps_up = 0;
nav_msgs::Odometry odom_track;

void imu_callback(const sensor_msgs::Imu::Ptr& input)
{
  // std::cout << " IMU data call back" << input->angular_velocity.x << std::endl;
  imu_track = * input;
  imu_up =1;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& input)
{
  //std::cout << __func__ << std::endl;

  odom_track = *input;
  gps_up = 1;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "GNSSINS2NonLinear");
  std::cout<<"GNSSINS2NonLinear node......"<<std::endl;

  ros::NodeHandle nh;

  ros::Subscriber param_sub = nh.subscribe("/imu_rt", 50, imu_callback);
  ros::Subscriber odom_sub = nh.subscribe("/Odom_0", 50, odom_callback);
  ros::Publisher factorGraphPose_pub = nh.advertise<nav_msgs::Odometry>("/factorGraphPose", 10);
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
  Point3 prior_point(initial_state.head<3>());
  Pose3 prior_pose(prior_rotation, prior_point);
  Vector3 prior_velocity(initial_state.tail<3>());
  imuBias::ConstantBias prior_imu_bias; // assume zero initial bias

  Values initial_values;
  int correction_count = 0;
  initial_values.insert(X(correction_count), prior_pose);
  initial_values.insert(V(correction_count), prior_velocity);
  initial_values.insert(B(correction_count), prior_imu_bias);  

  // Assemble prior noise model and add it the graph.
  noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
  noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.1); // m/s
  noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);

  // Add all prior factors (pose, velocity, bias) to the graph.
  NonlinearFactorGraph *graph = new NonlinearFactorGraph();
  graph->add(PriorFactor<Pose3>(X(correction_count), prior_pose, pose_noise_model));
  graph->add(PriorFactor<Vector3>(V(correction_count), prior_velocity,velocity_noise_model));
  graph->add(PriorFactor<imuBias::ConstantBias>(B(correction_count), prior_imu_bias,bias_noise_model));

  // We use the sensor specs to build the noise model for the IMU factor.
  
  // double accel_noise_sigma = 0.0003924;
  // double gyro_noise_sigma = 0.000205689024915;
  // double accel_bias_rw_sigma = 0.004905;
  // double gyro_bias_rw_sigma = 0.000001454441043;

  double accel_noise_sigma = 0.3924;
  double gyro_noise_sigma = 0.205689024915;
  double accel_bias_rw_sigma = 0.4905;
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

  // Keep track of the total error over the entire run for a simple performance metric.
  double current_position_error = 0.0, current_orientation_error = 0.0;

  double output_time = 0.0;
  double dt = 0.005;  // The real system has noise, but here, results are nearly 
                      // exactly the same, so keeping this for simplicity.

  // All priors have been set up, now iterate through the data file.
  ros::Rate rate(20);
  while (ros::ok()) { //ros::ok()
    ros::spinOnce();
    rate.sleep();

    if (imu_up == 1) { // IMU measurement
      cout<<"imu_up-> "<<imu_up<<endl;
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
     if (gps_up == 1) { // GPS measurement
      cout<<"gps_up-> "<<gps_up<<endl;
      Eigen::Matrix<double,7,1> gps = Eigen::Matrix<double,7,1>::Zero();
      
      gps(0) = odom_track.pose.pose.position.x;
      gps(1) = odom_track.pose.pose.position.y;
      gps(2) = odom_track.pose.pose.position.z;
      gps(3) = odom_track.pose.pose.orientation.x;
      gps(4) = odom_track.pose.pose.orientation.y;
      gps(5) = odom_track.pose.pose.orientation.z;
      gps(6) = 1;

      correction_count++;
      gps_up = 0;

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

      noiseModel::Diagonal::shared_ptr correction_noise = noiseModel::Isotropic::Sigma(3,0.01);
      GPSFactor gps_factor(X(correction_count),
                           Point3(gps(0),  // N,
                                  gps(1),  // E,
                                  gps(2)), // D,
                           correction_noise);
      graph->add(gps_factor);
      
      // Now optimize and compare results.
      prop_state = imu_preintegrated_->predict(prev_state, prev_bias);
      initial_values.insert(X(correction_count), prop_state.pose());
      initial_values.insert(V(correction_count), prop_state.v());
      initial_values.insert(B(correction_count), prev_bias);

      GaussNewtonOptimizer optimizer(*graph, initial_values);
      const clock_t begin_time = clock();
      Values result = optimizer.optimize();
      std::cout << "this optimizer used  time -> " << float(clock() - begin_time) / CLOCKS_PER_SEC << "\n\n";

      // Overwrite the beginning of the preintegration for the next step.
      prev_state = NavState(result.at<Pose3>(X(correction_count)),
                            result.at<Vector3>(V(correction_count)));
      prev_bias = result.at<imuBias::ConstantBias>(B(correction_count));

      // Reset the preintegration object.
      imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);

      // Print out the position and orientation error for comparison.
      Vector3 gtsam_position = prev_state.pose().translation();
      cout<<"current state ->" <<gtsam_position<<endl;
      nav_msgs::Odometry facgrapose = odom_track;
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
      cout<<"size of the graph ->"<<graph->size()<<endl;
      fprintf(fp_out, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", 
              output_time, gtsam_position(0), gtsam_position(1), gtsam_position(2),
              gtsam_quat.x(), gtsam_quat.y(), gtsam_quat.z(), gtsam_quat.w(),
              gps(0), gps(1), gps(2), 
              gps_quat.x(), gps_quat.y(), gps_quat.z(), gps_quat.w());

      output_time += 1.0; 

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
