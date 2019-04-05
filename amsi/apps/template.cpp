/* ----------------------------------------------------------------------------

 * Weisong Wen
 * HK, Califronia
 * All Rights Reserved
 * Authors: Weisong Wen, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file sensor integration for jd dataset
 * @brief Test example for using GTSAM ImuFactor and ImuCombinedFactor navigation code.
 * @author Garrett (ghemann@gmail.com), Luca Carlone
 */



// GTSAM related includes.
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <fstream>
#include <iostream>

//time 
#include <time.h>

// ros and math related head files
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
// head file for gps 
#include <sensor_msgs/NavSatFix.h>
#include <amsi/gnss_tools.hpp>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>

#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>

// Uncomment line below to use the CombinedIMUFactor as opposed to the standard ImuFactor.
// #define USE_COMBINED

using namespace gtsam;
using namespace std;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
typedef pcl::PointXYZI PointT;

class factorGraphFusion
{
  public:
    factorGraphFusion(ros::NodeHandle handle)
    {
      m_handle = handle;
      jdFix_sub =m_handle.subscribe("/fix", 32, &factorGraphFusion::jdFix_callback, this); // subscribe fix

      GNSS_odom_ENU_pub = m_handle.advertise<nav_msgs::Odometry>("/GNSS_odom_ENU", 5, false); // 
      pub_debug_marker_ = m_handle.advertise<visualization_msgs::MarkerArray>("/debug_marker", 5, false);

      referencellh.resize(3, 1);
      // start point of robot
      referencellh(0) = 116.4986266; //116.4986357 (pre-test) 116.4986266 (semi-final compet ) 116.4986473 (semi-final test )
      referencellh(1) = 39.7917427; // 39.7917481 ( pre-test) 39.7917427 (semi-final  compet ) 39.7917502 (semi-final  test )
      referencellh(2) = 22.1009979248; // 116.4986266 (semi-final )
      //-172.11 5.9 22 -2.51 (x ,y ,z, yaw)
      // rosbag record -O session2_090210.bag /ndt_pose /fix  /compensated_imu /odom /lidar_points

      GNSS_IMU_FusionTEST();

      //


    }
    ~factorGraphFusion(){}  

  private:    
    ros::NodeHandle m_handle;

    ros::Publisher GNSS_odom_ENU_pub;
    ros::Publisher pub_debug_marker_;


    ros::Subscriber jdFix_sub; // subscribe the /fix topic from jindong dataset  
    // gnss_tools
    GNSS_Tools gnss_tools_;

    /**********GNSS evaluation***********/
    struct result2DPose
    {
      double UTC_Time;
      double latitude;
      double longitude;
      double heading;
    };
    struct epochResult2DPose //  2D pose in one epoch
    {
      vector<result2DPose> epochData;
    };
    epochResult2DPose epochData_;
    vector<epochResult2DPose> results; // all the epochs
    double preTime=0;
    double firstEpoch =0;
    /**********GNSS evaluation***********/

    Eigen::MatrixXd referencellh; // origin llh
    epochResult2DPose ndtEpochData_;
    vector<epochResult2DPose> ndtResults; // all the epochs





  public:
    void run()
    {
      cout<<"run function ..."<<endl;
    }

    void jdFix_callback(const sensor_msgs::NavSatFixConstPtr& fix_msg)
    {
      // cout<<"jd /fix received "<<endl;
      result2DPose singleLLH;
      singleLLH.UTC_Time = fix_msg->header.stamp.toSec(); //((*seek)->stamp - gps_msg->header.stamp).toSec();
      // cout<< "singleLLH.UTC_Time  "<<singleLLH.UTC_Time<<endl;
      singleLLH.latitude = fix_msg->latitude;
      singleLLH.longitude = fix_msg->longitude;
      singleLLH.heading = fix_msg->position_covariance[0];

      sensor_msgs::NavSatFix navfix_ ;
      navfix_.header = fix_msg->header;
      navfix_.latitude = fix_msg->latitude;
      navfix_.longitude = fix_msg->longitude;
      navfix_.altitude = fix_msg->altitude;

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
      Eigen::MatrixXd vehicleENU; // 
      vehicleENU.resize(3, 1);
      eigenENU = gnss_tools_.ecef2enu(referencellh,ecef);
      std::cout << std::setprecision(17);
      cout<<"eigenENU ->"<<eigenENU<<endl;
      cout<<"curLLh ->"<<curLLh<<endl;

      /************* transform GNSS ENU to LiDAR coordinate system **************/
      double prex_ = eigenENU(0);
      double prey_ = eigenENU(1);
      double theta = (211.8 )*( 3.141592 / 180.0 ); //
      eigenENU(0) = prex_ * cos(theta) - prey_ * sin(theta) ;
      eigenENU(1) = prex_ * sin(theta) + prey_ * cos(theta) ; 

      // publish the GNSS odom in LiDAR coordiante system
      nav_msgs::Odometry GNSS_odom_enu;
      GNSS_odom_enu.header.stamp = fix_msg->header.stamp;
      GNSS_odom_enu.header.frame_id = "map";

      //GNSS_odom_enu.pose.pose.position.x = eigenENU(0);
      //GNSS_odom_enu.pose.pose.position.y = eigenENU(1);

      GNSS_odom_enu.pose.pose.position.x = 1 * eigenENU(1);
      GNSS_odom_enu.pose.pose.position.y = -1 * eigenENU(0);

      GNSS_odom_enu.pose.pose.position.z = 0;
      GNSS_odom_enu.child_frame_id = "base_link";
      GNSS_odom_enu.twist.twist.linear.x = 0.0;
      GNSS_odom_enu.twist.twist.linear.y = 0.0;
      GNSS_odom_enu.twist.twist.angular.z = 0.0;
      GNSS_odom_ENU_pub.publish(GNSS_odom_enu);
      /*************************************************/
    }

    

   /**
   * @brief TEST GNSS/INS integration 
   * @input GNSS INS saved in imuAndGPSdata.csv 
   * @output corresponding error

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
    void GNSS_IMU_FusionTEST() // 
    {
      string data_filename;
      printf("using default CSV file\n");
      data_filename = findExampleDataFile("imuAndGPSdata.csv");

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
      cout << "initial state:\n" << initial_state.transpose() << "\n\n";

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
      double accel_noise_sigma = 0.0003924;
      double gyro_noise_sigma = 0.000205689024915;
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

      // Keep track of the total error over the entire run for a simple performance metric.
      double current_position_error = 0.0, current_orientation_error = 0.0;

      double output_time = 0.0;
      double dt = 0.005;  // The real system has noise, but here, results are nearly 
                          // exactly the same, so keeping this for simplicity.

      // All priors have been set up, now iterate through the data file.
      while (file.good()) {

        

        // Parse out first value
        getline(file, value, ',');
        int type = atoi(value.c_str());

        if (type == 0) { // IMU measurement
          Eigen::Matrix<double,6,1> imu = Eigen::Matrix<double,6,1>::Zero();
          for (int i=0; i<5; ++i) {
            getline(file, value, ',');
            imu(i) = atof(value.c_str());
          }
          getline(file, value, '\n');
          imu(5) = atof(value.c_str());

          // Adding the IMU preintegration.
          imu_preintegrated_->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);

        } else if (type == 1) { // GPS measurement
          Eigen::Matrix<double,7,1> gps = Eigen::Matrix<double,7,1>::Zero();
          for (int i=0; i<6; ++i) {
            getline(file, value, ',');
            gps(i) = atof(value.c_str());
          }
          getline(file, value, '\n');
          gps(6) = atof(value.c_str());

          correction_count++;

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

          noiseModel::Diagonal::shared_ptr correction_noise = noiseModel::Isotropic::Sigma(3,1.0);
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

          LevenbergMarquardtOptimizer optimizer(*graph, initial_values);
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
          cout << "Position error:" << current_position_error << "\t " << "Angular error:" << current_orientation_error << "\n";

          fprintf(fp_out, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", 
                  output_time, gtsam_position(0), gtsam_position(1), gtsam_position(2),
                  gtsam_quat.x(), gtsam_quat.y(), gtsam_quat.z(), gtsam_quat.w(),
                  gps(0), gps(1), gps(2), 
                  gps_quat.x(), gps_quat.y(), gps_quat.z(), gps_quat.w());

          output_time += 1.0; 

        } else {
          cerr << "ERROR parsing file\n";
        }
      }
      fclose(fp_out);
      cout << "Complete, results written to " << output_filename << "\n\n";;
    }

    void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg);

  private: 
    const string output_filename = "imuFactorExampleResults.csv";

    // This will either be PreintegratedImuMeasurements (for ImuFactor) or
    // PreintegratedCombinedMeasurements (for CombinedImuFactor).
    PreintegrationType *imu_preintegrated_;

  


};


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "iSAMFusion");
  std::cout<<"iSAMFusion node......(factorGraphFusion)"<<std::endl;

  ros::NodeHandle nh;
  factorGraphFusion* factorGraphFusion_ = new factorGraphFusion(nh);// dynamic application memory
  factorGraphFusion_->run();
  ros::spin();



  
  return 0;
}
