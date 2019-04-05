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

// #include <rtklib/rtklib.h>
#include <Eigen/Dense>

#include "shadowmatching/utilities.h"
#include "shadowmatching/WeightLeastSquare.h"
#include "shadowmatching/SelectEphemeris.h"
#include "shadowmatching/shadowMatching.h"
#include "stdio.h"
//#define TARGETWEEK 1997
//#define STARTTIME 218012
//#define ENDTIME   218014

std::string main_directory = "result\\";
std::string data_name = "v2v107M";

extern void test()
{
  std::cout<<"read nav...."<<std::endl;
}

//read the rinex obx and nav(ephemeris) using rtklib function
extern int readobsnav(gtime_t ts, gtime_t te, double ti, char **infile,
  const int *index, int n, const prcopt_t *prcopt,
  obs_t *obs, nav_t *nav, sta_t *sta)
{
  int nepoch = 0;            /* number of observation epochs */
  int i, j, ind = 0, nobs = 0, rcv = 1;
  printf("%d %d %d %d\n", index[0], index[1], index[2], index[3]); // to read two obs file
  obs->data = NULL; obs->n = obs->nmax = 0;
  nav->eph = NULL; nav->n = nav->nmax = 0;
  nav->geph = NULL; nav->ng = nav->ngmax = 0;
  nav->seph = NULL; nav->ns = nav->nsmax = 0;

  for (i = 0; i < n; i++) {

    if (index[i] != ind) {
      if (obs->n > nobs) rcv++;
      ind = index[i]; nobs = obs->n;
    }
    //read rinex obs and nav file 
    if (readrnxt(infile[i], rcv, ts, te, ti, prcopt->rnxopt[rcv <= 1 ? 0 : 1], obs, nav,
      rcv <= 2 ? sta + rcv - 1 : NULL) < 0) {
      return 0;
    }
  }

  if (obs->n <= 0) {
    return 0;
  }

  nepoch = sortobs(obs);

  /*
  if (nav->n<=0&&nav->ng<=0&&nav->ns<=0) {
  return 0;
  }
  */
  // delete duplicated ephemeris
  //uniqnav(nav);

  // set time span for progress display 
  if (ts.time == 0 || te.time == 0)
  {
    for (i = 0; i < obs->n; i++)
      if (obs->data[i].rcv == 1)
        break;
    for (j = obs->n - 1; j >= 0; j--)
      if (obs->data[j].rcv == 1)
        break;
    if (i < j) {
      if (ts.time == 0) ts = obs->data[i].time;
      if (te.time == 0) te = obs->data[j].time;
      settspan(ts, te);
    }
  }

  return 1;
}


//read the precise ephemeris (not used in 3D-GNSS code)
static void readpreceph(char **infile, int n, const prcopt_t *prcopt,
  nav_t *nav, sbs_t *sbs, lex_t *lex)
{
  seph_t seph0 = { 0 };
  int i;
  //    char *ext;

  trace(3, "readpreceph: n=%d\n", n);

  nav->ne = nav->nemax = 0;
  nav->nc = nav->ncmax = 0;
  sbs->n = sbs->nmax = 0;
  lex->n = lex->nmax = 0;

  /* read precise ephemeris files */
  for (i = 0; i < n; i++) {
    if (strstr(infile[i], "%r") || strstr(infile[i], "%b")) continue;
    readsp3(infile[i], nav, 0);
  }
  /* read precise clock files */
  for (i = 0; i < n; i++) {
    if (strstr(infile[i], "%r") || strstr(infile[i], "%b")) continue;
    readrnxc(infile[i], nav);
  }
  /* read sbas message files */
  for (i = 0; i < n; i++) {
    if (strstr(infile[i], "%r") || strstr(infile[i], "%b")) continue;
    sbsreadmsg(infile[i], prcopt->sbassatsel, sbs);
  }
  /* read lex message files */
  for (i = 0; i < n; i++) {
    if (strstr(infile[i], "%r") || strstr(infile[i], "%b")) continue;
    lexreadmsg(infile[i], 0, lex);
  }
  /* allocate sbas ephemeris */
  nav->ns = nav->nsmax = NSATSBS * 2;
  if (!(nav->seph = (seph_t *)malloc(sizeof(seph_t)*nav->ns))) {
    showmsg("error : sbas ephem memory allocation");
    trace(1, "error : sbas ephem memory allocation");
    return;
  }
  for (i = 0; i < nav->ns; i++) nav->seph[i] = seph0;

}

/* dummy application functions for shared library ----------------------------*/
extern int showmsg(char *format,...);

extern void settspan(gtime_t ts, gtime_t te) {}
extern void settime(gtime_t time) {}


namespace amsi {

class shadowmatchingNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  shadowmatchingNodelet() {
  }
  virtual ~shadowmatchingNodelet() {
  }


  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();
    initialize_params();
  }

private:
  void initialize_params() {
  std::cout<<"initialize_params...."<<std::endl;
  #ifdef STARTTIME
    gtime_t ts = gpst2time(TARGETWEEK, STARTTIME);
  #else 
    gtime_t ts = { 0 }; ts.time = 0;              // start time (0:all)
  #endif

  #ifdef  ENDTIME
    gtime_t te = gpst2time(TARGETWEEK, ENDTIME);
  #else
    gtime_t te = { 0 }; te.time = 0;              // end time (0:all) 
  #endif

    /* Configuration of RTKLIB ------------------------*/
  int n = 0, i, stat;

  double ti = 0.0;            // processing interval  (s) (0:all)
  double tu = 0.0;            // unit time (s) (0:all)  

  /* options */
  prcopt_t prcopt = prcopt_default; // processing option
  solopt_t solopt = solopt_default; // output solution option
  filopt_t filopt = { "" };             // file option

  prcopt.mode = 2;        // 0: SPP 1: DGNSS 2:Kinematic RTK
  prcopt.navsys = SYS_ALL;
  prcopt.nf = 1;            // frequency (1:L1,2:L1+L2,3:L1+L2+L5) 
  prcopt.soltype = 0;         // 0:forward,1:backward,2:combined
  prcopt.elmin = 10.0*D2R;        // elevation mask (rad) 
  prcopt.tidecorr = 0;          // earth tide correction (0:off,1-:on) 
  prcopt.posopt[4] = 0;               // use RAIM FDE (qmo)
  prcopt.tropopt = TROPOPT_SAAS;        // troposphere option: Saastamoinen model
  prcopt.ionoopt = IONOOPT_BRDC;    // ionosphere option: Broad cast
  prcopt.sateph = EPHOPT_BRDC;      // ephemeris option: broadcast ephemeris

  prcopt.modear = 1;          // AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold)

  // HKSC
  prcopt.rb[0] = -2414266.9197;           // base position for relative mode {x,y,z} (ecef) (m)
  prcopt.rb[1] = 5386768.9868;      // base position for relative mode {x,y,z} (ecef) (m)
  prcopt.rb[2] = 2407460.0314;      // base position for relative mode {x,y,z} (ecef) (m)

  solopt.outopt = 1;          // output processing options (0:no,1:yes)
  solopt.timef = 0;           // time format (0:sssss.s,1:yyyy/mm/dd hh:mm:ss.s)
  solopt.timeu = 3;           // time digits under decimal point
  solopt.sep[0] = ',';          // field separator
  solopt.sstat = 0;           // solution statistics level (0:off,1:states,2:residuals)
  solopt.trace = 0;           // debug trace level (0:off,1-5:debug)
  solopt.sstat = 0;           // get the solution file
  solopt.posf = SOLF_XYZ;
  solopt.height = 0;

  char *rov = "", *base = "";
  char infile_[5][1024] = { "" }, *infile[5];
  char outfile[1024];
  /* set input files */
  for (i = 0; i < 5; i++) infile[i] = infile_[i];
  strcpy(infile[n++], "/home/wenws/amsipolyu/src/amsi/data/v2v107M2.obs");
  strcpy(infile[n++], "/home/wenws/amsipolyu/src/amsi/data/hksc107m.18n");
  strcpy(infile[n++], "/home/wenws/amsipolyu/src/amsi/data/hksc107m.18g");
  /**************************************************/
  // strcpy(infile[n++], "data\\TSTb120L.obs");
  // strcpy(infile[n++], "data\\hksc1200.18n");
  // strcpy(infile[n++], "data\\hksc1200.18g");

  /**************************************************/

  /*starting to read the obs and nav rinex data*/
  int index[4] = { 0 };

  obs_t obss = { 0 };          /* observation data */
  nav_t navs = { 0 };          /* navigation data */
  sta_t stas[MAXRCV];      /* station infomation */
  lex_t lexs = { 0 };          /* lex messages */

  if (prcopt.mode == PMODE_KINEMA || prcopt.mode == PMODE_DGPS) {
    index[1] = 1;
    index[2] = 2;
    index[3] = 0;
  }
  // Step1: rinex obs and nav to variable obss and navs! @@@
  // extern int readobsnav(gtime_t ts, gtime_t te, double ti, char **infile,const int *index, int n, const prcopt_t *prcopt,
  // obs_t *obs, nav_t *nav, sta_t *sta)

  std::cout<<"before read nav file...."<<std::endl;
  bool read_result = 0;
  settspan(ts,ts);
  read_result = readobsnav(ts, te, ti, infile, index, n, &prcopt, &obss, &navs, stas);
  if (!readobsnav(ts, te, ti, infile, index, n, &prcopt, &obss, &navs, stas)) 
    {
        std::cout<<"some error here...."<<std::endl;
    }   

    settime(te);



  //shadowMatching SM_;
  }
    



private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;
};

}


PLUGINLIB_EXPORT_CLASS(amsi::shadowmatchingNodelet, nodelet::Nodelet)
