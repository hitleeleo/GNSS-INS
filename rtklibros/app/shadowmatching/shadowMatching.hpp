#pragma once

// customized head file
#include "rtklib.h"
//#include "WeightLeastSquare.h"
//#include "SelectEphemeris.h"

// eigen 
#include "Eigen/Eigen"

// vector, io stream
#include <vector>
#include <iostream>

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
#include <omp.h>
// Define Infinite (Using INT_MAX caused overflow problems)
#define INF 10000
using namespace Eigen;
using namespace std;

//#define TARGETWEEK 1997
//#define STARTTIME 218012
//#define ENDTIME   218014


class shadowMatching
{
public:
	shadowMatching()
{
	std::cout<<"initializae the shadowMatching objects....."<<std::endl;
	map_.disReso =2.0;
	// reference LLH (mannually  set)
	posLLHR_.lon = 114.1772621294604;
	posLLHR_.lat  = 22.29842880200087;
	posLLHR_.alt  = 0;
	scoreThres = 0.7;

	ENU ENUTemp;
	ENUTemp.E = 0;
	ENUTemp.N = -16;
	ENUTemp.U = 0;
	skyMask skymask_;
	skymask_ = preparingSkymaskbyPose(ENUTemp);

	ENUTemp.E = 52; // not available
	ENUTemp.N = 188;
	ENUTemp.U = 0;
	skymask_ = preparingSkymaskbyPose(ENUTemp);

	//clock_t begin_time = clock();
	////readSkymask("skymaskTest.txt"); // read the skymasks in txt file
	//bool fileAvailaibility_ = readSkymask("skymask/162 417.txt"); // read the skymasks in txt file 259s for read whole skymask.txt file
	//std::cout << "       read sky mask used  time -> " << float(clock() - begin_time) / CLOCKS_PER_SEC << "\n\n";

	////begin_time = clock();
	//string2llh(skysmaskPosString, skysmaskString);
	//std::cout << "       parse sky mas	k used  time -> " << float(clock() - begin_time) / CLOCKS_PER_SEC << "\n\n";
}
	~shadowMatching()
{
}


public: // variables for particles
	double reserve;
	typedef struct obs_Nav_epoch // reserved
	{

	};

	typedef struct  // satellite information
	{
		double GNSS_time;
		double total_sv; // total satellite in this epoch
		double prn_satellites_index; // satellite prn 
		double pseudorange; // satellite pseudorange
		double snr; // satellite snr
		double elevation; // satellite elevation 
		double azimuth; // satellite azimuth 
		double err_tropo; //satellite erro_tropo
		double err_iono; // satellite ono
		double sat_clk_err; // satellite clock bias 
		double sat_pos_x; // satellite position x
		double sat_pos_y; // satellite position y
		double sat_pos_z; // satellite position z
		int visable; // satellite visability
		std::string sat_system; // satellite system ("GPS", "GLONASS")
	}satelliteInfo;

	typedef struct // single grid: determined in ENU coordiante system
	{
		double E_;
		double N_;
		double U_;
		double score;
		//
	}grid;

	typedef struct  //  grid sequences (grids)
	{
		std::vector<grid> grids_;
	}grids;

	typedef struct  // state for a particle
	{
		double GNSS_time = 0; // time stamps
							  //satellites information 
		std::vector<satelliteInfo>  satInfo; // incluide pseudorange.etc
		int particle_ID; // the ID for particle
		double scores; // scores for this particle 
					   //position in llh
		
		double lon; // latitude 
		double lat; // longtutude 
		double altitude; // altitude 


		grids smGrids; 	//grid at present for shadow matching (plane)
		std::vector<grids> smGridss; // all the grids from first epoch to last epoch (multi-plane)

		//position in ENU
		double E;
		double N;
		double U;
		double ini_lon;
		double ini_lat;
		double ini_alt;

		// related measurements
		MatrixXd eAllMeasurement;
		MatrixXd eELAZAngles; // all the possible azimuth 
		vector<int> visibility;

	}particle;
	std::vector<particle> particles; // all the particles at present
	std::vector< std::vector<particle> > particless; // all the particles from first epoch to last epoch

public:
	int reserved = 0; // not used yet

public:  // for generate map
	typedef struct
	{
		double lon;
		double lat;
		double alt;
	}LLH;
	typedef double coord_t;         // coordinate type
	typedef double coord2_t;  // must be big enough to hold 2*max(|coordinate|)^2
	struct Point {
		coord_t x, y;

		bool operator <(const Point &p) const {
			return x < p.x || (x == p.x && y < p.y);
		}
	};
	typedef struct
	{
		double E;
		double N;
		double U;
	}ENU;
	typedef struct // struct for a processed building information
	{
		double center_E; // center position of the building in E direction
		double center_N; // center position of the building in N direction
		double center_U; // center position of the building in U direction
		double ini_lon; // initial reference position for ENU
		double ini_lat; // initial reference position for ENU
		double ini_alt; // initial reference position for ENU
		vector <LLH> buildingLLHV; // building node list (each node with respect to a vertex) in llh
		vector <ENU> buildingENUV; // building node list (each node with respect to a vertex) in ENU
		vector<Point> buildingHull; // building hull points (used for later check if the point is inside the building) in ENU
		double sAzimuth; // smallest azimuth
		double bAzimuth; // bigest azimuth


	} building;
	vector<building> buildingS; // processed building information
	typedef struct  // struct a map
	{
		vector<building> buildingS_M; // processed building information save in a map struct
									  // lon lat alt boundary 
		double lonMax; // max longitude
		double latMax; // max latitude
		double altMax; // max altitude
		MatrixXd llhMax = MatrixXd::Random(3, 1); // available 

		double lonMin; // min longitude
		double latMin; // min latitude
		double altMin; // min altitude
		MatrixXd llhMin = MatrixXd::Random(3, 1); // available 

												  // ENU boundary
		double EMax; // max E
		double NMax; // max N
		double UMax; // max U
		MatrixXd ENUMax = MatrixXd::Random(3, 1); // available 

		double EMin; //  min E
		double NMin; //  min N
		double UMin; //  min U
		MatrixXd ENUMin = MatrixXd::Random(3, 1); // available 

												  // original llh
		double lonOri; // original lon of ENU (refers to reference point)
		double latOri; // original lat of ENU (refers to reference point)
		double altOri; // original alt of ENU (refers to reference point)
		MatrixXd llhOri = MatrixXd::Random(3, 1);

		// minimum elevation angle and corresponding maxinum searching distance
		double MinElevation = 15; // satellites with a elevation less than 15 degree will not be considered
		double maxSearDis; // the maximum searching distance when finding the insections
						   // azimuth search resolution 
		double aziReso = 1.0; // azimuth search resolution 
		double disReso = 2.0; // grid resolution for skymasks
	}map;
	map map_; // the map saving all the buildings

public: // for obtain sky mask
	vector<string> skysmaskPosString;
	vector<string> skysmaskString;

public:  // for obtain sky mask
	typedef struct // mask elevation
	{
		double azimuth; // the azimuth 
		double elevation; // the mask elevation
	}elevationMask;

	typedef struct
	{
		LLH posLLH; // pose in LLH
		LLH posLLHR; // pose in LLH for reference
		ENU poseENU; // pose in ENU
		double buildingNum; // how many this position can intersect: if point inside building (buildingNum =1)
		bool insideBuilding; // inside buildings (=1) , outside building (=0)
		float aziResol; // azimuth resolution 
		vector<elevationMask> aziElemask; // a vector 
	}skyMask; // the sky mask in a position (refers to the skyplot in one position)
	vector<skyMask> skyMaskS; // save all the skymsk in one vector 
	LLH posLLHR_; // pose in LLH for reference
	double scoreThres;

public:  // time delay
	void wait(int seconds)
	{
		clock_t endwait;
		endwait = clock() + seconds * CLOCKS_PER_SEC;
		while (clock() < endwait) {}
	}

public: // functions 
	
	/*
	author: WEN Weisong (17902061r@connect.polyu.hk)
	function: llh to ecef
	input: llh (Matrix3d)
	output: ecef (Matrix3d)
	*/
	Eigen::MatrixXd llh2ecef(Eigen::MatrixXd data) // transform the llh to ecef
	{
		Eigen::MatrixXd ecef; // the ecef for output
		ecef.resize(3, 1);
		double a = 6378137.0;
		double b = 6356752.314;
		double n, Rx, Ry, Rz;
		double lon = (double)data(0) * 3.1415926 / 180.0; // lon to radis
		double lat = (double)data(1) * 3.1415926 / 180.0; // lat to radis
		double alt = (double)data(2); // altitude
		n = a * a / sqrt(a * a * cos(lat) * cos(lat) + b * b * sin(lat) * sin(lat));
		Rx = (n + alt) * cos(lat) * cos(lon);
		Ry = (n + alt) * cos(lat) * sin(lon);
		Rz = (b * b / (a * a) * n + alt) * sin(lat);
		ecef(0) = Rx; // return value in ecef
		ecef(1) = Ry; // return value in ecef
		ecef(2) = Rz; // return value in ecef
		return ecef;

		/**************for test purpose*************************
		Eigen::MatrixXd llh;
		llh.resize(3, 1);
		Eigen::MatrixXd ecef;
		ecef.resize(3, 1);
		llh(0) = 114.1772621294604;
		llh(1) = 22.29842880200087;
		llh(2) = 58;
		ecef = llh2ecef(llh);
		cout << "ecef ->: " << ecef << "\n";
		*/
	}

	/*
	author: WEN Weisong (17902061r@connect.polyu.hk)
	function: ecef to llh
	input: ecef (Matrix3d)
	output: llh (Matrix3d)
	*/
	Eigen::MatrixXd ecef2llh(Eigen::MatrixXd data) // transform the ecef to llh
	{
		Eigen::MatrixXd llh; // the ecef for output
		double pi = 3.1415926; // pi
		llh.resize(3, 1);
		double x = data(0); // obtain ecef 
		double y = data(1);
		double z = data(2);
		double x2 = pow(x, 2);
		double y2 = pow(y, 2);
		double z2 = pow(z, 2);

		double a = 6378137.0000; //earth radius in meters
		double b = 6356752.3142; // earth semiminor in meters
		double e = sqrt(1 - (b / a) * (b / a));
		double b2 = b*b;
		double e2 = e*e;
		double  ep = e*(a / b);
		double  r = sqrt(x2 + y2);
		double  r2 = r*r;
		double  E2 = a * a - b*b;
		double F = 54 * b2*z2;
		double G = r2 + (1 - e2)*z2 - e2*E2;
		double c = (e2*e2*F*r2) / (G*G*G);
		double s = (1 + c + sqrt(c*c + 2 * c));
		s = pow(s, 1 / 3);
		double P = F / (3 * ((s + 1 / s + 1)*(s + 1 / s + 1)) * G*G);
		double Q = sqrt(1 + 2 * e2*e2*P);
		double ro = -(P*e2*r) / (1 + Q) + sqrt((a*a / 2)*(1 + 1 / Q) - (P*(1 - e2)*z2) / (Q*(1 + Q)) - P*r2 / 2);
		double tmp = (r - e2*ro)*(r - e2*ro);
		double U = sqrt(tmp + z2);
		double V = sqrt(tmp + (1 - e2)*z2);
		double zo = (b2*z) / (a*V);

		double height = U*(1 - b2 / (a*V));

		double lat = atan((z + ep*ep*zo) / r);

		double temp = atan(y / x);
		double long_;
		if (x >= 0)
			long_ = temp;
		else if ((x < 0) && (y >= 0))
			long_ = pi + temp;
		else
			long_ = temp - pi;
		llh(0) = (long_)*(180 / pi);
		llh(1) = (lat)*(180 / pi);
		llh(2) = height;
		return llh;

		/**************for test purpose*************************
		Eigen::MatrixXd ecef;
		ecef.resize(3, 1);
		Eigen::MatrixXd llh;
		llh.resize(3, 1);
		ecef(0) = -2418080.9387265667;
		ecef(1) = 5386190.3905763263;
		ecef(2) = 2405041.9305451373;
		llh = ecef2llh(ecef);
		cout << "llh ->: " << llh << "\n";
		*/
	}

	/*
	author: WEN Weisong (17902061r@connect.polyu.hk)
	function: ecef to enu
	input: original llh, and current ecef (Matrix3d)
	output: enu (Matrix3d)
	*/
	Eigen::MatrixXd ecef2enu(Eigen::MatrixXd originllh, Eigen::MatrixXd ecef) // transform the ecef to enu 
	{
		double pi = 3.1415926; // pi 
		double DEG2RAD = pi / 180.0;
		double RAD2DEG = 180.0 / pi;

		Eigen::MatrixXd enu; // the enu for output
		enu.resize(3, 1); // resize to 3X1
		Eigen::MatrixXd oxyz; // the original position 
		oxyz.resize(3, 1); // resize to 3X1

		double x, y, z; // save the x y z in ecef
		x = ecef(0);
		y = ecef(1);
		z = ecef(2);

		double ox, oy, oz; // save original reference position in ecef
		oxyz = llh2ecef(originllh);
		ox = oxyz(0); // obtain x in ecef 
		oy = oxyz(1); // obtain y in ecef
		oz = oxyz(2); // obtain z in ecef

		double dx, dy, dz;
		dx = x - ox;
		dy = y - oy;
		dz = z - oz;

		double lonDeg, latDeg, _; // save the origin lon alt in llh
		lonDeg = originllh(0);
		latDeg = originllh(1);
		double lon = lonDeg * DEG2RAD;
		double lat = latDeg * DEG2RAD;

		//save ENU
		enu(0) = -sin(lon) * dx + cos(lon) * dy;
		enu(1) = -sin(lat) * cos(lon) * dx - sin(lat) * sin(lon) * dy + cos(lat) * dz;
		enu(2) = cos(lat) * cos(lon) * dx + cos(lat) * sin(lon) * dy + sin(lat) * dz;
		return enu;

		/**************for test purpose*****suqare distance is about 37.4 meters********************
		Eigen::MatrixXd llh;  //original
		llh.resize(3, 1);
		llh(0) = 114.1775072541416;
		llh(1) = 22.29817969722738;
		llh(2) = 58;
		Eigen::MatrixXd ecef;
		ecef.resize(3, 1);
		ecef(0) = -2418080.9387265667;
		ecef(1) = 5386190.3905763263;
		ecef(2) = 2405041.9305451373;
		Eigen::MatrixXd enu;
		enu.resize(3, 1);
		enu = ecef2enu(llh, ecef);
		cout << "enu ->: " << enu << "\n";
		*/
	}

	/*
	author: WEN Weisong (17902061r@connect.polyu.hk)
	function: ecef to enu
	input: original llh, and current ecef (Matrix3d)
	output: enu (Matrix3d)
	*/
	Eigen::MatrixXd enu2ecef(Eigen::MatrixXd originllh, Eigen::MatrixXd enu) // transform the ecef to enu 
	{
		// enu to ecef
		double  e = enu(0);
		double  n = enu(1);
		double  u = enu(2);
		double lon = (double)originllh(0) * D2R;
		double lat = (double)originllh(1) * D2R;
		Eigen::MatrixXd oxyz; // the original position 
		oxyz.resize(3, 1); // resize to 3X1
		oxyz = llh2ecef(originllh);
		double ox = oxyz(0);
		double oy = oxyz(1);
		double oz = oxyz(2);

		oxyz(0) = ox - sin(lon) * e - cos(lon) * sin(lat) * n + cos(lon) * cos(lat) * u;
		oxyz(1) = oy + cos(lon) * e - sin(lon) * sin(lat) * n + cos(lat) * sin(lon) * u;
		oxyz(2) = oz + cos(lat) * n + sin(lat) * u;
		return oxyz;
	}

	/*
	author: WEN Weisong (17902061r@connect.polyu.hk)
	function: readSkymask from CSV file 
	input: CSV file containing skymasks 
	output: file availibility 
	*/
	bool readSkymask(string filepath) //  readSkymask from CSV file 
	{
		//  readSkymask from CSV file 
		bool fileAvail = 0;
		ifstream in(filepath);
		string filename;
		string line; // save content of one line 
		int lineNum = 0; // line number
		string search = "skymask"; // searching 
		vector<int> cLine; // save line numbers
		vector<string> allLines; // save all the lines of the kml file
		bool fileempty = (in.peek() == std::ifstream::traits_type::eof());
		// obtain the first line
		if (in && (!fileempty) ) // file is available   
		{
			getline(in, line);
			//cout << "line ->: " << line << "\n";
			std::stringstream ss(line); // split into three string
			vector<string> result;
			while (ss.good())
			{
				string substr;
				getline(ss, substr, ',');
				result.push_back(substr);
			}
			LLH llh_;
			for (int k = 0; k < result.size(); k++)
			{
				//std::cout<<"string result: " << result.at(k) << std::endl;
				string word;
				word = result.at(k);
				double value = strtod(word.c_str(), NULL);
				std::cout << std::setprecision(17);
				//std::cout<< "double result: " << value << '\n';

				if (k == 0) {
					llh_.lon = value;
				}
				else if (k == 1) llh_.lat = value;
				else if (k == 2) llh_.alt = value;
			}
			//posLLHR_ = llh_; // currently not accurate in the skymask file 
			//cout << "llh_ " << llh_.lat << "\n";
		}
		else // file is not available
		{
			/*cout << "no such file" << endl;*/
			return fileAvail;
		}
		
		// Read the following lines 
		if (in && (!fileempty)) // file is available   
		{
			while (getline(in, line)) // line not contain line break (enter)
			{
				lineNum++;
				//cout << line << endl;
				//std::cout << "line number" << lineNum << endl;
				allLines.push_back(line);
				if (line.find(search, 0) != string::npos) {
					//cout << "found: " << search << "line number : " << lineNum << endl;
					cLine.push_back(lineNum - 1); //  save line number
				}
			}
			//cout << "skymask amounts:" << cLine.size() / 2 << endl; // amount of skymask
			for (auto it = cLine.begin(); it != cLine.end(); it++) {
				int temp = *it;
				//cout << "one building data: " << allLines[temp+1] << endl;
				skysmaskPosString.push_back(allLines[temp + 1]); // position in ENU, inside building, azimuth resolution
				skysmaskString.push_back(allLines[temp + 2]);
				it++;
			}
			/*for (auto it = skysmaskString.begin(); it != skysmaskString.end(); it++) {
				cout << "-----------------------------------skymask--------------------------------" << "\n" << *it << "\n" << endl;
			}
			for (auto it = skysmaskPosString.begin(); it != skysmaskPosString.end(); it++) {
				cout << "-----------------------------------skysmaskPosString--------------------------------" << "\n" << *it << "\n" << endl;
			}*/

		}
		else // file is not available
		{
		/*	cout << "no such file" << endl;*/
			return fileAvail;
		}
		fileAvail = 1;
		return fileAvail;
	}

	/*
	author: WEN Weisong (17902061r@connect.polyu.hk)
	function: parse llh from string
	input: vector<string>, vector<string>
	output: 
	// the first [] indicates the building index, the second [] indicates the lla index in one building
	*/
	void string2llh(vector<string> poseReInsi,  vector<string> skysmaskString_)// extract the llh from the string for each building
	{
		// Read elevation information second line in one skymask
		vector<string> oneSkymask;
		#pragma omp parallel for
		// for (auto it = skysmaskString_.begin(); it != skysmaskString_.end(); it++) { // for one skymask
		for (int it = 0; it < skysmaskString_.size(); it++) { // for one skymask
			string temp = skysmaskString_[it];
			string result;
			stringstream input(temp); // split by blank
			while (input >> result)
				oneSkymask.push_back(result);
			skyMask skyMask_;
			#pragma omp parallel for
			for (int i = 0; i<oneSkymask.size(); i++) { // for one skymask again 
				//split the string 114.1772621294604,22.29842880200087,58 and save it to  :vector <LLH> buildingLLH; 
				elevationMask elevationMask_;

				//std::stringstream ss(oneSkymask[i]); // split into three string
				//vector<string> result;
				//while (ss.good())
				//{
				//	string substr;
				//	getline(ss, substr, ',');
				//	result.push_back(substr);
				//}
				//for (int k = 0; k < result.size(); k++)
				//{
				//	string word;
				//	word = result.at(k);
				//	double value = strtod(word.c_str(), NULL);
				//	std::cout << std::setprecision(17);
				//	elevationMask_.azimuth = i * (360.0/(oneSkymask.size()-1) );
				//	elevationMask_.elevation = value;
				//	/*cout << "each azimuth:  " << elevationMask_.azimuth  << endl;
				//	cout << "each elevation:  " << elevationMask_.elevation << endl;*/
				//}

				string word = oneSkymask[i];
				double value = strtod(word.c_str(), NULL);
				std::cout << std::setprecision(17);
				elevationMask_.azimuth = i * (360.0 / (oneSkymask.size() - 1));
				elevationMask_.elevation = value;
				//cout << " elevationMask_.elevation :   " << elevationMask_.elevation <<"\n";


				skyMask_.aziElemask.push_back(elevationMask_);
			}
			oneSkymask.clear(); // clear this after process a building (very important)
			/*cout << "size of elevation saved in one skymask: " << skyMask_.aziElemask.size() << "\n" << endl;
			cout << "finish one skymask......................................................................................................................." << endl;*/
			skyMaskS.push_back(skyMask_); // each building save in this vector
			skyMask_.aziElemask.clear(); // clear this after fully finished one building (very important)
		}
		//cout << "finish all skymasks, total skymask number is : " << skyMaskS.size() <<"            position number: "<< skysmaskPosString.size()<< "\n" << endl; // 
		std::cout << std::setprecision(17);


		// Read position information: first line in one skymask 
		vector<string> oneInfo;
		int index = 0; // index for skymasks 
		for (auto it = poseReInsi.begin(); it != poseReInsi.end(); it++) { // for one position (one position refers to one skymask and one info)
			string temp = *it;
			string result;
			stringstream input(temp); // split by blank
			while (input >> result)
				oneInfo.push_back(result);
			for (int i = 0; i<oneInfo.size(); i++) { // for one position again 
				//-101.816,-118.224,0,1
				//LLH posLLH_; // pose in LLH
				//LLH posLLHR_; // pose in LLH for reference
				ENU poseENU_; // pose in ENU
				bool insideBuilding_=0; // inside buildings (=1) , outside building (=0)
				std::stringstream ss(oneInfo[i]); // split into several string
				vector<string> result;
				while (ss.good())
				{
					string substr;
					getline(ss, substr, ',');
					result.push_back(substr);
				}
				for (int k = 0; k < result.size(); k++)
				{
					//std::cout<<"string result: " << result.at(k) << std::endl;
					string word;
					word = result.at(k);
					double value = strtod(word.c_str(), NULL);
					std::cout << std::setprecision(17);

					if (k == 0) {
						poseENU_.E = value;
					}
					else if (k == 1)
					{
						poseENU_.N = value;
						poseENU_.U = 0;
					}
					else if (k == 2) insideBuilding_ = value;
					//cout << "size of this skymask :  " << oneSkymask.size() << endl;
				}
				skyMaskS[index].poseENU = poseENU_;
				skyMaskS[index].insideBuilding = insideBuilding_;
			}
			oneInfo.clear(); // clear this after process a skymask (very important)
		}
		std::cout << std::setprecision(17);

		//cout << " skyMaskS[index].poseENU" << skyMaskS[0].poseENU.E << "                "<<skyMaskS[0].poseENU.N << "                " << skyMaskS[0].insideBuilding<<"\n";
		//cout << "-----------------------------------------------skymasks have been read ----------------------------------------\n\n" << endl; // 
	}

	/*
	author: WEN Weisong (17902061r@connect.polyu.hk)
	function: prepare the skymask based on a certain position
	input: ENU coordinate 
	output: the skymask
	*/
	skyMask preparingSkymaskbyPose(ENU ENU_)// extract the llh from the string for each building
	{
		skyMask skymask_; // save the final result: the skymask

		// obtain the file name based on the postion 
		string filename;
		std::ostringstream filenameStrs; //
		filenameStrs << "skymask/";
		filenameStrs << int(ENU_.E);
		filenameStrs << " ";
		filenameStrs << int(ENU_.N);
		filenameStrs << ".txt";
		filename = filenameStrs.str();

		string filename1; // right 
		std::ostringstream filenameStrs1; //
		filenameStrs1 << "skymask/";
		filenameStrs1 << int(ENU_.E + map_.disReso);
		filenameStrs1 << " ";
		filenameStrs1 << int(ENU_.N);
		filenameStrs1 << ".txt";
		filename1 = filenameStrs1.str();

		string filename2; // left 
		std::ostringstream filenameStrs2; //
		filenameStrs2 << "skymask/";
		filenameStrs2 << int(ENU_.E - map_.disReso/2);
		filenameStrs2 << " ";
		filenameStrs2 << int(ENU_.N);
		filenameStrs2 << ".txt";
		filename2 = filenameStrs2.str();

		string filename3; // down 
		std::ostringstream filenameStrs3; //
		filenameStrs3 << "skymask/";
		filenameStrs3 << int(ENU_.E);
		filenameStrs3 << " ";
		filenameStrs3 << int(ENU_.N - map_.disReso/2);
		filenameStrs3 << ".txt";
		filename3 = filenameStrs3.str();

		string filename4; // up 
		std::ostringstream filenameStrs4; //
		filenameStrs4 << "skymask/";
		filenameStrs4 << int(ENU_.E);
		filenameStrs4 << " ";
		filenameStrs4 << int(ENU_.N + map_.disReso/2);
		filenameStrs4 << ".txt";
		filename4 = filenameStrs4.str();
		// 
		clock_t begin_time = clock();

		// clear the string vector saving the poses and skymasks
		skysmaskPosString.clear();
		skysmaskString.clear();
		bool fileAvailaibility_ = readSkymask(filename); // read expected position file 
		if (!fileAvailaibility_)
		{
			fileAvailaibility_ = readSkymask(filename1);
			/*if(fileAvailaibility_)
				cout << "find file in right neighbour :  "<< fileAvailaibility_<<"\n";*/
		}
		if (!fileAvailaibility_)
		{
			fileAvailaibility_ = readSkymask(filename2);
			/*if (fileAvailaibility_)
				cout << "find file in left neighbour:  " << fileAvailaibility_ << "\n";*/
		}
		if (!fileAvailaibility_)
		{
			fileAvailaibility_ = readSkymask(filename3);
			/*if (fileAvailaibility_)
				cout << "find file in down neighbour :  " << fileAvailaibility_ << "\n";*/
		}
		if (!fileAvailaibility_)
		{
			fileAvailaibility_ = readSkymask(filename4);
			/*if (fileAvailaibility_)
				cout << "find file in up neighbour :  " << fileAvailaibility_ << "\n";*/
		}
		//std::cout << "       read sky mask file	 used  time -> " << float(clock() - begin_time) / CLOCKS_PER_SEC << "\n\n";

		if (fileAvailaibility_) // parse only if the file is available
		{
			//begin_time = clock();
			skyMaskS.clear();
			string2llh(skysmaskPosString, skysmaskString);
			//std::cout << "       parse sky mask used  time -> " << float(clock() - begin_time) / CLOCKS_PER_SEC << "\n\n";
			/*cout << "how many skymask in this file :  " << skyMaskS.size() << "\n\n";*/
			if (skyMaskS.size() == 1)
			{
				skymask_ = skyMaskS[0];
			}
		}
		
		return skymask_;
	}

	/*
	author: WEN Weisong (17902061r@connect.polyu.hk)
	function: calculate the expected satellite visibility
	input: MatrixXd eELAZAngles (if there is no building ), shadowMatching::skyMask skyMask_
	output: vector<int>

	// (PRN, EL, AZ)
	*/
	vector<int> visibilityCal(MatrixXd eELAZAngles, shadowMatching::skyMask skyMask_) // calculate the expected visibility
	{
		vector<int> result;
		for (int index = 0; index < eELAZAngles.rows(); index++)
		{
			int azi_ = 0;
			if (map_.aziReso == 1)
			{
				azi_ = (int)eELAZAngles(index, 2);
				if (eELAZAngles(index, 1) > skyMask_.aziElemask[azi_].elevation)
				{
					result.push_back(eELAZAngles(index, 0));
				}
				/*cout << "expected azimuth" << azi_ << "     azimuth in skymask " << skyMask_.aziElemask[azi_].azimuth<<"\n";*/
				//cout << "expected elevation" << eELAZAngles(index, 1) << "     elevation in skymask " << skyMask_.aziElemask[azi_].elevation << "\n";
			}
			else
			{
				cout << "please change with the azimuth resolution..." << "\n";
			}
		}
		
		return result;
	}


	/*
	author: WEN Weisong (17902061r@connect.polyu.hk)
	function: calculate the scores 
	input: MatrixXd eAllMeasurement, vector<int> visibility
	output: vector<int>
	*/
	double scoreCal(MatrixXd eAllMeasurement, vector<int> visibility) // calculate the scores of shadowmatching
	{
		double  score =0;
		if (visibility.size()) // at least one satellite 
		{
			for (int index = 0; index < eAllMeasurement.rows(); index++)
			{
				if (std::find(visibility.begin(), visibility.end(), eAllMeasurement(index, 0)) != visibility.end())
				{
					score++;
				}
			}
		}
		return score;
	}

	/*
	author: WEN Weisong (17902061r@connect.polyu.hk)
	function: Generate particle near the WLS solution
	input: WLS solution, radis of the circle, resolution of grids 
	output: particles 

	radis: 100m 
	resol: 2m 
	*/
	vector<particle> generateParticles(Eigen::MatrixXd WLSECEF, double radis, double resol, Eigen::MatrixXd eAllMeasurement, Eigen::MatrixXd eELAZAngles) // Generate particle near the WLS solution
	{
		vector<particle> particles_;
		// obtain the ENU of WLSECEF
		Eigen::MatrixXd ecef_; // the ecef
		//ecef_.resize(3, 1);
		ecef_ = WLSECEF;

		Eigen::MatrixXd LLHR_; // reference LLH in matrix:  
		LLHR_.resize(3, 1); // resize to 3X1
		LLHR_(0) = posLLHR_.lon;
		LLHR_(1) = posLLHR_.lat;
		LLHR_(2) = posLLHR_.alt;

		Eigen::MatrixXd enu; // enu
		enu.resize(3, 1); // resize to 3X1
		enu = ecef2enu(LLHR_, ecef_);
		cout << "enu-> " << enu<<"\n\n\n";

		int count = 0;
		clock_t begin_time = clock();
		for (double N = enu(1) - radis; N <= enu(1) + radis; N = N + map_.disReso) // search in the by line (N direction)
		{
			for (double E = enu(0) - radis; E <= enu(0) + radis; E = E + map_.disReso) // search in the by line (E direction)
			{
				particle particle_; // one particle
				double dis = sqrt((E - enu(0)) * (E - enu(0)) + (N - enu(1)) * (N - enu(1))); 
				if (dis < radis) // in circle
				{
					ENU ENU_;
					ENU_.E = E;
					ENU_.N = N;
					ENU_.U = 0;
					particle_.E = E;
					particle_.N = N;
					skyMask skymask_;
					vector<int> visibility;
					skymask_ = preparingSkymaskbyPose(ENU_);
					if(skymask_.aziElemask.size()>0)
						visibility = visibilityCal(eELAZAngles, skymask_); // calculate the expected visibility at each position
					particle_.eAllMeasurement = eAllMeasurement;
					particle_.eELAZAngles = eELAZAngles;
					particle_.visibility = visibility;
					double score = scoreCal(eAllMeasurement, visibility);
					particle_.scores = scoreCal(eAllMeasurement, visibility);
					// eAllMeasurement -> (n,3) PRN CNO Pseudorange
					// eELAZAngles -> // (PRN, EL, AZ)

				/*	cout << "eAllMeasurement.size-> " << eAllMeasurement.size() << "  eELAZAngles.size->  " << eELAZAngles.size() <<"       scores : "<< score << "\n";
					cout << "finish one hypothesis (particle)-------------------"<< ENU_.E <<"      "<< ENU_.N << "      "<< "  visibility.size()"<< visibility.size() <<"\n";*/
					if(visibility.size()) // at least one satellite available
						particles.push_back(particle_);
				}
				count++;
			}
		}
		/*MatrixXd llh_ = weightSM_();*/
		std::cout << std::setprecision(17); // make sure enough precision is provided 
		
		/*std::cout << "one circle  used  time -> " << float(clock() - begin_time) / CLOCKS_PER_SEC << "\n\n";*/
		cout << ".......finish one circle......" << count << "\n";

		return particles_;
	}

	/*
	author: WEN Weisong (17902061r@connect.polyu.hk)
	function: weight all the particles and obtain the final solution
	input: WLS solution, radis of the circle, resolution of grids
	output: particles

	radis: 100m
	resol: 2m
	*/
	MatrixXd weightSM_() // weight all the particles and obtain the final solution
	{
		//
		Eigen::MatrixXd llh; // 
		llh.resize(3, 1); // 

		ENU ENU_;
		Eigen::MatrixXd ecef; // 
		ecef.resize(3, 1); // 

		Eigen::MatrixXd orillh; // 
		orillh.resize(3, 1); // 
		orillh(0) = posLLHR_.lon;
		orillh(1) = posLLHR_.lat;
		orillh(2) = posLLHR_.alt;
		if (particles.size() > 0) // do only when there at least one particle 
		{
			particle particle_ = particles[0]; // one particle
			/*#pragma omp parallel for*/
			for (int index = 0; index < particles.size(); index++) // index all the particles to get maximum socre
			{
				if (particles[index].scores > particle_.scores)
					particle_ = particles[index];
			}
			Eigen::MatrixXd enu; // 
			enu.resize(3, 1); // 
			enu(0) = particle_.E;
			enu(1) = particle_.N;
			enu(2) = 0;
			ecef = enu2ecef(orillh, enu);
			llh = ecef2llh(ecef);
		}
		else
		{
			cout << "particle size is less than 1..."  << "\n";
		}
		return llh;
	}




};




