#pragma comment(lib,"winmm.lib")
#pragma comment(lib,"wsock32.lib")

#include "rtklib/rtklib.h"
#include <Eigen/Dense>

#include "utilities.h"
#include "WeightLeastSquare.h"
#include "SelectEphemeris.h"

#include "shadowMatching.h"


//#define TARGETWEEK 1997
//#define STARTTIME 218012
//#define ENDTIME   218014

std::string main_directory = "result\\";
std::string data_name = "v2v107M";

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



/* main function */
int main(int argc, char * argv[])
{

#ifdef STARTTIME
	gtime_t ts = gpst2time(TARGETWEEK, STARTTIME);
#else	
	gtime_t ts = { 0 }; ts.time = 0;							// start time (0:all)
#endif

#ifdef 	ENDTIME
	gtime_t te = gpst2time(TARGETWEEK, ENDTIME);
#else
	gtime_t te = { 0 }; te.time = 0;							// end time (0:all)	
#endif

		/* Configuration of RTKLIB ------------------------*/
	int n = 0, i, stat;

	double ti = 0.0;						// processing interval  (s) (0:all)
	double tu = 0.0;						// unit time (s) (0:all)	

	/* options */
	prcopt_t prcopt = prcopt_default;	// processing option
	solopt_t solopt = solopt_default;	// output solution option
	filopt_t filopt = { "" };	            // file option

	prcopt.mode = 2;				// 0: SPP 1: DGNSS 2:Kinematic RTK
	prcopt.navsys = SYS_ALL;
	prcopt.nf = 1;						// frequency (1:L1,2:L1+L2,3:L1+L2+L5) 
	prcopt.soltype = 0;					// 0:forward,1:backward,2:combined
	prcopt.elmin = 10.0*D2R;				// elevation mask (rad)	
	prcopt.tidecorr = 0;					// earth tide correction (0:off,1-:on) 
	prcopt.posopt[4] = 0;               // use RAIM FDE (qmo)
	prcopt.tropopt = TROPOPT_SAAS;        // troposphere option: Saastamoinen model
	prcopt.ionoopt = IONOOPT_BRDC;		// ionosphere option: Broad cast
	prcopt.sateph = EPHOPT_BRDC;			// ephemeris option: broadcast ephemeris

	prcopt.modear = 1;					// AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold)

	// HKSC
	prcopt.rb[0] = -2414266.9197;           // base position for relative mode {x,y,z} (ecef) (m)
	prcopt.rb[1] = 5386768.9868;			// base position for relative mode {x,y,z} (ecef) (m)
	prcopt.rb[2] = 2407460.0314;			// base position for relative mode {x,y,z} (ecef) (m)

	solopt.outopt = 1;					// output processing options (0:no,1:yes)
	solopt.timef = 0;						// time format (0:sssss.s,1:yyyy/mm/dd hh:mm:ss.s)
	solopt.timeu = 3;						// time digits under decimal point
	solopt.sep[0] = ',';					// field separator
	solopt.sstat = 0;						// solution statistics level (0:off,1:states,2:residuals)
	solopt.trace = 0;						// debug trace level (0:off,1-5:debug)
	solopt.sstat = 0;						// get the solution file
	solopt.posf = SOLF_XYZ;
	solopt.height = 0;

	char *rov = "", *base = "";
	char infile_[5][1024] = { "" }, *infile[5];
	char outfile[1024];

	shadowMatching SM_;

	/* set input files */
	for (i = 0; i < 5; i++) infile[i] = infile_[i];
	strcpy(infile[n++], "data\\v2v107M2.obs");
	strcpy(infile[n++], "data\\hksc107m.18n");
	strcpy(infile[n++], "data\\hksc107m.18g");

	//strcpy(infile[n++], "data\\TSTb120L.obs");
	//strcpy(infile[n++], "data\\hksc1200.18n");
	//strcpy(infile[n++], "data\\hksc1200.18g");

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
	if (!readobsnav(ts, te, ti, infile, index, n, &prcopt, &obss, &navs, stas)) return 0;
	printf("obss.n = %d\n", obss.n);
	printf("navs.n navs.ng = %d %d\n", navs.n, navs.ng);

	// show raw measurement obss nav

	/*Initializa the output file in RTKLIB format*/
	FILE * fp;
	//std::string rtklibName2 = main_directory + data_name + ".pos";
	std::string rtklibName2 = main_directory + "SM_" + ".pos";
	fopen_s(&fp, rtklibName2.c_str(), "w");
	solopt.posf = SOLF_LLH;
	if (fp)
	{
		outheader_Q(fp, infile, n, &prcopt, &solopt, &obss);
		fclose(fp);
	}
	/*---------------------------*/

	/**************************************************/

	/* Step2: starting to use obs and nav rinex data @@@*/

	int i_obs, j_obs; // (i_obs,j_obs) are the (start,end) of rover obs.
	//for (i_obs = 0; i_obs<obss.n; i_obs++)
	//	printf("%d %d \t", i_obs, obss.data[i_obs].rcv);
	for (i_obs = 0; i_obs < obss.n; i_obs++)
		if (obss.data[i_obs].rcv == 1)
			break;

	for (j_obs = obss.n - 1; j_obs >= 0; j_obs--)
		if (obss.data[j_obs].rcv == 1)
			break;

	bool bFindSameData = false;

	int IndexStart = i_obs, IndexEnd = 0;
	int idx_epoch = 0;
	// note that the for is not every epoch of gps time
	for (int idx = i_obs; idx < j_obs + 1; idx++)
	{
		if (idx == obss.n + 1)
		{
			break;
		}

		int current_week1, current_week2;
		double current_tow1, current_tow2;
		current_tow1 = time2gpst(obss.data[idx].time, &current_week1);
		current_tow2 = time2gpst(obss.data[idx + 1].time, &current_week2);

		if ((current_week1 != current_week2) || fabs(current_tow1 - current_tow2) >0.49) // assuming 1Hz data rate. (if data rate higher than 2 Hz this does not valid)
		{
			IndexEnd = idx; // for current epoch
			bFindSameData = true;
		}
		//printf("IndexStart = %d, IndexEnd = %d\n", IndexStart, IndexEnd);

		/* Step3:find the measurements at the same epoch @@@*/

		if (bFindSameData)
		{
			idx_epoch++;
			int current_week;
			double current_tow;
			current_tow = time2gpst(obss.data[IndexStart].time, &current_week);
			printf("current_week = %4d - current_tow = %6.0f\n", current_week, current_tow);

			/* Step3.1:construct measurement matrix  @@@*/
			Eigen::MatrixXd eAllMeasurement; // (n,3) PRN CNO Pseudorange
			eAllMeasurement = ConstructMeasurementMatrix(IndexStart, IndexEnd, &obss);

			/* Step3.2: Select Ephemeris*/
			int target_idx[100], target_glo_idx[100]; // maximun number of satellite is 100. (GPS and GLONASS) 
			int iNumValidSV = 0, iNumValidSV_glo = 0;
			SelectEphemeris(&navs, current_week, current_tow, target_idx, target_glo_idx, iNumValidSV, iNumValidSV_glo);

			/* Step3.3: caculating satellite positions @@@*/
			// caculating satellite positions (I suggest to make a structure about satellite position related information)
			Eigen::MatrixXd eAllSVPositions; // (n,5) prn, sx, sy, sz, 
			eAllSVPositions = SatellitePosition(&navs, current_week, current_tow, target_idx, target_glo_idx, iNumValidSV, iNumValidSV_glo);

			/* Step3.4: WLS @@@
			initial guess for shadow Matching is availalble,
			*/
			Eigen::MatrixXd  eWLSSolutionECEF; // 6 unknowns with two clock bias variables
			eWLSSolutionECEF = WeightLeastSquare(eAllSVPositions, eAllMeasurement);

			// example of coordinate transformation from ECEF to LLH (please see more in rtklib.h. hint:F12 can trace back the function)
			Eigen::MatrixXd eWLSSolutionLLH(3, 1);
			double pos[3], r[3];
			for (int idx = 0; idx < 3; idx++) {
				pos[idx] = eWLSSolutionECEF(idx);
				//printf("%d %f %f\n", idx, eWLSSolutionECEF(idx), pos[idx]);
			}
			ecef2pos(pos, r);
			for (int idx = 0; idx < 3; idx++) {
				eWLSSolutionLLH(idx) = r[idx];
				//printf("%d %f %f\n", idx, eWLSSolutionLLH(idx), r[idx]);
			}

			// example to calculate azimuth and angle 2018/05/17 by LT Hsu
			Eigen::MatrixXd eELAZAngles;
			eELAZAngles = CalcELAZAngle(eWLSSolutionLLH, eAllSVPositions);

			// Add by Weisong WEN
			clock_t begin_time = clock();
			vector<shadowMatching::particle> particlesMain;
			//cout << "eAllMeasurement.size-> " << eAllMeasurement.rows() << "  eELAZAngles.size->  " << eELAZAngles.rows() << "\n";
			MatrixXd llh_;
			MatrixXd ecef_;
			for (int index = 0; index < 2; index++)
			{
				particlesMain = SM_.generateParticles(eWLSSolutionECEF, 50, 2, eAllMeasurement, eELAZAngles);
				llh_ = SM_.weightSM_();
				ecef_ = SM_.llh2ecef(llh_);
				SM_.particles.clear();

			}
			std::cout << "shadow matching  result: \n" << llh_ << "\n\n";
			std::cout << "one circle  used  time ---------------------------------------------------------------------------------------------> " << float(clock() - begin_time) / CLOCKS_PER_SEC << "\n\n";
			//Output as RTKLIB format
			fopen_s(&fp, rtklibName2.c_str(), "a");
			if (fp)
			{
				//fprintf(fp, "%4d, %9.3f,  %11.9f, %12.9f,  %9.4f, %d\n", current_week, current_tow, eWLSSolutionLLH(0)*R2D, eWLSSolutionLLH(1)*R2D, eWLSSolutionLLH(2),1);
				fprintf(fp, "%4d, %9.3f,  %11.9f, %12.9f,  %9.4f, %d\n", current_week, current_tow, llh_(1), llh_(0), llh_(2), 1);
				fclose(fp);
			}

			// End of an epoch
			IndexStart = idx + 1; // don't touch this InderStart
			//printf("\n");
			bFindSameData = false;

			// coordinate convertion.

		}



	}
	/**************************************************/

	//shadowMatching SM_;
	system("pause");
	return 0;
}

/* dummy application functions for shared library ----------------------------*/
extern int showmsg(char *format, ...) {
	va_list arg;
	char buff[1024];
	if (*format) {
		va_start(arg, format);
		vsprintf(buff, format, arg);
		va_end(arg);
		printf("%s\n", buff);
	}
	return 0;
}
extern void settspan(gtime_t ts, gtime_t te) {}
extern void settime(gtime_t time) {}
