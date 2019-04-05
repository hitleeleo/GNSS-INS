#include <rtklib.h>
#include <Eigen/Dense>

static bool PRNisGPS(int prn)
{
	if (prn <= 32 || prn == 84)
		return true;
	else{
		return false;
	}	
}

static bool PRNisGLONASS(int prn)
{
	if (prn > 32 && prn <= 56)
		return true;
	else{
		return false;
	}
}

static bool PRNisBeidou(int prn)
{
	if ((prn <= 121) && (prn >= 87))
		return true;
	else{
		return false;
	}
}

// construct measurement matrix 
static Eigen::MatrixXd ConstructMeasurementMatrix(int IndexStart, int IndexEnd, obs_t* obss){

	Eigen::MatrixXd eAllMeasurementTemp;
	//printf("IndexStart = %d, IndexEnd = %d\n", IndexStart, IndexEnd);
	eAllMeasurementTemp.resize(IndexEnd - IndexStart, 3);
	int sv_idx = 0;
	for (int jdx = IndexStart; jdx < IndexEnd; jdx++){
		int prn = obss->data[jdx].sat;
		if (PRNisGPS(prn) || PRNisGLONASS(prn)){
			//printf("[%3d] - C/N0 = %3d - Pr = %11.2f\n", obss->data[jdx].sat, (obss->data[jdx].SNR[0] / 4), obss->data[jdx].P[0]);
			eAllMeasurementTemp(sv_idx, 0) = double(obss->data[jdx].sat); // PRN
			eAllMeasurementTemp(sv_idx, 1) = double((obss->data[jdx].SNR[0] / 4)); // C/N0
			eAllMeasurementTemp(sv_idx, 2) = obss->data[jdx].P[0]; // pseudorange
 			sv_idx++;
		}
	}

	// 
	Eigen::MatrixXd eAllMeasurement;
	eAllMeasurement.resize(sv_idx, 3);	
	for (int idx = 0; idx < sv_idx; idx++){
		eAllMeasurement(idx, 0) = eAllMeasurementTemp(idx, 0);
		eAllMeasurement(idx, 1) = eAllMeasurementTemp(idx, 1);
		eAllMeasurement(idx, 2) = eAllMeasurementTemp(idx, 2);
	}

	//printf("\n");
	return eAllMeasurement;
}

////	 show the raw measurement from rinex data
//for (int idx = 0; idx < navs.n; idx++)
//{
//	int current_week;
//	double current_tow;
//	current_tow = time2gpst(navs.eph[idx].toe, &current_week);
//	printf("%4d %4d-%10.3f ", navs.eph[idx].week, current_week, current_tow);
//	printf("%2d %2d %f %f\n", navs.eph[idx].sat, navs.eph[idx].iode, navs.eph[idx].M0);
//}
//for (int idx = 0; idx < obss.n; idx++)
//{
//	int current_week;
//	double current_tow;
//	current_tow = time2gpst(obss.data[idx].time,&current_week);
//	printf("%4d-%10.3f ", current_week,current_tow);
//	printf("%2d %2d %2d %f %f\n",obss.data[idx].sat,obss.data[idx].rcv,obss.data[idx].SNR[0]/4,obss.data[idx].P[0],obss.data[idx].D[0]);
//}