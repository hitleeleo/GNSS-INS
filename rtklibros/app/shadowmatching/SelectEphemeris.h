
#include "rtklib.h"
#include <Eigen/Dense>

#define SECONDSINAWEEK 604800

void SelectEphemeris(nav_t* navs, int current_week, double current_tow, int* target_idx, int* target_glo_idx, int &iNumValidSV, int &iNumValidSV_glo)
{
	// GPS and QZSS case	
	//int iNumValidSV = 0;

	for (int idx = 0; idx < navs->n; idx++) {
		//if (navs->eph[idx].sat <= 32 || navs->eph[idx].sat == 84){

		int eph_week;
		double eph_toe, eph_toc, eph_ttr;

		eph_toe = time2gpst(navs->eph[idx].toe, &eph_week);
		eph_toc = time2gpst(navs->eph[idx].toc, &eph_week);
		eph_ttr = time2gpst(navs->eph[idx].ttr, &eph_week);
		//printf("%2d %3d - %f %f %f %f \n",idx,navs->eph[idx].sat,current_tow,eph_toe,eph_toc,eph_ttr);

		double diffTime = ((current_week*SECONDSINAWEEK + current_tow) - (eph_week*SECONDSINAWEEK + eph_toe));
		if (fabs(diffTime) <= 7200)
		{
			bool bSVExist = false;

			//delete the existing SV.
			for (int jdx = 0; jdx < iNumValidSV; jdx++) {
				//printf("%2d/%2d - %d = %d\n", jdx, iNumValidSV, navs->eph[target_idx[jdx]].sat,navs->eph[idx].sat);
				if (navs->eph[target_idx[jdx]].sat == navs->eph[idx].sat)
				{
					bSVExist = true;
					break;
				}
			}
			//printf("\n");

			//save the satellite for processing
			if (!bSVExist) {
				target_idx[iNumValidSV] = idx;
				iNumValidSV++;
			}
			//printf("%f %2d %3d - %f - %f %f %f \n", diffTime, idx, navs->eph[idx].sat, current_tow, eph_toe, eph_toc, eph_ttr);
		}
		//}

	}


	for (int idx = 0; idx < navs->ng; idx++)
	{

		int eph_week;
		double eph_toe, eph_tof;
		eph_toe = time2gpst(navs->geph[idx].toe, &eph_week);
		eph_tof = time2gpst(navs->geph[idx].tof, &eph_week);

		double diffTime = ((current_week*SECONDSINAWEEK + current_tow) - (eph_week*SECONDSINAWEEK + eph_toe));
		if (true)
			if (fabs(diffTime) <= 1000)
			{

				bool bSVExist = false;

				//delete the existing SV.
				for (int jdx = 0; jdx < iNumValidSV_glo; jdx++) {
					//printf("%2d/%2d - %d = %d\n", jdx, iNumValidSV, navs->eph[target_idx[jdx]].sat,navs->eph[idx].sat);
					if (navs->geph[target_glo_idx[jdx]].sat == navs->geph[idx].sat)
					{
						bSVExist = true;
						break;
					}
				}
				//printf("\n");

				//save the satellite for processing
				if (!bSVExist) {
					target_glo_idx[iNumValidSV_glo] = idx;
					iNumValidSV_glo++;
				}
				//printf("%f %2d %3d - %f - %f\n", diffTime, idx, navs->geph[idx].sat, current_tow, eph_toe);

			}
	}
	//printf("\n");
	//return iNumValidSV;
}

Eigen::MatrixXd SatellitePosition(nav_t* navs, int current_week, double current_tow, int* target_idx, int* target_glo_idx, int iNumValidSV, int iNumValidSV_glo)
{
	Eigen::MatrixXd eAllSVPositions;
	eAllSVPositions.resize(iNumValidSV + iNumValidSV_glo, 5);


	for (int idx = 0; idx < iNumValidSV; idx++)
	{
		int prn = navs->eph[target_idx[idx]].sat;

		double rs[6], dts[2], var;

		// GPS & QZSS case
		if (prn <= 32 || prn == 84)
		{
			eph_t eph = navs->eph[target_idx[idx]];

			gtime_t time_gps;
			double dt;
			double rst[3], dtst[1], tt = 1E-3;

			time_gps = gpst2time(current_week, current_tow - double(20200.0 * 1e3 / CLIGHT));
			//time_gps = gpst2time(current_week, current_tow );
			dt = eph2clk(time_gps, &eph);
			time_gps = timeadd(time_gps, -dt);
			eph2pos(time_gps, &eph, rs, dts, &var);

			int eph_weekQ;
			double eph_towQ;
			eph_towQ = time2gpst(time_gps, &eph_weekQ);
			//printf("eph2pos: time=%d %f sat=%2d %f %f %f\n", eph_weekQ, eph_towQ - tt, prn, rs[0], rs[1], rs[2]);	

			//this->m_dblClockBias = dts[0];
			//this->m_pointSatellite.SetEcef(rs[0], rs[1], rs[2]);
		}

		// Beidou
		else if ((prn <= 121) && (prn >= 87))
		{
			eph_t eph = navs->eph[target_idx[idx]];

			gtime_t time_BEIDOU;
			double dt;
			double rst[3], dtst[1], tt = 1E-3;
			//double rs[6], dts[2], var;

			time_BEIDOU = gpst2time(current_week, current_tow);
			dt = eph2clk(time_BEIDOU, &eph);
			time_BEIDOU = timeadd(time_BEIDOU, -dt);
			eph2pos(time_BEIDOU, &eph, rs, dts, &var);

			int eph_weekQ;
			double eph_towQ;
			eph_towQ = time2gpst(time_BEIDOU, &eph_weekQ);
			//printf("eph2pos: time=%d %f sat=%2d %f %f %f\n", eph_weekQ, eph_towQ - tt, prn, rs[0], rs[1], rs[2]);


			//this->m_dblClockBias = dts[0];
			//this->m_pointSatellite.SetEcef(rs[0], rs[1], rs[2]);

		}
		eAllSVPositions(idx, 0) = prn;
		for (int jdx = 0; jdx < 3; jdx++)
		{
			eAllSVPositions(idx, 1 + jdx) = rs[jdx];
		}
		eAllSVPositions(idx, 4) = dts[0];
	}
	//printf("\n");
	// glonass
	for (int idx = 0; idx < iNumValidSV_glo; idx++)
	{
		int prn = navs->geph[target_glo_idx[idx]].sat;

		double rs[6], dts[2], var;

		if (prn <= 56)
		{
			// glonass
			geph_t geph = navs->geph[target_glo_idx[idx]];

			//printf("prn = %2d target_glo_idx[idx] = %2d idx = %2d %f %f %f\n", prn, target_glo_idx[idx], idx,geph.pos[0], geph.pos[1], geph.pos[2]);
			gtime_t time_glo;
			double dt;
			double rst[3], dtst[1], tt = 1E-3;
			//double rs[6], dts[2], var;

			time_glo = gpst2time(current_week, current_tow);
			dt = geph2clk(time_glo, &geph);
			time_glo = timeadd(time_glo, -dt);
			geph2pos(time_glo, &geph, rs, dts, &var);

			int eph_weekQ;
			double eph_towQ;
			eph_towQ = time2gpst(time_glo, &eph_weekQ);
			//printf("geph2pos: time=%d %f sat=%2d %f %f %f\n", eph_weekQ, eph_towQ - tt, prn, rs[0], rs[1], rs[2]);


		}
		eAllSVPositions(iNumValidSV + idx, 0) = double(prn);
		for (int jdx = 0; jdx < 3; jdx++)
		{
			eAllSVPositions(iNumValidSV + idx, 1 + jdx) = rs[jdx];
		}
		eAllSVPositions(iNumValidSV + idx, 4) = dts[0];
	}
	//printf("\n");

	//for (int idx = 0; idx < eAllSVPositions.rows(); idx++){
	//			printf("%2d-[%3d] - (%10.2f,%10.2f,%10.2f) %f\n", idx,int(eAllSVPositions(idx, 0)), eAllSVPositions(idx, 1), eAllSVPositions(idx, 2), eAllSVPositions(idx, 3), eAllSVPositions(idx, 4));
	//}

	return eAllSVPositions;
}



Eigen::MatrixXd CalcELAZAngle(Eigen::MatrixXd  eRefPoLLH, Eigen::MatrixXd eAllSVPositions)
{
	int iNumSV = eAllSVPositions.rows();

	Eigen::MatrixXd eELAZAngles;
	eELAZAngles.resize(iNumSV, 3); // (PRN, EL, AZ)

								   // example of coordinate transformation from ECEF to LLH (please see more in rtklib.h. hint:F12 can trace back the function)
	double pos[3];
	for (int idx = 0; idx < 3; idx++) {
		pos[idx] = eRefPoLLH(idx);
		//printf("%d %f %f\n", idx, eRefPoLLH(idx), pos[idx]);
	}
	for (int idx = 0; idx < iNumSV; idx++) {

		double r[3], e[3];
		for (int jdx = 0; jdx < 3; jdx++) {
			r[jdx] = eAllSVPositions(idx, 1 + jdx);
			//printf("%d %f %f\n", idx, eRefPoLLH(idx), pos[idx]);
		}
		eELAZAngles(idx, 0) = eAllSVPositions(idx, 0);
		ecef2enu(pos, r, e);

		double az_r, el_r;
		Eigen::Vector3d enu;


		for (int jdx = 0; jdx < 3; jdx++) {
			enu(jdx) = e[jdx];
		}
		el_r = asin(enu(2) / enu.norm());
		az_r = atan2(enu(0), enu(1));
		eELAZAngles(idx, 1) = el_r * 180.0 / PI;
		eELAZAngles(idx, 2) = az_r * 180.0 / PI;
		if (eELAZAngles(idx, 2) < 0) {
			eELAZAngles(idx, 2) += 360;
		}
		//printf("[%2d] EL = %6.2f AZ = %6.2f\n", int(eELAZAngles(idx, 0)), eELAZAngles(idx, 1), eELAZAngles(idx, 2));

	}
	printf("\n");

	return eELAZAngles;
}