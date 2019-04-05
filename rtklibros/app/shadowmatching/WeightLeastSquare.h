
#include "rtklib.h"
#include "Eigen/Eigen"


static Eigen::MatrixXd WeightLeastSquare(Eigen::MatrixXd eAllSVPositions, Eigen::MatrixXd eAllMeasurement){
	
	Eigen::MatrixXd eWLSSolution;
	eWLSSolution.resize(5, 1);

	//for (int idx = 0; idx < eAllSVPositions.rows(); idx++){
	//	printf("%2d-[%3d] - (%10.2f,%10.2f,%10.2f) %f\n", idx, int(eAllSVPositions(idx, 0)), eAllSVPositions(idx, 1), eAllSVPositions(idx, 2), eAllSVPositions(idx, 3), eAllSVPositions(idx, 4));
	//}

	//for (int idx = 0; idx < eAllMeasurement.rows(); idx++){
	//	printf("%2d-[%3d] - C/N0 = %5.2f, Pr = %10.2f\n", idx, int(eAllMeasurement(idx, 0)), eAllMeasurement(idx, 1), eAllMeasurement(idx, 2));
	//}

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

	Eigen::MatrixXd validMeasurement;
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
	// std::cout << "validMeasurement ----------\n"<<validMeasurement<<std::endl;


	int iNumSV = validMeasurement.rows();

	/*Find the received SV and Sort based on the order of Measurement matrix*/
	Eigen::MatrixXd eExistingSVPositions;
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
	// std::cout << "eExistingSVPositions ----------\n"<<eExistingSVPositions<<std::endl;
	//for (int idx = 0; idx < eExistingSVPositions.rows(); idx++){
	//	printf("%2d-[%3d] - (%10.2f,%10.2f,%10.2f) %f\n", idx, int(eExistingSVPositions(idx, 0)), eExistingSVPositions(idx, 1), eExistingSVPositions(idx, 2), eExistingSVPositions(idx, 3), eExistingSVPositions(idx, 4)*CLIGHT);
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

			dGeoDistance = geodist(rs, rr, e);

			// Making H matrix			
			eH_Matrix(idx, 0) = -(rs[0] - rr[0]) / dGeoDistance;
			eH_Matrix(idx, 1) = -(rs[1] - rr[1]) / dGeoDistance;
			eH_Matrix(idx, 2) = -(rs[2] - rr[2]) / dGeoDistance;

			if (PRNisGPS(prn)){
				eH_Matrix(idx, 3) = 1;
				eH_Matrix(idx, 4) = 0;
			}
			else if (PRNisGLONASS(prn))
			{
				eH_Matrix(idx, 3) = 1;
				eH_Matrix(idx, 4) = 1;
			}

			// Making delta pseudorange
			double rcv_clk_bias;
			if (PRNisGPS(prn)){
				rcv_clk_bias = eWLSSolution(3);				
			}
			else if (PRNisGLONASS(prn))
			{
				rcv_clk_bias = eWLSSolution(3) + eWLSSolution(4);
			}
			double sv_clk_bias = eExistingSVPositions(idx, 4) * CLIGHT;
			eDeltaPr(idx, 0) = pr - dGeoDistance - rcv_clk_bias + sv_clk_bias;
			//printf("%2d - %f %f %f %f \n", prn, pr, dGeoDistance, eDeltaPr(idx, 0), rcv_clk_bias);
		}

			//printf("MatrixH = {\n");
			//for (int i = 0; i<iNumSV; i++)
			//{
			//	printf(" { \n");

			//	for (int j = 0; j<5; j++)
			//	{
			//		if (j != 0)
			//		{
			//			printf(",");
			//		}
			//		printf(" %f ", eH_Matrix(i, j));
			//	}
			//	printf(" }\n");
			//}

		// Least Square Estimation 
		eDeltaPos = (eH_Matrix.transpose() * eH_Matrix).ldlt().solve(eH_Matrix.transpose() *  eDeltaPr);
		//eDeltaPos = (eH_Matrix.transpose() * eH_Matrix).inverse() * eH_Matrix.transpose() *  eDeltaPr;
		//eDeltaPos = eH_Matrix.householderQr().solve(eDeltaPr);

		//for (int idx = 0; idx < eDeltaPos.rows(); idx++)
		//	printf("%f ", eDeltaPos(idx));
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
	printf("WLS -> (%11.2f,%11.2f,%11.2f)\n\n", eWLSSolution(0), eWLSSolution(1), eWLSSolution(2));

	return eWLSSolution;
}

//void FindExistingSVbasedonMeasurement(Eigen::MatrixXd eAllSVPositions, Eigen::MatrixXd eAllMeasurement){
//
//}