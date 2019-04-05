#include "rtklib.h"
#include <Eigen/Dense>


//void OutputSolutioninRTKLIBFormat(FILE* fp, int current_week, double current_tow, Eigen::MatrixXd  eWLSSolution){
//
//
//
//	FILE * fp5;
//	std::string rtklibName2 = main_directory + data_name + ".pos";
//
//	fopen_s(&fp5, rtklibName2.c_str(), "a");
//	if (fp5)
//	{
//
//			fprintf(fp5, "%4d, %9.3f,  %11.9f, %12.9f,  %7.4f\n",
//				current_week, current_tow, eWLSSolution(0), eWLSSolution(1), eWLSSolution(2));
//
//		fclose(fp5);
//	}
//
//}