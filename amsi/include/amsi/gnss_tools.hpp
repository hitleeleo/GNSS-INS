#ifndef GNSS_Tools_HPP
#define GNSS_Tools_HPP

#define D2R 3.1415926/180.0
/**
 * @brief GNSS Tools
 * @note  GNSS related functions
 */
class GNSS_Tools {
public:
  GNSS_Tools() {
    double reserve = 0.01;
  }

public:
  /*
author: WEN Weisong, visiting Ph.D student in Univeristy of California, Berkeley. (weisong.wen@berkeley.edu)
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
author: WEN Weisong, visiting Ph.D student in Univeristy of California, Berkeley. (weisong.wen@berkeley.edu)
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
author: WEN Weisong, visiting Ph.D student in Univeristy of California, Berkeley. (weisong.wen@berkeley.edu)
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
author: WEN Weisong, visiting Ph.D student in Univeristy of California, Berkeley. (weisong.wen@berkeley.edu)
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
};

#endif // POSE_SYSTEM_HPP
