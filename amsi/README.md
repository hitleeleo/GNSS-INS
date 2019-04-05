### amsi package

## Capabilities
- gps_imu_loose_ekf.cpp loosely fuse gps and imu using ekf
```
/* ----------------------------------------------------------------------------

 * amsi Copyright 2019, Positioning and Navigation Laboratory,
 * Hong Kong Polytechnic University
 * All Rights Reserved
 * Authors: Weisong Wen, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file gps_imu_loose_ekf.cpp
 * @brief fuse gps (ENU) and imu (ENU) using ekf (loosely coupling)
 * @author Weisong Wen (weisong.wen@connect.polyu.hk)
 */

/**
 * Example of use of the imuFactors (imuFactor and combinedImuFactor) in conjunction with GPS
 
 *  - we read IMU and GPS data from rosbag, with the following format:
 *  A topic with "/imu/data" is an imu measurement
 *  linAccN, linAccE, linAccD, angVelN, angVelE, angVelD
 *  A topic with "/ublox_gps_node/fix" is a gps correction formatted with
 *  lat, lon, altitude
 */
```
- gps_imu_tight_ekf.cpp tightly fuse gps and imu using ekf
```
/* ----------------------------------------------------------------------------

 * amsi Copyright 2019, Positioning and Navigation Laboratory,
 * Hong Kong Polytechnic University
 * All Rights Reserved
 * Authors: Weisong Wen, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file gps_imu_tight_ekf.cpp
 * @brief fuse gps (ENU) and imu (ENU) using ekf (tight coupling)
 * @author Weisong Wen (weisong.wen@connect.polyu.hk)
 */

/**
 * Example of use of the imuFactors (imuFactor and combinedImuFactor) in conjunction with GPS
 
 *  - we read IMU and GPS data from rosbag, with the following format:
 *  A topic with "/imu/data" is an imu measurement
 *  linAccN, linAccE, linAccD, angVelN, angVelE, angVelD
 *  A topic with "/GNSS_" is a gps correction formatted with format nlosExclusion/GNSS_Raw_Array
 *  
  std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
  nlosExclusion/GNSS_Raw[] GNSS_Raws
  float64 GNSS_time
  float64 total_sv
  float64 prn_satellites_index
  float64 pseudorange
  float64 snr
  float64 elevation
  float64 azimuth
  float64 err_tropo
  float64 err_iono
  float64 sat_clk_err
  float64 sat_pos_x
  float64 sat_pos_y
  float64 sat_pos_z
  int64 visable
  string sat_system
 */
```
- gps_imu_loose_fg.cpp loosely fuse gps and imu using factor graph
```
/* ----------------------------------------------------------------------------

 * amsi Copyright 2019, Positioning and Navigation Laboratory,
 * Hong Kong Polytechnic University
 * All Rights Reserved
 * Authors: Weisong Wen, et al. (see THANKS for the full author list)

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
```


## How to use this for your data

The data is saved in Dropbox. The data for public will be opened soon,


## Research papers for reference

1. Wen, Weisong, Guohao Zhang, and Li-Ta Hsu. "Exclusion of GNSS NLOS receptions caused by dynamic objects in heavy traffic urban scenarios using real-time 3D point cloud: An approach without 3D maps." Position, Location and Navigation Symposium (PLANS), 2018 IEEE/ION. IEEE, 2018. (https://ieeexplore.ieee.org/abstract/document/8373377/)
2. Wen, W.; Hsu, L.-T.*; Zhang, G. (2018) Performance analysis of NDT-based graph slam for autonomous vehicle in diverse typical driving scenarios of Hong Kong. Sensors 18, 3928.
3. Wen, W., Zhang, G., Hsu, Li-Ta (Presenter), Correcting GNSS NLOS by 3D LiDAR and Building Height, ION GNSS+, 2018, Miami, Florida, USA.

4. Zhang, G., Wen, W., Hsu, Li-Ta, Collaborative GNSS Positioning with the Aids of 3D City Models, ION GNSS+, 2018, Miami, Florida, USA. (Best Student Paper Award)

5. Zhang, G., Wen, W., Hsu, Li-Ta, A Novel GNSS based V2V Cooperative Localization to Exclude Multipath Effect using Consistency Checks, IEEE PLANS, 2018, Monterey, California, USA.

## Claim

ASPANS is for adaptively integration of multi-sensor information. As some of the code refers to some existing repositories, including [Autoware](https://github.com/CPFL/Autoware). If there is any thing inappropriate, please contact me through 17902061r@connect.polyu.hk (Weisong WEN).


## LICENSE
### BSD License – PolyU

Copyright (c) 2018 [Weisong WEN](https://gitlab.com/wenweisong)

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

* Neither the name of the <organization> nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.