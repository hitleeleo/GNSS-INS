### Localization package (This package is still under development)

This package seeks to provide navigation solution in diverse scenarios using different sensors, with:
## Expected Capabilities (under development)
- evaluation of algorithm: filtering (eg.EKF) and smoothing (eg. factor graph)

## Using

- GNSS(GPS/BeiDou in Asian area): raw measurements is better (robust GNSS). 
- INS: using Xsens MTi 10. state-of-the-art pre-integration is applied.
- LiDAR odometry: using LiDAR odometry by matching two frame of point clouds.
- LiDAR matching: using LiDAR matching between map and real-time point clouds.

## Package description
   - amsi- adaptive multi-sensor integration package
   - nlosExclusion- GNSS NLOS exclusion package (under development)
   - rtklibros- rtklib in ros package [rtklib](https://github.com/tomojitakasu/RTKLIB)
   - Mapping- LiDAR mapping and its integration with GNSS (in local frame) (SLAM)

## Currently support
- loosely coupled GNSS/INS integration using EKF at [here](https://github.com/weisongwen/GNSS-INS/blob/master/amsi/apps/gps_imu_loose_ekf.cpp).
- tightly coupled GNSS/INS integration using EKF at [here](https://github.com/weisongwen/GNSS-INS/blob/master/amsi/apps/gps_imu_tight_ekf.cpp).
- loosely coupled GNSS/INS integration using factor graph at [here](https://github.com/weisongwen/GNSS-INS/blob/master/amsi/apps/gps_imu_loose_fg.cpp).
- tightly coupled GNSS/INS integration using factor graph at here (to be developed).

## How to use this for your data

The data for public will be opened soon,


## Research papers for reference

1. Wen, Weisong, Guohao Zhang, and Li-Ta Hsu. "Exclusion of GNSS NLOS receptions caused by dynamic objects in heavy traffic urban scenarios using real-time 3D point cloud: An approach without 3D maps." Position, Location and Navigation Symposium (PLANS), 2018 IEEE/ION. IEEE, 2018. (https://ieeexplore.ieee.org/abstract/document/8373377/)
2. Wen, W.; Hsu, L.-T.*; Zhang, G. (2018) Performance analysis of NDT-based graph slam for autonomous vehicle in diverse typical driving scenarios of Hong Kong. Sensors 18, 3928.
3. Wen, W., Zhang, G., Hsu, Li-Ta (Presenter), Correcting GNSS NLOS by 3D LiDAR and Building Height, ION GNSS+, 2018, Miami, Florida, USA.

4. Zhang, G., Wen, W., Hsu, Li-Ta, Collaborative GNSS Positioning with the Aids of 3D City Models, ION GNSS+, 2018, Miami, Florida, USA. (Best Student Paper Award)

5. Zhang, G., Wen, W., Hsu, Li-Ta, A Novel GNSS based V2V Cooperative Localization to Exclude Multipath Effect using Consistency Checks, IEEE PLANS, 2018, Monterey, California, USA.

## Claim

This package is for integration of multi-sensor information. As some of the code refers to some existing repositories. If there is any thing inappropriate, please contact me through 17902061r@connect.polyu.hk (Weisong WEN).


## LICENSE
### BSD License – PolyU

Copyright (c) 2018 [Weisong WEN](https://weisongwen.wixsite.com/weisongwen)

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

* Neither the name of the <organization> nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.