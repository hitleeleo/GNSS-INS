// map_.buildingS_M
    for(int i =0; i<map_.buildingS_M.size();i++)
    {
      for(int j=0; j<map_.buildingS_M[i].buildingENUV.size()-1;j++)
      {
        visualization_msgs::Marker marker,points;
        marker.header.frame_id = "world";

        points.header.frame_id = "world";
        points.header.stamp = ros::Time::now();
        points.ns = "points_and_lines";
        points.action  = visualization_msgs::Marker::ADD;
        points.pose.orientation.w  = 1.0;
        points.id = i*j*2;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 3;
        points.scale.y = 3;
        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;

        // marker.header.stamp = filtered->header.stamp;
        marker.ns = "Rays";
        marker.id = i*j;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = 0;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.5; // 0.1
        marker.points.resize(2);
       
        marker.colors.resize(2);
        marker.colors[0].a = 1;
        marker.colors[0].r = 1.0;
        marker.colors[0].g = 0.0;
        marker.colors[0].b = 0.0;

        marker.colors[1].a = 1;
        marker.colors[1].r = 1.0;
        marker.colors[1].g = 0.0;
        marker.colors[1].b = 0.0;
        // std::cout<<"loop buildings"<< map_.buildingS_M[i].buildingENUV[j].E<<std::endl;
        marker.points[0].x = map_.buildingS_M[i].buildingENUV[j].E;
        marker.points[0].y = map_.buildingS_M[i].buildingENUV[j].N;
        marker.points[0].z = map_.buildingS_M[i].buildingENUV[j].U;

        marker.points[1].x = map_.buildingS_M[i].buildingENUV[j+1].E;
        marker.points[1].y = map_.buildingS_M[i].buildingENUV[j+1].N;
        marker.points[1].z = map_.buildingS_M[i].buildingENUV[j+1].U;

        geometry_msgs::Point p;
        p.x = map_.buildingS_M[i].buildingENUV[j].E;
        p.y = map_.buildingS_M[i].buildingENUV[j].N;
        p.z = map_.buildingS_M[i].buildingENUV[j].U;
        points.points.push_back(p);

        if(j==1)
        {
          std::cout<<"j == "<< j<<std::endl;
          visualization_msgs::Marker marker;
          marker.header.frame_id = "world";
          // marker.header.stamp = filtered->header.stamp;
          marker.ns = "Rays";
          marker.id = i*j*5;
          marker.type = visualization_msgs::Marker::LINE_STRIP;
          marker.action = 0;
          marker.pose.position.x = 0.0;
          marker.pose.position.y = 0.0;
          marker.pose.position.z = 0.0;
          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;
          marker.scale.x = marker.scale.y = marker.scale.z = 0.5; // 0.1
          marker.points.resize(2);
         
          marker.colors.resize(2);
          marker.colors[0].a = 1;
          marker.colors[0].r = 1.0;
          marker.colors[0].g = 0.0;
          marker.colors[0].b = 0.0;

          marker.colors[1].a = 1;
          marker.colors[1].r = 1.0;
          marker.colors[1].g = 0.0;
          marker.colors[1].b = 0.0;
          // std::cout<<"loop buildings"<< map_.buildingS_M[i].buildingENUV[j].E<<std::endl;
          marker.points[0].x = map_.buildingS_M[i].buildingENUV[0].E;
          marker.points[0].y = map_.buildingS_M[i].buildingENUV[0].N;
          marker.points[0].z = map_.buildingS_M[i].buildingENUV[0].U;

          marker.points[1].x = map_.buildingS_M[i].buildingENUV[map_.buildingS_M[i].buildingENUV.size()].E;
          marker.points[1].y = map_.buildingS_M[i].buildingENUV[map_.buildingS_M[i].buildingENUV.size()].N;
          marker.points[1].z = map_.buildingS_M[i].buildingENUV[map_.buildingS_M[i].buildingENUV.size()].U;
        }

        markers.markers.push_back(marker);
        markers.markers.push_back(points);
        
      }
      
    }
    std::cout<<"markers.size "<<markers.markers.size()<<std::endl;
    pub_debug_marker_.publish(markers);


    // // Draw the uncertainty ellipse
    // visualization_msgs::Marker marker;
    // marker.header.frame_id = "velodyne";
    // marker.header.stamp = ros::Time();
    // marker.ns = "ellipse";
    // marker.id = 0;
    // marker.type = visualization_msgs::Marker::CUBE;
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.position.x = 0;
    // marker.pose.position.y = 0;
    // marker.pose.position.z = -1;
    // marker.pose.orientation.x = 0.0;
    // marker.pose.orientation.y = 0.0;
    // marker.pose.orientation.z = 0.0;
    // marker.pose.orientation.w = 1.0;
    // marker.scale.x = 20/1.3; // /10
    // marker.scale.y = 20; // /10
    // marker.scale.z = 20; // /10
    // marker.color.a = 0.6; // Don't forget to set the alpha!
    // marker.color.r = 1.0;
    // marker.color.g = 0.0;
    // marker.color.b = 0.0;
    // markers.markers.push_back(marker);
    // pub_debug_marker_.publish(markers);
