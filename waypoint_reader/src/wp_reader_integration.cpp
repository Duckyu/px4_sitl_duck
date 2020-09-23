#include <ros/ros.h>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <iostream>
#include "geometry_msgs/Pose.h"
#include "px4_sitl_duck/linenumin_waypointout.h"

using namespace std;

geometry_msgs::Pose wp_list[100];
geometry_msgs::Pose temp_wp;
double value;
int rmd;
int seq;
bool file_die=false;
int wp_size =0;
bool request_wp(px4_sitl_duck::linenumin_waypointout::Request &req, px4_sitl_duck::linenumin_waypointout::Response &res){
  if(wp_size<req.line_num){
    res.total_line=wp_size;
    res.exist=false;
  }
  else{
    res.exist=true;
    res.waypoint = wp_list[req.line_num];
  }
  ROS_INFO("request validity : %d, requested waypoint : %d",res.exist,req.line_num);
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "waypoint_reader");
  ros::NodeHandle nh;
  string filepath;
  nh.getParam("/wp_file_path",filepath);

    string content="";
    ifstream openFile(filepath);
    while(openFile.is_open()) {
        openFile >> content;
        ROS_INFO("seq : %d",seq);
        if(seq>3){
            value = stod(content);
            rmd = seq%4;
            ROS_INFO("rmd : %d",rmd);
            wp_size = (seq-rmd)/4-1;
            ROS_INFO("wp_size : %d",wp_size);
            switch(rmd){
            case 0:
              temp_wp.position.x = value;
              break;
            case 1:
              temp_wp.position.y = value;
              break;
            case 2:
              temp_wp.position.z = value;
              break;
            case 3:
              temp_wp.orientation.w = value;
              wp_list[wp_size].position.x=temp_wp.position.x;
              wp_list[wp_size].position.y=temp_wp.position.y;
              wp_list[wp_size].position.z=temp_wp.position.z;
              wp_list[wp_size].orientation.w=temp_wp.orientation.w;
              cout<<"wp_list"<<wp_list[wp_size].position.x<<", "<<wp_list[wp_size].position.y<<", "<<wp_list[wp_size].position.z<<", "<<wp_list[wp_size].orientation.w<<endl;
              if(wp_size>0){
                  if((wp_list[wp_size].position.x==wp_list[wp_size-1].position.x)&&(wp_list[wp_size].position.y==wp_list[wp_size-1].position.y)&&(wp_list[wp_size].position.z==wp_list[wp_size-1].position.z)&&(wp_list[wp_size].orientation.w==wp_list[wp_size-1].orientation.w)){
                    file_die=true;
                    cout<<"file reading terminated"<<endl;
                  }
              }
              break;
            }
          }
          seq++;
        cout<<openFile.is_open()<<endl;
        if (file_die == true) openFile.close();
    }
    wp_size -= 2;
    cout << "wp_size : "<< wp_size << endl;
  ros::ServiceServer wp_reader=nh.advertiseService("/wp_reader/request",request_wp);
  ros::spin();
}
