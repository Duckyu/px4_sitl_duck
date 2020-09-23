#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;
  ROS_INFO("start");
  ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::PoseStamped>("turtle2/cmd_vel", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
                  listener.waitForTransform("/base_link", "/posicao_objectivo",
                              ros::Time(0), ros::Duration(3.0));
                  listener.lookupTransform("/base_link", "/posicao_objectivo",
                                           ros::Time(0), transform);
                }
                catch (tf::TransformException &ex) {
                  ROS_ERROR("%s",ex.what());
                  ros::Duration(1.0).sleep();
                  continue;
                }

//             geometry_msgs::Twist vel_msg;
//                        vel_msg.angular.z = 0.2 * atan2(transform.getOrigin().y(),
//                                                            transform.getOrigin().x());
//                        vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
//                                                          pow(transform.getOrigin().y(), 2));
//                        robot_vel.publish(vel_msg);

//                        rate.sleep();
    }
    return 0;
}
