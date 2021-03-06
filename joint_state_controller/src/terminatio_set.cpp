#include "ros/ros.h"
#include <joint_state_controller/Pose_Desired3d.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <cstdlib> 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_base_master_set_coordinate");
char buf[20];
double px,py,pz,ox,oy,oz;
  if (argc != 8||
		sscanf(argv[1],"%s", buf) != 1||
		sscanf(argv[2],"%lf", &px) != 1||
		sscanf(argv[3],"%lf", &py) != 1|| 
		sscanf(argv[4],"%lf", &pz) != 1||
		sscanf(argv[5],"%lf", &ox) != 1||
		sscanf(argv[6],"%lf", &oy) != 1||
		sscanf(argv[7],"%lf", &oz) != 1) 
	{
		fprintf(stderr, "Usage: frameid  pose:px py pz ox oy oz\n");
		exit(1);
	}

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<joint_state_controller::Pose_Desired3d>("pose_arm");
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("terminatio", 3);
  joint_state_controller::Pose_Desired3d pose;
  pose.request.pose.header.frame_id=buf;
  pose.request.pose.pose.position.x=px;
  pose.request.pose.pose.position.y=py;
  pose.request.pose.pose.position.z=pz;
  pose.request.pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(ox,oy,oz);
  pose.request.pose.header.stamp= ros::Time::now();
  std::cout<<pose.request.pose.pose<<std::endl;
  if (client.call(pose))
  {
ROS_INFO("OK");
   // ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }
 ros::Rate r(30);	
  while(n.ok())
  {
	  pose_pub.publish(pose.request.pose);
	  r.sleep();
  }
  return 0;
}
