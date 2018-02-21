#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf/transform_broadcaster.h>
#include <cstdlib> 
#include <vector>
#include <cmath> 
ros::Publisher *pose_pub;

geometry_msgs::PoseStamped getpose(std::vector<double> positions)
{
geometry_msgs::PoseStamped pose;
double r,h;
r=0.05-0.28*sin(positions[1])-0.30896*sin(positions[1]-M_PI/2+positions[2]+0.09725319825);
h=0.137+0.28*cos(positions[1])+0.30896*cos(positions[1]-M_PI/2+positions[2]+0.09725319825);
pose.pose.position.x=r*sin(-positions[0]);
pose.pose.position.y=r*cos(-positions[0]);
pose.pose.position.z=h;
std::cout<<positions[1]<<"--"<<positions[2]<<std::endl;
tf::Quaternion q1,q2,q3,q4,q5;
q1.setRPY(0,0,positions[0]);
q2.setRotation( tf::Vector3(1,0,0),positions[1]+positions[2]);
q1=q1*q2;
q2.setRotation( tf::Vector3(0,1,0),positions[3]);
q1=q1*q2;
q2.setRotation( tf::Vector3(1,0,0),positions[4]);
q1=q1*q2;
q2.setRotation( tf::Vector3(0,1,0),positions[5]);
q1=q1*q2;
q2.setRotation( tf::Vector3(0,0,1),M_PI/2);
q1=q1*q2;
pose.pose.orientation.x=q1.x();
pose.pose.orientation.y=q1.y();
pose.pose.orientation.z=q1.z();
pose.pose.orientation.w=q1.w();
return pose;
}
void gettrace(const moveit_msgs::DisplayTrajectory::ConstPtr& input)
{
 // ROS_INFO("I heard: [%s]", msg->data.c_str());
//std::cout<<input->trajectory[0].joint_trajectory.points[0];
geometry_msgs::PoseArray poses;
poses.poses.resize(input->trajectory[0].joint_trajectory.points.size());
	for(int i=0;i<input->trajectory[0].joint_trajectory.points.size();i++)
	{
		poses.poses[i]=getpose(input->trajectory[0].joint_trajectory.points[i].positions).pose;
		poses.header.stamp=ros::Time::now();
		poses.header.frame_id="arm_link";
		pose_pub->publish(poses);
	}
}
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
 // ros::ServiceClient client = n.serviceClient<joint_state_controller::Pose_Desired3d>("pose_arm");
 ros::Publisher pub0 = n.advertise<geometry_msgs::PoseArray>("terminatio", 3);
 pose_pub=&pub0;

 ros::Subscriber sub = n.subscribe("/move_group/display_planned_path", 10, gettrace);
ros::spin();
 ros::Rate r(30);	
  while(n.ok())
  {
	 // pose_pub.publish(pose.request.pose);
	  r.sleep();
  }
  return 0;
}
