#include <iostream>
#include <vector>
#include <list>
#include <signal.h>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <joint_state_controller/Pose_Desired3d.h>

#define DEBUG

using namespace std;
//########ROS
ros::NodeHandle* nh;
ros::Publisher* joint_pub_ptr;
#ifdef DEBUG
ros::Publisher* pose_pub_ptr;
#endif
tf::TransformListener *listener;
//########Param
double publish_frequency;
string publish_topic_name;
string subscript_topic_name;
string base_frame_id;
//string child_frame_id="arm_link";
vector<string> temp;
struct Joint_setting
{
	int num;
	vector<double> pose;
	vector<double> eph;
	vector<double> sub;
	vector<double> epl;
	vector<double> vmax;
	vector<double> amax;
}joint_setting;
//########Data
struct Joint_state
{
	sensor_msgs::JointState out;
	sensor_msgs::JointState old;
	sensor_msgs::JointState in;
	sensor_msgs::JointState desired;
}joint_state;
struct Terminatio
{
	geometry_msgs::Pose pose_Desired;
}terminatio;

//########Param_process
int get_Param()
{
int error=0;
	nh->getParam("joint_state_controller/publish_frequency", publish_frequency);
	nh->getParam("joint_state_controller/publish_topic_name", publish_topic_name);
	nh->getParam("joint_state_controller/base_frame_id", base_frame_id);
	nh->getParam("joint_state_controller/subscript_topic_name", subscript_topic_name);
	nh->getParam("joint_state_controller/joint_setting/num", joint_setting.num);
	nh->getParam("joint_state_controller/joint_setting/eph", joint_setting.eph);
	if(joint_setting.eph.size()!=joint_setting.num){ROS_ERROR("wrong element number of \"eph\"");error=1;}
	nh->getParam("joint_state_controller/joint_setting/sub", joint_setting.sub);
	if(joint_setting.sub.size()!=joint_setting.num){ROS_ERROR("wrong element number of \"sub\"");error=1;}
	nh->getParam("joint_state_controller/joint_setting/epl", joint_setting.epl);
	if(joint_setting.epl.size()!=joint_setting.num){ROS_ERROR("wrong element number of \"epl\"");error=1;}
	nh->getParam("joint_state_controller/joint_setting/vmax", joint_setting.vmax);
	if(joint_setting.vmax.size()!=joint_setting.num){ROS_ERROR("wrong element number of \"vmax\"");error=1;}
	nh->getParam("joint_state_controller/joint_setting/amax", joint_setting.amax);
	if(joint_setting.amax.size()!=joint_setting.num){ROS_ERROR("wrong element number of \"amax\"");error=1;}

	joint_state.desired.position.resize(joint_setting.num);
	joint_state.desired.velocity.resize(joint_setting.num);
	joint_state.desired.effort.resize(joint_setting.num);
	joint_state.in=joint_state.desired;
	joint_state.out=joint_state.desired;
	joint_state.old=joint_state.out;

	nh->getParam("joint_state_controller/joint_setting/joint_name", joint_state.out.name);
	if(joint_state.out.name.size()!=joint_setting.num){ROS_ERROR("wrong element number of \"joint_name\"");error=1;}
	if(error!=0) ros::shutdown();
}

int set_Param()
{
	nh->setParam("joint_state_controller/publish_frequency", publish_frequency);
	nh->setParam("joint_state_controller/publish_topic_name", publish_topic_name);
	nh->setParam("joint_state_controller/base_frame_id", base_frame_id);
	nh->setParam("joint_state_controller/subscript_topic_name", subscript_topic_name);
	nh->setParam("joint_state_controller/joint_setting/num", joint_setting.num);
	nh->setParam("joint_state_controller/joint_setting/eph", joint_setting.eph);
	nh->setParam("joint_state_controller/joint_setting/sub", joint_setting.sub);
	nh->setParam("joint_state_controller/joint_setting/epl", joint_setting.epl);
	nh->setParam("joint_state_controller/joint_setting/vmax", joint_setting.vmax);
	nh->setParam("joint_state_controller/joint_setting/amax", joint_setting.amax);
	nh->setParam("joint_state_controller/joint_setting/joint_name", joint_state.out.name);
}


//########Core
bool calculate_joints(geometry_msgs::PoseStamped &pose)
{
	geometry_msgs::PoseStamped object_pose;	
	string object_frame_id=base_frame_id;
	try
	{
		listener->transformPose(object_frame_id, pose, object_pose);
#ifdef DEBUG	
		ROS_INFO("\nsource_point: %12s   %.2f, %.2f. %.2f---%.2f, %.2f, %.2f, %.2f \nobject_point: %12s   %.2f, %.2f, %.2f---%.2f, %.2f, %.2f, %.2f ",
		pose.header.frame_id.c_str(),
		pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
		pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w,
		object_pose.header.frame_id.c_str(),
		object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z,
		object_pose.pose.orientation.x, object_pose.pose.orientation.y, object_pose.pose.orientation.z,object_pose.pose.orientation.w);	
#endif	
	}
	catch(tf::TransformException& ex)
	{
	   	ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
	return false;  					
	}
	double px,py,pz;
	px=object_pose.pose.position.x;
	py=object_pose.pose.position.y;
	pz=object_pose.pose.position.z;
	joint_state.in.position[0]=-atan2(px,py);
	double offset_x,offset_y,offset_z,r1,r2,a,b,th,th0;
	offset_x=0.0;
	offset_y=0.05;
	offset_z=0.137;
	a=0.28;
	b=0.30896;
	r1=sqrt(px*px+py*py)-offset_y;
	r2=sqrt(r1*r1+(pz-offset_z)*(pz-offset_z));
	if(r2>=a+b)return false;
	th=atan2(pz-offset_z,r1);
	th0=0.09725319825;
	joint_state.in.position[1]=-M_PI/2+th+acos((a*a+r2*r2-b*b)/(2*a*r2));
	joint_state.in.position[2]=-M_PI/2+acos((a*a+b*b-r2*r2)/(2*a*b))-th0;
	
	tf::Quaternion q1,q2,q3,q4,q5;
	//q1.setRPY(0,joint_state.in.position[1]+joint_state.in.position[2],joint_state.in.position[0]);
	tf::quaternionMsgToTF(object_pose.pose.orientation,q1);//q1 in
	q2.setRPY(0,0,joint_state.in.position[0]);
	q3.setRotation( tf::Vector3(1,0,0),joint_state.in.position[1]+joint_state.in.position[2]);
	q2=q2*q3;
	q3.setRotation( tf::Vector3(0,0,1),M_PI/2);
	q2=q2*q3;//q2 
	q4=q2.inverse()*q1;//q4=q2-->q1
	q3=tf::Quaternion(1,0,0,0);
	q3=((q4*q3)*q4.inverse());

	px=q3.x();
	py=q3.y();
	pz=q3.z();
	r1=sqrt(pz*pz+py*py);
	//tf::createQuaternionMsgFromRollPitchYaw(0,0,0); 
	joint_state.in.position[3]=-atan2(py,pz);//-M_PI/2;
	if(joint_state.in.position[3]>M_PI/2)
	{
		joint_state.in.position[3]-=M_PI;
		r1=-r1;
	}
	else if(joint_state.in.position[3]<-M_PI/2)
	{
		joint_state.in.position[3]+=M_PI;
		r1=-r1;
	}
	joint_state.in.position[4]=atan2(r1,px);
	
	q3.setRotation( tf::Vector3(1,0,0),joint_state.in.position[3]);
	q5.setRotation( tf::Vector3(0,-1,0),joint_state.in.position[4]);
	q3=q3*q5;
#ifdef DEBUG
	geometry_msgs::PoseStamped pose_ter;
	pose_ter.header.frame_id=base_frame_id;
	pose_ter.header.stamp= ros::Time::now();
	pose_ter.pose.position=object_pose.pose.position;
	tf::quaternionTFToMsg(q2*q3,pose_ter.pose.orientation);
	pose_pub_ptr->publish(pose_ter);
#endif

	q3=q3.inverse()*q4;
	if(q3.x()>=0)joint_state.in.position[5]=q3.getAngle();
	else joint_state.in.position[5]=-q3.getAngle(); 
	joint_state.desired.position=joint_state.in.position;
	joint_state.desired.velocity=joint_state.in.position;
	
	joint_state.desired.effort[0]=1;
	joint_state.desired.effort[1]=1;
	joint_state.desired.effort[2]=1;
	joint_state.desired.effort[3]=1;
	joint_state.desired.effort[4]=1;
	joint_state.desired.effort[5]=1;


	return true;
}

bool get_PoseDesired(joint_state_controller::Pose_Desired3d::Request  &req,
         joint_state_controller::Pose_Desired3d::Response &res)
{
	res.result=calculate_joints(req.pose);
	return true;
}
void joint_state_puplish_loop(const ros::TimerEvent& event)
{
	joint_state.old.position=joint_state.out.position;
	joint_state.old.velocity=joint_state.out.velocity;
	joint_state.old.effort=joint_state.out.effort;
	
	joint_state.out.position=joint_state.desired.position;
	joint_state.out.velocity=joint_state.desired.velocity;
	joint_state.out.effort=joint_state.desired.effort;

	joint_state.out.header.stamp = ros::Time::now();
	joint_pub_ptr->publish(joint_state.out);
}
int main(int argc, char** argv) 
{
	ros::init(argc, argv, "joint_state_controller");
	ros::NodeHandle n;
	nh=&n;
	tf::TransformListener listener0(ros::Duration(10));
	listener=&listener0; 
	get_Param();
	set_Param();

	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>(publish_topic_name, 3);
	joint_pub_ptr=&joint_pub;
#ifdef DEBUG
	ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/joint_state_controller/debug/terminatio", 3);
	pose_pub_ptr=&pose_pub;
#endif
	ros::ServiceServer service = n.advertiseService(subscript_topic_name, get_PoseDesired);

	ROS_INFO("\npublishing \033[47;31m%d\033[0m joints by topic:\"\033[47;31m%s\033[0m\" at \033[47;31m%.1lf\033[0m Hz\nsubscript topic:\"\033[47;31m%s\033[0m\" ...",joint_setting.num,publish_topic_name.c_str(),publish_frequency,subscript_topic_name.c_str());
		

	//tf::TransformBroadcaster broadcaster;
	
	const double degree = M_PI/180;

	ros::Timer timer1 = n.createTimer(ros::Duration(1.0/publish_frequency), joint_state_puplish_loop);
	ros::spin();

	return 0;
}
