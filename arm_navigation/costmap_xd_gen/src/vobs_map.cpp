#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <move_arm_msgs/CostMap_xD_Stamped.h>
#include <tf/transform_broadcaster.h>
#include <cstdlib> 


#include <pluginlib/class_loader.h>
#include <costmap_gen_plugins/costmap_gen_base.h>



#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "costmap_gen_test");
	char buf[20];
	double px,py,pz,ox,oy,oz;
	if (argc != 4||
		sscanf(argv[1],"%lf", &px) != 1||
		sscanf(argv[2],"%lf", &py) != 1|| 
		sscanf(argv[3],"%lf", &pz) != 1) 
		{
			fprintf(stderr, "Usage: frameid  pose:px py pz ox oy oz\n");
			exit(1);
		}

	ros::NodeHandle n;
	ros::Publisher costcloud_pub = n.advertise<sensor_msgs::PointCloud>("costcloud", 3);
	move_arm_msgs::CostMap_xD_Stamped costmap1;
	costmap1.costmap.dimension=3;
	costmap1.costmap.size.resize(costmap1.costmap.dimension);
	costmap1.costmap.resolution.resize(costmap1.costmap.dimension);
	costmap1.costmap.size[0]=px;
	costmap1.costmap.size[1]=py;
	costmap1.costmap.size[2]=pz;
	costmap1.costmap.data.resize(costmap1.costmap.size[0]*costmap1.costmap.size[1]*costmap1.costmap.size[2]);

	pluginlib::ClassLoader<costmap_gen_base::BaseClass> poly_loader("costmap_xd_gen", "costmap_gen_base::BaseClass");
	try
	{
		boost::shared_ptr<costmap_gen_base::BaseClass> joint_limit = poly_loader.createInstance("costmap_gen_plugins::Joint_Limit");
		joint_limit->process(costmap1);
		ROS_INFO("Joint_Limit_process OK!");

		boost::shared_ptr<costmap_gen_base::BaseClass> obstacle_limit = poly_loader.createInstance("costmap_gen_plugins::Obstacle_Limit");
		obstacle_limit->process(costmap1);
		ROS_INFO("Joint_Limit_process OK!");
 	}
	catch(pluginlib::PluginlibException& ex)
	{
		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
	}


	sensor_msgs::PointCloud cloud4view;
	cloud4view.header.frame_id="map";
	
	//cloud4view.points.resize(costmap1.costmap.data.size());
	cloud4view.channels.resize(1);
	cloud4view.channels[0].name="cost";	
	//cloud4view.channels[0].values.resize(costmap1.costmap.data.size());	
	int ii=0;
	for(int i=-pz/2;i<pz/2;i++)
	{
		for(int j=-py/2;j<py/2;j++)
		{
			for(int k=-px/2;k<px/2;k++)
			{
				double r , th,ba,r0,r1,r2;
				r1=sqrt(k*k+j*j+i*i);
				r=sqrt(k*k+j*j);
				th=atan2(i,r);
				ba=atan2(k,j)+M_PI/2;
		
				if(th>0)
				{
					r0=0;
					r2=0;
				}
				else
				{
					r0=pz/200*60*cos(th);
					r2=pz/200*30*cos(th);
				}
				double err=r1-r0;
				double err2=r1-r2;
				if(err<=0)err=-err;
				if(err2<=0)err2=-err2;
				if(ba<0.22&&ba>-0.22&&(err<=2.4||err2<=1.9))
				{
					geometry_msgs::Point32 point;
					point.x=k;
					point.y=j;
					point.z=i;
					cloud4view.points.push_back(point);
					cloud4view.channels[0].values.push_back(1);
					ii++;	
				}					
			}	
		}
	}
	ROS_INFO("get cloud!");
	ros::Rate r(1.0);	
	while(n.ok())
	{
		cloud4view.header.stamp= ros::Time::now();
		costcloud_pub.publish(cloud4view);
	 // pose_pub.publish(pose.request.pose);
		r.sleep();
	}
	return 0;
}
