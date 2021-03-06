#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <move_arm_msgs/CostMap_xD_Stamped.h>
#include <tf/transform_broadcaster.h>
#include <cstdlib> 


#include <pluginlib/class_loader.h>
#include <costmap_gen_plugins/costmap_gen_base.h>



#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "costmap_gen_test");
	char buf[20];
	double px,py,pz,ox,oy,oz;
/*	if (argc != 8||
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
		}*/

	ros::NodeHandle n;
	ros::Publisher costcloud_pub = n.advertise<sensor_msgs::PointCloud>("costcloud", 3);
	move_arm_msgs::CostMap_xD_Stamped costmap1;
	costmap1.costmap.dimension=3;
	costmap1.costmap.size.resize(costmap1.costmap.dimension);
	costmap1.costmap.resolution.resize(costmap1.costmap.dimension);
	costmap1.costmap.size[0]=40;
	costmap1.costmap.size[1]=40;
	costmap1.costmap.size[2]=40;
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
	
	cloud4view.points.resize(costmap1.costmap.data.size());
	cloud4view.channels.resize(1);
	cloud4view.channels[0].name="cost";	
	cloud4view.channels[0].values.resize(costmap1.costmap.data.size());	
	int ii=0;
	for(int i=0;i<costmap1.costmap.size[2];i++)
	{
		for(int j=0;j<costmap1.costmap.size[1];j++)
		{
			for(int k=0;k<costmap1.costmap.size[0];k++)
			{
				cloud4view.points[ii].x=k-20;
				cloud4view.points[ii].y=j-20;
				cloud4view.points[ii].z=i-20;
				cloud4view.channels[0].values[ii]=(ii%costmap1.costmap.data.size())/(1.0*costmap1.costmap.data.size());
				ii++;	
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
