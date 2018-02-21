#include <pluginlib/class_list_macros.h>
#include <costmap_gen_plugins/costmap_gen_base.h>
#include <costmap_gen_plugins/Obstacle_Limit.h>
#include <fstream>
namespace costmap_gen_plugins
{
	void Obstacle_Limit::initialize(){return;}
 	int Obstacle_Limit::process(move_arm_msgs::CostMap_xD_Stamped & data)
	{
		return 0;
	}
};
PLUGINLIB_EXPORT_CLASS(costmap_gen_plugins::Obstacle_Limit,costmap_gen_base::BaseClass)

