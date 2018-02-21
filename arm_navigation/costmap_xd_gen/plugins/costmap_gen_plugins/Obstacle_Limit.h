#ifndef COSTMAP_GEN_JOINTLIM_H_
#define COSTMAP_GEN_JOINTLIM_H_
#include <costmap_gen_plugins/costmap_gen_base.h>
#include <move_arm_msgs/CostMap_xD_Stamped.h>
#include <cmath>

namespace costmap_gen_plugins
{
  class Obstacle_Limit :public costmap_gen_base::BaseClass
  {
    public:
	void initialize();
 	int process(move_arm_msgs::CostMap_xD_Stamped & data);
  };
};
#endif
