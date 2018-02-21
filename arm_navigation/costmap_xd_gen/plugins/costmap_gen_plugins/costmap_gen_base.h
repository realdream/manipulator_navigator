#ifndef COSTMAP_GEN_BASE_H_
#define COSTMAP_GEN_BASE_H_
#include <move_arm_msgs/CostMap_xD_Stamped.h>
namespace costmap_gen_base
{
  class BaseClass
  {
    public:
      virtual void initialize() =0;
      virtual int process(move_arm_msgs::CostMap_xD_Stamped & data) = 0;
      virtual ~BaseClass(){}

    protected:
      BaseClass(){}
  };
};
#endif

