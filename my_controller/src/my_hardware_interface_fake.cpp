#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/differential_transmission.h>
#include <transmission_interface/transmission_interface.h>

#include <vector>

#include <pluginlib/class_list_macros.h>
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>
class MyRobot : public hardware_interface::RobotHW
{
public:
	MyRobot()
	{ 
		// connect and register the joint state interface
		hardware_interface::JointStateHandle state_handle_a_1("joint_arm_a_1", &pos[0], &vel[0], &eff[0]);
		jnt_state_interface.registerHandle(state_handle_a_1);

		hardware_interface::JointStateHandle state_handle_a_2("joint_arm_a_2", &pos[1], &vel[1], &eff[1]);
		jnt_state_interface.registerHandle(state_handle_a_2);

		hardware_interface::JointStateHandle state_handle_a_3("joint_arm_a_3", &pos[2], &vel[2], &eff[2]);
		jnt_state_interface.registerHandle(state_handle_a_3);

		hardware_interface::JointStateHandle state_handle_b_1("joint_arm_b_1", &pos[3], &vel[3], &eff[3]);
		jnt_state_interface.registerHandle(state_handle_b_1);

		hardware_interface::JointStateHandle state_handle_b_2("joint_arm_b_2", &pos[4], &vel[4], &eff[4]);
		jnt_state_interface.registerHandle(state_handle_b_2);

		hardware_interface::JointStateHandle state_handle_b_3("joint_arm_b_3", &pos[5], &vel[5], &eff[5]);
		jnt_state_interface.registerHandle(state_handle_b_3);

		hardware_interface::JointStateHandle state_handle_f_a("joint_finger_a", &pos[6], &vel[6], &eff[6]);
		jnt_state_interface.registerHandle(state_handle_f_a);

		//hardware_interface::JointStateHandle state_handle_f_b("joint_finger_b", &pos[7], &vel[7], &eff[7]);
		//jnt_state_interface.registerHandle(state_handle_f_b);


		registerInterface(&jnt_state_interface);

   		// connect and register the joint position interface
		hardware_interface::JointHandle pos_handle_a_1(jnt_state_interface.getHandle("joint_arm_a_1"), &cmd[0]);
		jnt_pos_interface.registerHandle(pos_handle_a_1);

		hardware_interface::JointHandle pos_handle_a_2(jnt_state_interface.getHandle("joint_arm_a_2"), &cmd[1]);
		jnt_pos_interface.registerHandle(pos_handle_a_2);

		hardware_interface::JointHandle pos_handle_a_3(jnt_state_interface.getHandle("joint_arm_a_3"), &cmd[2]);
		jnt_pos_interface.registerHandle(pos_handle_a_3);

		hardware_interface::JointHandle pos_handle_b_1(jnt_state_interface.getHandle("joint_arm_b_1"), &cmd[3]);
		jnt_pos_interface.registerHandle(pos_handle_b_1);

		hardware_interface::JointHandle pos_handle_b_2(jnt_state_interface.getHandle("joint_arm_b_2"), &cmd[4]);
		jnt_pos_interface.registerHandle(pos_handle_b_2);

		hardware_interface::JointHandle pos_handle_b_3(jnt_state_interface.getHandle("joint_arm_b_3"), &cmd[5]);
		jnt_pos_interface.registerHandle(pos_handle_b_3);

		hardware_interface::JointHandle pos_handle_f_a(jnt_state_interface.getHandle("joint_finger_a"), &cmd[6]);
		jnt_pos_interface.registerHandle(pos_handle_f_a);

		//hardware_interface::JointHandle pos_handle_f_b(jnt_state_interface.getHandle("joint_finger_b"), &cmd[7]);
		//jnt_pos_interface.registerHandle(pos_handle_f_b);

		registerInterface(&jnt_pos_interface);

		


		jointState_pub= n.advertise<sensor_msgs::JointState>("/joint_states", 3); 
		joint_State.header.frame_id="arm_link";
		joint_State.name.resize(7);
		joint_State.position.resize(7);
		joint_State.velocity.resize(7);
		joint_State.effort.resize(7);
		for(int i=0;i<7;i++)
		{
		joint_State.position[i]=0;
		joint_State.velocity[i]=0;
		joint_State.effort[i]=0;
		pos_hw[i]=0;
		}
		joint_State.name[0]="joint_arm_a_1";
		joint_State.name[1]="joint_arm_a_2";
		joint_State.name[2]="joint_arm_a_3";
		joint_State.name[3]="joint_arm_b_1";
		joint_State.name[4]="joint_arm_b_2";
		joint_State.name[5]="joint_arm_b_3";
		joint_State.name[6]="joint_finger_a";
		//joint_State.name[7]="joint_finger_b";
	}
	void read()
	{
		for(int i=0;i<7;i++)
		{
			pos[i]=pos_hw[i];
		}
        	for(int i=0;i<7;i++)
		{
			joint_State.position[i]=pos[i];
		} 
		joint_State.header.stamp=ros::Time::now();
		jointState_pub.publish(joint_State);
	}
	void write()
	{
		for(int i=0;i<7;i++)
		{
			pos_hw[i]=cmd[i];
		}
	}
	void printf()
	{
		for(int i=0;i<7;i++)
		std::cout<<pos[i]<<std::endl;
	}
private:
	hardware_interface::JointStateInterface jnt_state_interface;
	hardware_interface::PositionJointInterface jnt_pos_interface;
	double cmd[7];
	double pos[7];
	double vel[7];
	double eff[7];

	double pos_hw[7];

	ros::NodeHandle n;
	ros::Publisher jointState_pub;
	sensor_msgs::JointState joint_State;
	sensor_msgs::JointState joint_Cmd;

};

controller_manager::ControllerManager * cm_ptr;
void control_loop(const ros::TimerEvent& event)
{
//cm_ptr->update(ros::Time::now(), ros::Duration(0.03));
}

int main(int argc,char ** argv)
{
	ros::init(argc, argv, "my_cm");
	ros::NodeHandle n;
	MyRobot robot;
	controller_manager::ControllerManager cm(&robot);
	cm_ptr=&cm;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	float publish_frequency=30.0;
	ros::Timer timer1 = n.createTimer(ros::Duration(1.0/publish_frequency), control_loop);
	ros::Time pre_time=ros::Time::now();
	ros::Rate rate(50.0);
	cm.loadController("my_arm_controller");
	cm.loadController("my_finger_controller");
	std::vector<std::string> start_controllers;
	start_controllers.push_back("my_arm_controller");

	std::vector<std::string> stop_controllers;
	stop_controllers.push_back("my_finger_controller");
  
  //cm.switchController(start_controllers,stop_controllers,1);




	while (n.ok())
	{
		const ros::Time now =ros::Time::now();
		const ros::Duration period =now-pre_time;
		robot.read();
    // robot.printf();
		cm_ptr->update(now, period);
		robot.write();
     
     //ros::spinOnce();
		rate.sleep();
		pre_time=now;
	}
}
