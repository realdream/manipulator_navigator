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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <termios.h>

#include <string.h>
#include <errno.h>


#define UART_DATA_PACKET_INT             13
#define UART_DATA_PACKET_CHAR            UART_DATA_PACKET_INT*2
#define UART_DATA_PACKET                 UART_DATA_PACKET_CHAR
#define SERVOSETTINGS_CH_NUMELEM		 9
union UART_Buff
{
  char  Data_char[UART_DATA_PACKET_CHAR];
  short int  Data_int[UART_DATA_PACKET_INT];
};
typedef union UART_Buff UART_Buff;
static union UART_Buff UART_Buff_In;//pc->mcu:0xaacc:pose:veli:0xccaa
static union UART_Buff UART_Buff_Out;//pc->mcu:0xaacc:pose:veli:0xccaa
static unsigned char UART_P_R=0;
static unsigned char UART_P_W=0;
char UART_FIFO[256];
static int fd;

struct ServoData
{
  short int pos;
  short int vel;
  short int eff;
  short int cmd;
};
typedef struct ServoData ServoData;
static ServoData servoData[SERVOSETTINGS_CH_NUMELEM];

static char buf;
static char issync=0;
static char data_cont=0;
static char chk=0;
static char check[5]="_CMD";
static unsigned short int sync_count=0;
static unsigned short int lose_sync=25;
void getstate()
{
 for(;;)
    { 
          if(UART_P_R!=UART_P_W)
          {
            buf=UART_FIFO[(UART_P_R)];
            UART_P_R++;
            if(issync==1)//同步成功
            {
                sync_count=lose_sync;
                UART_Buff_In.Data_char[data_cont]=buf;
                data_cont++;
              if(data_cont>UART_DATA_PACKET-1-4)
              {
                issync=0;
                data_cont=0;
                chk=0;
                unsigned char k;
                for(k=0;k<SERVOSETTINGS_CH_NUMELEM;k++)
                servoData[k].pos=UART_Buff_In.Data_int[k];
              }
            }
            else//issync==0
            {
              if(check[chk]==buf)
              {
                chk++;
                if(chk>3)
                {
                  issync=1;
                }
              }
              else
              {
                if(buf==check[0])
                {
                  chk=1;
                }
                else
                {
                  issync=0;
                  data_cont=0;
                  chk=0;
                }
              }
            }
          }
          else
            break;
        }
}
void send_cmd()
{
 	short int buf[20]={0x435f,0x444d};
	for(int i=0 ; i<10;i++)
	buf[i+2]=servoData[i].cmd;
		//buf[2]=swip16(pit);
		//buf[3]=swip16(yaw);
		//ioctl(fd, on, led_no);
		if (write(fd,buf,26) < 0)
		{
			perror("write to device hidraw:");
			if(close(fd)<0)
			{
				perror("close device hidraw:");
				exit(1);
			}
			exit(1);
		}
}

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
	void arm_read()
	{
		for(int i=0;i<7;i++)
		{
			pos[i]=servoData[i].pos/10000.0;
		}
        for(int i=0;i<7;i++)
		{
			joint_State.position[i]=pos[i];
		} 
		joint_State.header.stamp=ros::Time::now();
		jointState_pub.publish(joint_State);
	}
	void arm_write(int force_send)
	{
		UART_Buff_Out.Data_char[0]='_';
		UART_Buff_Out.Data_char[1]='C';
		UART_Buff_Out.Data_char[2]='M';
		UART_Buff_Out.Data_char[3]='D';
		int flag=0;
		for(int i=0;i<7;i++)
		{
			if(UART_Buff_Out.Data_int[i+2]!=(short int)(10000*cmd[i]))flag++;
			
			UART_Buff_Out.Data_int[i+2]=10000*cmd[i];	
		}
		if(flag!=0||force_send)
		if (::write(fd,UART_Buff_Out.Data_char,26) < 0)
		{
			perror("write to device hidraw:");
			//if(close(fd)<0)
			//{
			//	perror("close device hidraw:");
			//	exit(1);
			//}
			exit(1);
		}
	}
	void printf()
	{
		for(int i=0;i<8;i++)
		std::cout<<pos[i]<<std::endl;
	}
	double pos_hw[8];
private:
	hardware_interface::JointStateInterface jnt_state_interface;
	hardware_interface::PositionJointInterface jnt_pos_interface;
	double cmd[8];
	double pos[8];
	double vel[8];
	double eff[8];



	ros::NodeHandle n;
	ros::Publisher jointState_pub;
	sensor_msgs::JointState joint_State;
	sensor_msgs::JointState joint_Cmd;


};


controller_manager::ControllerManager * cm_ptr;


int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
/* 五个参量 fd打开文件 speed设置波特率 bit数据位设置   neent奇偶校验位 stop停止位 */
    struct termios newtio,oldtio;
    if ( tcgetattr( fd,&oldtio) != 0) { 
        perror("SetupSerial 1");
        return -1;
    }
    bzero( &newtio, sizeof( newtio ) );
    newtio.c_cflag |= CLOCAL | CREAD; 
    newtio.c_cflag &= ~CSIZE; 
    switch( nBits )
    {
    case 7:
        newtio.c_cflag |= CS7;
    break;
    case 8:
        newtio.c_cflag |= CS8;
    break;
    }
    switch( nEvent )
    {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E': 
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N': 
        newtio.c_cflag &= ~PARENB;
        break;
    }
switch( nSpeed )
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
         cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if( nStop == 1 )
        newtio.c_cflag &= ~CSTOPB;
    else if ( nStop == 2 )
    newtio.c_cflag |= CSTOPB;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd,TCIFLUSH);
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
}
void UART_read_loop(const ros::TimerEvent& event)
{
	//UART_Buff_In
	  int nread=0;
       while ((nread = read(fd, UART_FIFO+UART_P_W, 1))>0)
       {
           //printf("\nLen %d\n", nread);
           UART_P_W+=nread;
           //printf("\n%s", buff);
       }
		getstate();
	
}
void UART_read()
{
	//UART_Buff_In
	  int nread=0;
       while ((nread = read(fd, UART_FIFO+UART_P_W, 1))>0)
       {
           //printf("\nLen %d\n", nread);
           UART_P_W+=nread;
           //printf("\n%s", buff);
       }
		getstate();
	
}

int main(int argc,char ** argv)
{
	ros::init(argc, argv, "my_cm");
	ros::NodeHandle n;
	int dev=0;
	char dir[128]="";
	n.getParam("USB_dev", dev);
	sprintf(dir,"/dev/ttyUSB%d",dev);
	fd = open(dir,O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd < 0) {
		perror("open device:");
		exit(1);
	}
        if(set_opt(fd,115200,8,'N',1)<0)
	{
		perror("set_opt error");
		exit(1);
	}

	MyRobot robot;
	controller_manager::ControllerManager cm(&robot);
	cm_ptr=&cm;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	float publish_frequency=30.0;
	//ros::Timer timer1 = n.createTimer(ros::Duration(1.0/50.0), UART_read_loop);
	ros::Time pre_time=ros::Time::now();
	ros::Rate rate(100.0);
	cm.loadController("my_arm_controller");
	cm.loadController("my_finger_controller");
	std::vector<std::string> start_controllers;
	start_controllers.push_back("my_arm_controller");

	std::vector<std::string> stop_controllers;
	stop_controllers.push_back("my_finger_controller");
  
  //cm.switchController(start_controllers,stop_controllers,1);


    int count=0;

	while (n.ok())
	{
		count++;
		const ros::Time now =ros::Time::now();
		const ros::Duration period =now-pre_time;
		UART_read();
		robot.arm_read();
    // robot.printf();
		cm_ptr->update(now, period);
		robot.arm_write(count%256==1);
     
     //ros::spinOnce();
		rate.sleep();
		pre_time=now;
	}
}
