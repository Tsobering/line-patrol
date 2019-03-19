#include<iostream>  
#include<stdlib.h>  
#include<math.h>  

#include<ros/ros.h>  //+
#include<image_transport/image_transport.h>//+
#include<sensor_msgs/image_encodings.h> //+
#include<cv_bridge/cv_bridge.h>//+
#include<sensor_msgs/Image.h>//+
#include<std_msgs/Empty.h>//+
#include<geometry_msgs/Twist.h>//+
#include<ardrone_autonomy/Navdata.h>


using namespace std;
using namespace cv;

bool turn_s=false;

void takeoff_()
{	
	ros::NodeHandle video_n;   //节点句柄
	ros::Publisher takeoff_pub; //起飞话题发布者
	takeoff_pub=video_n.advertise<std_msgs::Empty>("bebop/takeoff", 1);
	
	ros::Rate loop_rate(10);
	std_msgs::Empty empty_off;
	
	int second=0;
	while(ros::ok())
	{
		//at last four time can be fly steady
		if(second<10)
		{	//发布起飞话题，开始起飞
			takeoff_pub.publish(empty_off);
			cout<<"pub takeoff：无人机起飞"<<endl;
		}	
		else
			break;
	    loop_rate.sleep();
		++second;		
	}
	
}

void land_()
{
	ros::NodeHandle video_n;   //节点句柄
	 ros::Publisher land_pub;   //着陆话题发布者
	 //初始化 着陆话题发布者
   	land_pub=video_n.advertise<std_msgs::Empty>("bebop/land",1);
	
	ros::Rate loop_rate(10);
	//time
	int second=0;
	std_msgs::Empty empty_land;
	
	while(ros::ok())
	{
		//only one time can be landed
		if(second<5)
		{
			//发送着陆指令
			land_pub.publish(empty_land);
			cout<<"pub land：无人机开始降落"<<endl;
		}
		else
			break;
		 
		++second;
	    loop_rate.sleep();
		
	}
	
}

geometry_msgs::Twist FLY()
{
	
	geometry_msgs::Twist speedpid;//速度
	if(turn_s)
	{
		speedpid.linear.x =0;
		speedpid.linear.y =0;
		speedpid.linear.z =0;
		speedpid.angular.z =0.2;
		cout<<"稳定无人机准备转弯"<<endl;
	}
	else if (!turn_s)//如果在飞行态
	{
		speedpid.linear.x =0.2;
		speedpid.linear.y =0;
		speedpid.linear.z =0;
		speedpid.angular.z =0;
		cout<<"无人机正在巡航"<<endl;
	}
		
	
	
	cout << "线速度 x= ：" << speedpid.linear.x << endl;
	cout << "线速度 y= ：" << speedpid.linear.y << endl;
	cout << "角速度 ：" << speedpid.angular.z << endl;

	return speedpid;
	
}
void advertop(int scds){
	ros::NodeHandle video_cv;   //节点句柄
	ros::Publisher cmd_vel_pub; //移动话题发布者
	cmd_vel_pub = video_cv.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);
	
	ros::Rate loop_rate(10);
	//time
	int i=0;
	geometry_msgs::Twist speeds;
	
	while(ros::ok())
	{
		speeds=FLY();
		//only one time can be landed
		if(i<scds)
		{
			//发送着陆指令
			cmd_vel_pub.publish(speeds);
			++i;
		}
		else	++i;
			break;
		 
		
	    loop_rate.sleep();
		
	}
	
}

int main(int argc,char **argv){
	ros::init(argc,argv,"cvideo");
	cout<<"ros init finshed"<<endl;
	takeoff_();
	sleep(5);
	advertop(50);
	//sleep(5);
	turn_s=true;
	advertop(10);
	turn_s=false;
	//sleep(5);
	//advertop(20);
	//sleep(5);
	land_();

}
