//巡线飞行主函数
//开发者：曹枭/重庆大学自动化学院智慧工程研究院无人机项目组

#include<bebop2_flight.h>

using namespace std;
using namespace cv;


//主函数
int main(int argc,char **argv)
{
	//初始化ROS
	ros::init(argc,argv,"PatrolFlight");
	cout<<"ros init finshed"<<endl;
	
	//创建无人机实例
	Drone myDrone_;
	//创建巡线方法实例
	PatrolFlight myPatrolFlight_;
	//进入巡线
	patrolFlight_main(myDrone_,myPatrolFlight_);
	
	return 0;
}