//寻线飞行头文件;两个类：巡线方法类，无人机类;
//开发者：张涛/重庆大学自动化学院智慧工程研究院无人机项目组
//声明文件(函数和变量的声明可以包含在头文件中，但定义只能在一个CPP文件中)

//常规包含
#include<iostream>  
#include<stdlib.h>  
#include<math.h>  
#include<syslog.h> 
#include <fstream>
#include<fcntl.h>  
#include<unistd.h>  
#include<sys/stat.h>  
#include<linux/input.h> //linux/input.h->linux/input-event-codes.h和opencv2/opencv.hpp宏定义冲突,需要更改
#include <signal.h>

//ROS相关包含
#include<ros/ros.h>  //+
#include<image_transport/image_transport.h>//+
#include<sensor_msgs/image_encodings.h> //+
#include<cv_bridge/cv_bridge.h>//+
#include<sensor_msgs/Image.h>//+
#include<std_msgs/Empty.h>//+
#include<geometry_msgs/Twist.h>//+
#include<ardrone_autonomy/Navdata.h>  // bebop2???

//OPENCV包含
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

//宏定义
#define uchar unsigned char
#define PI 3.14


using namespace std;
using namespace cv;

//误差结构体
struct errors
{
	double angle; //箭头弧度
	double dist;//箭头距离
	double e_height;//高度误差
	double true_angle;//真实角度
};

//无人机传回数据
struct droneData
{
	double altitude_;//无人机高度
	double battery_;//电量
	Mat imageRow_; //无人机传回原始图像
};

//声明程序全局变量
extern struct droneData mydroneData;  //无人机传回数据

//导航数据获取函数
void navigationData(const ardrone_autonomy::NavdataConstPtr & nav); 

//图像数据获取函数
void imageData(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& infomsg);


//寻线方法类
class PatrolFlight
{
	private:
		//自适应参数——定高
		double u_z;
		double hat_w_z;
		double hat_w_sec_z;

		//自适应参数——偏航
		double u_yaw;
		double hat_w_yaw;
		double hat_w_sec_yaw;

		//自适应参数——侧移
		double u_dist;
		double hat_w_dist;
		double hat_w_sec_dist;
		
		//检测第一行和两边,以获取目标点的坐标
		int iaimr1_l, iaimr1_r;//左右两边目标点
		int iaimr1;//第一行中间实际目标点
		int iaimr10_l, iaimr10_r;//十分之一行处左右两边的目标点
		int iaimr10;//十分之一行处中间实际目标点
		bool getaimr1;//第一行检测标志
		bool getaimr10;//第十行检测标志
		bool getaimr;//行检测标志

		int iaimc1_u, iaimc1_d;//第一列上下两边目标点
		int iaimc1;//第一列实际目标点
		int iaimcn_u, iaimcn_d;//最后一列上下两边目标点
		int iaimcn;//最后一列的实际目标点
		int iaimc10_u, iaimc10_d;//十分之一列上下两边目标点
		int iaimc10;//十分之一列的实际目标点
		int iaimc_10_u, iaimc_10_d;//后十分之一列上下两边目标点
		int iaimc_10;//十分之一列的实际目标点
		bool getaimc1;//第一列检测标志
		bool getaimcn;//最后一列检测标志
		bool getaimcn_10;//右列检测标志
		bool getaimc1_10;//左列检测标志
		bool getaimc10;//十分之一列检测标志
		bool getaimc_10;//十分之一列检测标志
		bool getaimc;//列检测标志
		
		//二值化参数
		int ilowh;
		int ihighh;
		int ilows;
		int ihighs;
		int ilowv;
		int ihighv;
		
		double line_a;  //扫描线宽度
		double line_b;//真实线宽度
		double pixel_dist;//每个像素点的物理长度
		bool fly_s;//飞行态和调整态
		bool sta_s;//稳定态
		int ss;//
		int rr;//
		
	public:	
		int icolst, irowst, imidtc, imidtr;//图像大小
		struct errors my_error;//初始化误差结构体
		float flight_angle;//飞行合理角度
		float flight_dist;//飞行合理距离
		int times;//未检测到线的次数
		Mat image_row; //原始图
		Mat throed; //二值图
		Mat image_jz; //矫正图
		Mat image_cut;   //剪切图
		Mat image_rotate; //旋转图
		bool call_vf; //回调函数标志	
		geometry_msgs::Twist speedpid;//速度
	
	public:
		PatrolFlight();//构造函数
		~PatrolFlight();//虚构函数
		
		Mat imgCorrection(const Mat img); //图像畸变矫正函数
		void imgRotate(Mat& img, Mat& newIm, double angle); //图片旋转函数
		Mat imgCut(const Mat image);//图像剪切函数
		void dataTotext(const struct errors errors_);//把数据写入文件
		double adaptiveHeight(double eheight);//自适应定高
		double adaptiveYaw(double error_yaw);//自适应调整偏航角
		double adaptiveDist(double error_dist);//自适应侧移
		void pathDetection(const Mat& image_row);//路径检测函数
		geometry_msgs::Twist Adaptive();//速度配置函数，正常摄像头
		geometry_msgs::Twist AdaptiveUnnormal();//速度配置函数,畸变摄像头
		void parameterInitialization();//参数再次初始化
		
		void ChangeControlParameters_h(double u_z_,double hat_w_z_,double hat_w_sec_z_);//更改自适应参数——定高
		void ChangeControlParameters_z(double u_yaw_,double hat_w_yaw_,double hat_w_sec_yaw_);//更改自适应参数——偏航
		void ChangeControlParameters_y(double u_dist_,double hat_w_dist_,double hat_w_sec_dist_);//更改自适应参数—侧移
		void SetFlightThreshold(float flight_angle_,float flight_dist_);//设定飞行状态阈值
		void SetLineColor(int ilowh_,int ihighh_,int ilows_,int ihighs_,int ilowv_,int ihighv_);//设定检测线的颜色

};


//无人机类
class Drone
{
	public:
		ros::NodeHandle drone;   //节点句柄
	 	image_transport::ImageTransport image_transport;//视频发布类节点
	 	image_transport::CameraSubscriber image_sub;//视频话题接收者 	
	 	ros::Subscriber rh_navdata_sub;//导航话题接收者
	 	ros::Publisher move_pub; //无人机移动话题发布者
		ros::Publisher camera_move; //相机移动话题发布者
		ros::Publisher land_pub;   //着陆话题发布者
		ros::Publisher takeoff_pub; //起飞话题发布者
		
		geometry_msgs::Twist speedadjust;//无人机速度
		geometry_msgs::Twist cameraadjust;//相机调整
		
		double altitude;//无人机高度
		double target_height;  //无人机目标高度
		double error_height;//高度误差
		double battery;//电量
		std_msgs::Empty empty_land;
		std_msgs::Empty empty_off;
		
		bool FixedFlight;//是否定高飞行
		bool call_rh; 
		bool start_line;//开始巡线标志
		
		Mat imageRow; //无人机传回原始图像
		
	public:
		Drone(); //构造函数
		~Drone(); //虚构函数
	
		void parameterInitialization();//参数初始化
		void takeoff();//起飞
		void land();//降落
		void circle();//转圈
		void stable();//无人机稳定	
};

//巡线函数
void patrolFlight_main(Drone& myDrone,PatrolFlight& myPatrolFlight);

//紧急调用的降落函数
void land_();

//覆盖原来的Ctrl+C中断函数，原来的只会调用ros::shutdown()
void MySigintHandler(int sig);
