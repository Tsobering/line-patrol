//常规包含
#include<iostream>  
#include<stdlib.h>  
#include<math.h>  

//linux/input.h->linux/input-event-codes.h和opencv2/opencv.hpp宏定义冲突
#include<linux/input.h>  
#include<fcntl.h>  
#include<unistd.h>  
#include <sys/stat.h>  

//ROS相关包含
#include<ros/ros.h>  //+
#include<image_transport/image_transport.h>//+
#include<sensor_msgs/image_encodings.h> //+
#include<cv_bridge/cv_bridge.h>//+
#include<sensor_msgs/Image.h>//+
#include<std_msgs/Empty.h>//+
#include<geometry_msgs/Twist.h>//+
#include<ardrone_autonomy/Navdata.h>

//OPENCV包含
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

#include <vector>
#include <Eigen/Core>
#include <Eigen/Eigen>


#define uchar unsigned char
#define PI 3.14

using namespace std;
using namespace cv;

//回调函数标志
bool call_rh = true;
bool call_vf = true;

float angle = 0; //箭头弧度
float dist = 0;//箭头距离
double e_height = 0.0;

bool fly_s = false;//飞行态和调整态
bool sta_s=true;//稳定态
int ss=0;
int rr=0;
//开始巡线标志
bool start_line=false;

//飞行合理角度
float flight_angle=0.3;
float flight_dist=190;
//创建两个窗口
//cv::namedWindow("底部相机原始图像");
//cv::namedWindow("二值化图像");

int times=0;

Mat image_row; //原始图
Mat throed; //二值图



//自适应参数——定高
double u_z = 0.0;
double hat_w_z = 0.0;
double hat_w_sec_z = 0.0;

//自适应参数——偏航
double u_yaw = 0.0;
double hat_w_yaw = 0.0;
double hat_w_sec_yaw = 0.0;

//自适应参数——侧移
double u_dist = 0.0;
double hat_w_dist = 0.0;
double hat_w_sec_dist = 0.0;

double NeroNetworkUncertainty(double target_x, double target_y, double target_z);
double target_x_prior = 0.0;
double target_y_prior = 0.0;
double target_z_prior = 0.0;

double F_norm = 0.0;



//pid类
/*class pid
{
	private:
		float Setvalue;            //定义设定值
		float Actualvalue;        //定义实际值
		float err;                //定义偏差值
		float err_last;            //定义上一个偏差值
		float Kp,Ki,Kd;            //定义比例、积分、微分系数
		float speed;                //定义速度（控制执行器的变量）
		float integral;            //定义积分值
  		float critical_point;     //临界点
	public:
		pid(float kp_)
		{
			Setvalue=0.0;
			Actualvalue=0.0;
			err=0.0;
			err_last=0.0;
			speed=0.0;
			integral=0.0;
			Kp=kp_;
			Ki=0.0;
			Kd=0.0;
			critical_point=0.0;
		}
	
		void set_kp(float kp)
		{
			Kp=kp;
		}
		void set_ki(float ki)
		{
			Ki=ki;
		}
		void set_kd(float kd)
		{
			Kd=kd;
		}
		void set_value(float setvalue)
		{
			Setvalue=setvalue;
		}
		void set_critical_point(float cri_pit)
		{
			critical_point=cri_pit;
		}
		float get_kp()
		{
			return Kp;
		}
	
		float PID_realize(float error)
		{
			    err=error;
			//	if(fabs(error)<critical_point)
			//	{
			//		integral=0.0;
			//	}
			//	else
			//		integral+=err;
				speed=Kp*err;//+Ki*integral+Kd*(err-err_last);
				//err_last=err;
			  //  pid.Actualvalue=pid.speed*1.0;
				return speed;
		}
	
};*/


//pid pid_dist(0.00015);
//std::cout<<"dist kp"<<pid_dist.get_kp()<<std::endl;
//pid_dist.set_kp(0.0015);
//pid pid_angle(0.636);
//pid_angle.set_kp(0.636);



//起飞
void takeoff_()
{	
	ros::NodeHandle video_n;   //节点句柄
	ros::Publisher takeoff_pub; //起飞话题发布者
	takeoff_pub=video_n.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
	
	ros::Rate loop_rate(10);
	std_msgs::Empty empty_off;
	
	int second=0;
	while(ros::ok())
	{
		//at last four time can be fly steady
		if(second<5)
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



//降落
void land_()
{
	ros::NodeHandle video_n;   //节点句柄
	 ros::Publisher land_pub;   //着陆话题发布者
	 //初始化 着陆话题发布者
   	land_pub=video_n.advertise<std_msgs::Empty>("ardrone/land",1);
	
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


//定高回调函数
/*void callback(const ardrone_autonomy::NavdataConstPtr & nav)  
{
		ros::NodeHandle video_n;   //节点句柄
		//移动话题发布者
		ros::Publisher rh_cmd_vel_pub; 
		//初始化 移动话题发布者
		rh_cmd_vel_pub=video_n.advertise<geometry_msgs::Twist>("cmd_vel",1);

		float alt=nav->altd;
		cout<<"无人机高度为： "<<alt<<endl;
		//ROS_INFO("the ardrone altitude is %f",alt);

		//ros::Rate loop_rate(10);

		//time
		//int second=0;
		//double s=0.0005;
		//高度低于
		if(alt<1970)
		{
			geometry_msgs::Twist speed;
			speed.linear.x=0;
			speed.linear.y=0;
			//float z=1.0-(alt*s);
			speed.linear.z=1.0;
			speed.angular.x=0;
			speed.angular.y=0;
			speed.angular.z=0;
			rh_cmd_vel_pub.publish(speed);  
			cout<<"rising：上升"<<endl;
		}
		//
		else if(alt>2030)
		{	
			geometry_msgs::Twist speed;
			speed.linear.x=0;
			speed.linear.y=0;
			speed.linear.z=-0.5;
			speed.angular.x=0;
			speed.angular.y=0;
			speed.angular.z=0;
	  		rh_cmd_vel_pub.publish(speed);  
			cout<<"droping：下降"<<endl;
		}
		else
		{
			call_rh=false;
			cout << "到达高度" << endl;
			//goto over;
		}
	    //loop_rate.sleep();
		//++second;
		
	
}*/




//自适应定高

double adaptive(double eheight)
{
	
	geometry_msgs::Twist speed;
	
	double k_z = 0.5;
	double pho_z = 0.001;
	double delta_z = 0.001;
	
	double e_z = eheight;
	
	double neroHeight = NeroNetworkUncertainty(e_z,0,0);
	
	double u_1 = -k_z*e_z-pho_z*e_z*hat_w_z*neroHeight;
	
	hat_w_sec_z = hat_w_z +(pho_z*neroHeight*e_z*e_z-delta*hat_w_z)*0.001;
	hat_w_z = hat_w_sec;
	
	if(u_1>=0.15)
	{
		u_z = 0.15;
	}
	else if(u_1<=-0.15)
	{
		u_z = -0.15;
	}
	else
	{
		u_z = u_1;
	}
	
	/*speed.linear.x=0;
	speed.linear.y=0;
	speed.linear.z = u;
	speed.angular.x=0;
	speed.angular.y=0;
	speed.angular.z=0;*/
	
	//cout<<"rising：控制输入"<< speed.linear.z<<endl;
	
	return u_z;
}


//自适应调整偏航角
double adaptiveYaw(double error_yaw)
{
	
	geometry_msgs::Twist yawspeed;
	
	double k_yaw = 0.7;
	double pho_yaw = 0.001;
	double delta_yaw = 0.001;
	
	double e_yaw = -error_yaw;
	
	double neroYaw = NeroNetworkUncertainty(0,e_yaw,0);
	
	double u_2 = -k_yaw*e_yaw-pho_yaw*e_yaw*hat_w_yaw*neroYaw;
	
	hat_w_sec_yaw = hat_w_yaw +(pho_yaw*neroYaw*e_yaw*e_yaw-delta_yaw*hat_w_yaw)*0.01;
	hat_w_yaw = hat_w_sec_yaw;
	
	if(u_2>=0.9)
	{
		u_yaw = 0.9;
	}
	else if(u_2<=-0.9)
	{
		u_yaw = -0.9;
	}
	else
	{
		u_yaw = u_2;
	}
	
	yawspeed.linear.x=0;
	yawspeed.linear.y=0;
	yawspeed.linear.z = 0;
	yawspeed.angular.x=0;
	yawspeed.angular.y=0;
	yawspeed.angular.z=u_yaw;
	
	//cout<<"转向控制输入angular.z"<< yawspeed.angular.z<<endl;
	
	return u_yaw;
}



//自适应侧移
double adaptiveDist(double error_dist)
{
	
	geometry_msgs::Twist distspeed;
	
	double k_dist = 0.005;
	double pho_dist = 0.001;
	double delta_dist = 0.001;
	
	double e_dist = -error_dist;
	double neroDist = NeroNetworkUncertainty(0,0,e_dist);
	
	double u_3 = -k_dist*e_dist-pho_dist*e_dist*hat_w_dist*neroDist;
	
	hat_w_sec_dist = hat_w_dist + (pho_dist*e_dist*e_dist*neroDist-delta_dist*hat_w_dist)*0.05;
	hat_w_dist = hat_w_sec_dist;
	
	if(u_3>=0.005)
	{
		u_dist = 0.005;
	}
	else if(u_3<=-0.005)
	{
		u_dist = -0.005;
	}
	else
	{
		u_dist = u_3;
	}

        u_dist = u_3;
	
	distspeed.linear.x=0;
	distspeed.linear.y=u_dist;
	distspeed.linear.z = 0;
	distspeed.angular.x=0;
	distspeed.angular.y=0;
	distspeed.angular.z=0;
	
	//cout<<"转向控制输入angular.z"<< distspeed.angular.z<<endl;
	
	return u_dist;
}


double NeroNetworkUncertainty(double target_x, double target_y, double target_z)
{
     
    int i = 0;
    int j = 0;

    double array[5] = {0};
    double b[5] = {3,3,3,3,3};

    Eigen::VectorXf  input(3), del(3),sum_vector(5);

    //Eigen::VectorXf c1(3),c2,c3,c4,c5;

    Eigen::MatrixXf c(3,5);
    c << -3,-2,0,2,3,
         -3,-2,0,2,3,
         -3,-2,0,2,3;
    
    double dot_target_x = (target_x - target_x_prior)/step;
    target_x_prior = target_x;
    
    double dot_target_y = (target_y - target_y_prior)/step;
    target_y_prior = target_y;

    double dot_target_z = (target_z - target_z_prior)/step;
    target_z_prior = target_z;

    input(0) = dot_target_x;
    input(1) = dot_target_y;
    input(2) = dot_target_z;

    std::cout<<input<<endl;

    //input << dot_target_x,dot_target_y,dot_target_z;

    while(i<5)
    {
      while( j < 3)
      {
         del(j) = input(j) - c(j,i);

         j++;
      }
      
      //del << 1,2,3;

      array[i] = exp(-del.norm()*del.norm()/(2*b[i]*b[i]));

      i++;
    }

    sum_vector << array[0],array[1],array[2],array[3],array[4];

    F_norm = sum_vector.norm();
    
}


void callback(const ardrone_autonomy::NavdataConstPtr & nav)  
{
		ros::NodeHandle n;   //节点句柄
		//移动话题发布者
		ros::Publisher cmd_vel_pub_ada; 
		//初始化 移动话题发布者
		cmd_vel_pub_ada = n.advertise<geometry_msgs::Twist>("cmd_vel",1);

		float alt = nav->altd;
		cout<<"无人机高度为： "<<alt<<endl;
		//ROS_INFO("the ardrone altitude is %f",alt);

		//ros::Rate loop_rate(10);
	
	        double height= alt/1000;

	        double target_height = 1.500;
            e_height = height-target_height;
		
			//geometry_msgs::Twist speedadaptive;
	
	        //speedadaptive = adaptive(target_height,height);
			//cmd_vel_pub_ada.publish(speedadaptive); 
	
	        //cout<<"rising：控制输入"<< speedadaptive.linear.z<<endl;
	        cout<<"rising：height(m)"<< height <<endl;
			cout<<"rising：自适应定高测试"<<endl;
	
	        //ros::Duration(0.02).sleep();
	        //speedadaptive.linear.z = 0.0;
	        //cmd_vel_pub_ada.publish(speedadaptive);
		
		//
		/*if(alt>1500)
		{	
			geometry_msgs::Twist speed;
			speed.linear.x=0;
			speed.linear.y=0;
			speed.linear.z=-0.5;
			speed.angular.x=0;
			speed.angular.y=0;
			speed.angular.z=0;
	  		cmd_vel_pub_ada.publish(speed);  
			cout<<"droping：下降"<<endl;
		}*/
		/*else
		{
			call_rh=false;
			cout << "到达高度" << endl;
			//goto over;
		}*/
	    //loop_rate.sleep();
		//++second;
}



//上升并定高
int Rising_heighted_()
{
		ros::NodeHandle video_n;   //节点句柄
		//ros::Publisher rh_cmd_vel_pub; //移动话题发布者	
		ros::Subscriber rh_navdata_sub;//导航话题接收者
	
		//初始化 移动话题发布者
  	 //	rh_cmd_vel_pub=video_n.advertise<geometry_msgs::Twist>("cmd_vel",1);

		//初始化 导航数据话题接收者
	
		rh_navdata_sub=video_n.subscribe<ardrone_autonomy::Navdata>("ardrone/navdata",1,callback);
	
	
		ros::Rate loop_rate(20);
  
		while (ros::ok() )
		{
   
    		ros::spinOnce();                 
   		 	loop_rate.sleep();
		}
	
    	
		return 0;
}



//稳定
void Stable()
{
	ros::NodeHandle video_n;   //节点句柄
	 ros::Publisher cmd_vel_pub; //移动话题发布者	
	//初始化 移动话题发布者
  	 cmd_vel_pub=video_n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	
	ros::Rate loop_rate(10);
	//time
	int second=0;
	geometry_msgs::Twist speed;
    speed.linear.x=0.0;
    speed.linear.y=0.0;
	speed.linear.z=0.0;

	speed.angular.x=0.0;
	speed.angular.y=0.0;
	speed.angular.z=0.0;
	
	while(ros::ok())
	{
		//only one time can be landed
		if(second<5)
		{
			cmd_vel_pub.publish(speed);  
			cout<<"稳定无人机"<<endl;
		}
		else
			break;
		 
		++second;
	    loop_rate.sleep();
		
	}
	
}

//路径检测函数，检测路径返回方向和距离
void PathDetection(const Mat& image_row)
{
	//转换为HSV格式
	Mat hsv;
	cvtColor(image_row, hsv, COLOR_BGR2HSV);

	//直方图均衡化
	vector<Mat> hsvsplit;
	split(hsv, hsvsplit);
	equalizeHist(hsvsplit[2], hsvsplit[2]);
	merge(hsvsplit, hsv);

	//二值化
	//Mat throed;
	int ilowh = 20;
	int ihighh = 40;

	int ilows = 50;
	int ihighs = 255;

	int ilowv = 0;
	int ihighv = 255;
	inRange(hsv, Scalar(ilowh, ilows, ilowv), Scalar(ihighh, ihighs, ihighv), throed);

	//开闭操作
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(throed, throed, MORPH_OPEN, element);
	morphologyEx(throed, throed, MORPH_CLOSE, element);

	//显示二值化图像
	//imshow("二值化图像", throed);

	//获取图像大小
	int icolst, irowst, imidtc, imidtr;
	icolst = throed.cols;
	irowst = throed.rows;
	//cout<<"icolst: irowst:"<<icolst<<" "<<irowst<<endl;
	imidtc = icolst / 2;
	imidtr = irowst / 2;
	//itenc = icolst / 10;
	//itenr = irowst / 10;
	//cout<<"imidtc: imidtr:"<<imidtc<<" "<<imidtr<<endl;
	//检测第一行和两边,以获取目标点的坐标
	int iaimr1_l=0, iaimr1_r=0;//左右两边目标点
	int iaimr1=0;//第一行中间实际目标点
	int iaimr10_l=0, iaimr10_r=0;//十分之一行处左右两边的目标点
	int iaimr10 = 0;//十分之一行处中间实际目标点
	bool getaimr1 = false;//第一行检测标志
	bool getaimr10 = false;//第十行检测标志
	bool getaimr = false;//行检测标志

	int iaimc1_u=0, iaimc1_d=0;//第一列上下两边目标点
	int iaimc1 = 0;//第一列实际目标点
	int iaimcn_u = 0, iaimcn_d = 0;//最后一列上下两边目标点
	int iaimcn=0;//最后一列的实际目标点
	int iaimc10_u = 0, iaimc10_d = 0;//十分之一列上下两边目标点
	int iaimc10 = 0;//十分之一列的实际目标点
	int iaimc_10_u = 0, iaimc_10_d = 0;//后十分之一列上下两边目标点
	int iaimc_10 = 0;//十分之一列的实际目标点
	bool getaimc1 = false;//第一列检测标志
	bool getaimcn = false;//最后一列检测标志
	bool getaimc10 = false;//十分之一列检测标志
	bool getaimc_10 = false;//十分之一列检测标志
	bool getaimc = false;//列检测标志

	//float angle = 0; //箭头弧度
	//float dist = 0;//箭头距离
	//float pi = 3.14;

	//检测第一行目标点
	//先正向扫描目标点
	for (int i = 0; i < icolst; ++i)
	{
		if (throed.at<uchar>(0, i) == 255)
		{
			iaimr1_l = i;
			break;	
		}
	}
	//再反向扫描
	for (int j = icolst - 1; j >= 0; --j)
	{
		if (throed.at<uchar>(0, j) == 255)
		{
			iaimr1_r = j;
			break;
		}
	}
	//如果检测到两端，并且距离较长，可能存在实际目标点
	if ((iaimr1_r - iaimr1_l) > 5)
	{
		//如果中间点是目标点，则检测到实际目标点
		if (throed.at<uchar>(0, (iaimr1_l + iaimr1_r) / 2) == 255)
		{
			iaimr1 = (iaimr1_l + iaimr1_r) / 2;
			getaimr1 = true;
			
		}
	}

	//如果检测到第一行，说明箭头在第一行
	if (getaimr1 == true)
	{
		//再检测十分之一行
		//先正向扫描目标点
		for (int i = 0; i < icolst; ++i)
		{
			if (throed.at<uchar>(irowst/10, i) == 255)
			{
				iaimr10_l = i;
				break;
			}
		}
		//再反向扫描
		for (int j = icolst - 1; j >= 0; --j)
		{
			if (throed.at<uchar>(irowst/10, j) == 255)
			{
				iaimr10_r = j;
				break;
			}
		}
		//如果检测到两端，并且距离较长，可能存在实际目标点
		if ((iaimr10_r - iaimr10_l) > 5)
		{
			//如果中间点是目标点，则检测到实际目标点
			if (throed.at<uchar>(irowst/10, (iaimr10_l + iaimr10_r) / 2) == 255)
			{
				iaimr10 = (iaimr10_l + iaimr10_r) / 2;
				getaimr10 = true;

			}
		}
	}

	//检测到行
	if (getaimr1 && getaimr10)
	{
		//箭头距离
		dist = imidtc-iaimr1;
		//箭头弧度
		angle = atan2((iaimr10-iaimr1), (irowst / 10));
		//
		getaimr = true;
	}
	else //行检测失败，检测列
	{
		
		//检测第一列
		//先向下扫描目标点
		for (int i = 0; i < irowst; ++i)
		{
			if (throed.at<uchar>(i, 0) == 255)
			{
				iaimc1_u = i;
				break;
			}
		}
		//再向上扫描目标点
		for (int j = irowst - 1; j >= 0; --j)
		{
			if (throed.at<uchar>(j, 0) == 255)
			{
				iaimc1_d = j;
				break;
			}
		}
		//如果检测到两端，并且距离较长，可能存在实际目标点
		if ((iaimc1_d - iaimc1_u) > 5)
		{
			//如果中间点是目标点，则检测到实际目标点
			if (throed.at<uchar>((iaimc1_d + iaimc1_u) / 2, 0) == 255)
			{
				iaimc1 = (iaimc1_d + iaimc1_u) / 2;
				getaimc1 = true;
			}
		}
		//检测十分之一列
		//先向下扫描目标点
		for (int i = 0; i < irowst; ++i)
		{
			if (throed.at<uchar>(i, (icolst / 10)) == 255)
			{
				iaimc10_u = i;
				break;
			}
		}
		//再向上扫描目标点
		for (int j = irowst - 1; j >= 0; --j)
		{
			if (throed.at<uchar>(j, (icolst / 10)) == 255)
			{
				iaimc10_d = j;
				break;
			}
		}
		//如果检测到两端，并且距离较长，可能存在实际目标点
		if ((iaimc10_d - iaimc10_u) > 5)
		{
			//如果中间点是目标点，则检测到实际目标点
			if (throed.at<uchar>((iaimc10_d + iaimc10_u) / 2, (icolst / 10)) == 255)
			{
				iaimc10 = (iaimc10_d + iaimc10_u) / 2;
				getaimc10 = true;
			}
		}
		//再检测最后一列
		//先向下扫描目标点
		for (int i = 0; i < irowst; ++i)
		{
			if (throed.at<uchar>(i, icolst - 1) == 255)
			{
				iaimcn_u = i;
				break;
			}
		}
		//再向上扫描目标点
		for (int j = irowst - 1; j >= 0; --j)
		{
			if (throed.at<uchar>(j, icolst - 1) == 255)
			{
				iaimcn_d = j;
				break;
			}
		}
		//如果检测到两端，并且距离较长，可能存在实际目标点
		if ((iaimcn_d - iaimcn_u) > 5)
		{
			//如果中间点是目标点，则检测到实际目标点
			if (throed.at<uchar>((iaimcn_d + iaimcn_u) / 2, icolst - 1) == 255)
			{
				iaimcn = (iaimcn_d + iaimcn_u) / 2;
				getaimcn = true;
			}
		}
		//检测后十分之一列
		//先向下扫描目标点
		for (int i = 0; i < irowst; ++i)
		{
			if (throed.at<uchar>(i, (icolst / 10) * 9) == 255)
			{
				iaimc_10_u = i;
				break;
			}
		}
		//再向上扫描目标点
		for (int j = irowst - 1; j >= 0; --j)
		{
			if (throed.at<uchar>(j, (icolst / 10) * 9) == 255)
			{
				iaimc_10_d = j;
				break;
			}
		}
		//如果检测到两端，并且距离较长，可能存在实际目标点
		if ((iaimc_10_d - iaimc_10_u) > 5)
		{
			//如果中间点是目标点，则检测到实际目标点
			if (throed.at<uchar>((iaimc_10_d + iaimc_10_u) / 2, (icolst / 10) * 9) == 255)
			{
				iaimc_10 = (iaimc_10_d + iaimc_10_u) / 2;
				getaimc_10 = true;
			}
		}

		//如果检测到
		if (getaimcn == true || getaimc1 == true)
		{
			//如果两边都检测到
			if (getaimcn == true && getaimc1 == true)
			{
				if (iaimc1 < iaimcn)  //算第一列
				{
					
					if (getaimc1&&getaimc10)
					{
						//计算箭头弧度
						angle = atan2((icolst / 10), (iaimc10-iaimc1));
						//计算箭头距离
						dist = imidtc + iaimc1 * tan(angle);
						//
						getaimc = true;
					}
				
				}
				else  //算最后一列
				{
				
					if (getaimcn&&getaimc_10)
					{
						//计算箭头弧度
						angle = atan2(-(icolst / 10), (iaimc_10 - iaimcn));
						//计算箭头距离
						dist = -(imidtc + iaimcn * tan(angle));
						//
						getaimc = true;
					}
				}

			}
			else if (getaimc1 == true && getaimcn == false)//如果只检测到第一列
			{
				if (getaimc1&&getaimc10)
				{
					//计算箭头弧度
					angle = atan2((icolst / 10), (iaimc10 - iaimc1));
					//计算箭头距离
					dist = imidtc + iaimc1 * tan(angle);
					//
					getaimc = true;
				}
			}
			else  //如果只检测到最后一列
			{
				if (getaimcn&&getaimc_10)
				{
					//计算箭头弧度
					angle = atan2(-(icolst / 10), (iaimc_10 - iaimcn));
					//计算箭头距离
					dist = -(imidtc + iaimcn * tan(angle));
					//
					getaimc = true;
				}
			}
			
		}

	}
	
	if (getaimc == false && getaimr == false)
	{
		++times;
		call_vf = false;//停止视频检测循环，飞机降落
		//rr=0;
		//ss=0;
	}
	else
	{
		//角度小于十度，距离小于30进入轨道飞行
		if(fabs(angle) < flight_angle && fabs(dist) < flight_dist)
		{
			fly_s = true;
			++ss;
			rr=0;
			if(ss==1)
			{
				sta_s=true;
				cout<<"到达轨道，稳定无人机准备飞行"<<endl;
			}
			if(ss>=2)
			{	
				ss=2;
			 	sta_s=false;
			}
		}
		
		else
		{
			fly_s = false;
			ss=0;
			++rr;
			if(rr==1)
			{
				sta_s=true;
				cout<<"遇到弯道，稳定无人机准备拐弯"<<endl;
			}
			else
			{
				rr=2;
				sta_s=false;
			}
		}
		
		//返回偏角
		cout << "偏角：" << angle * (180 / PI) << endl;
		cout << "距离：" << dist << endl;
	}
	
	
}


//速度配置函数
geometry_msgs::Twist Adaptive()
{
	//float abs_angle = fabs(angle);//绝对值
	geometry_msgs::Twist speedpid;//速度
	
	//float factorz = 0.3;//角速度乘积因子
	//float factorx = 0.287;//线速度乘积因子
	//double factory = 0.0003;
	//float addz = 0.0; //加值

	//如果在稳定态
	/*if(sta_s)
	{
		speedpid.linear.x =0;
		speedpid.linear.y =0;
		speedpid.linear.z =0;
		speedpid.angular.z =0;
		cout<<"稳定无人机准备飞行或转弯"<<endl;
	}
	else
	{*/
		//如果在飞行态
		if (fly_s)
		{
			speedpid.linear.x = 0.05;//0.1
			speedpid.linear.y =0.02*adaptiveDist(dist);//最大0.01
            speedpid.linear.z =adaptive(e_height);

			speedpid.angular.z =adaptiveYaw(angle);
			cout<<"无人机正在巡航"<<endl;
		}
		//调整态，平移和旋转
		else 
		{
			speedpid.linear.y =0.02*adaptiveDist(dist);//最大0.01
			speedpid.linear.x = 0;
            speedpid.linear.z =adaptive(e_height);
			speedpid.angular.z =adaptiveYaw(angle);//最大1.0
			cout<<"无人机正在转弯"<<endl;
			
		}
	//}
	
	
	cout << "线速度 x= ：" << speedpid.linear.x << endl;
	cout << "线速度 y= ：" << speedpid.linear.y << endl;
	cout << "角速度 ：" << speedpid.angular.z << endl;

	return speedpid;
	
}

//图像回调函数
void video_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& infomsg)
{
	ros::NodeHandle video_cv;   //节点句柄
	ros::Publisher cmd_vel_pub; //移动话题发布者
	//初始化 移动话题发布者
	cmd_vel_pub = video_cv.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	//开始巡线

	 cv_bridge::CvImagePtr cv_ptr; 
	 geometry_msgs::Twist speedCruise;
	// geometry_msgs::Twist speedZero;
	// float s = 0.13; //乘积因子
	

	//转换成BGR格式图像并显示
	try
	{
		//转化成CVImage    
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		image_row = cv_ptr->image;
		//imshow("底部相机原始图像", image_row);

	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception is %s", e.what());
		return;
	}

	//图像检测函数，计算方向弧度和距离偏差
	PathDetection(image_row);
	//float abs_angle_z = fabs(angle_z);//绝对值

	//显示二值化图像
	//imshow("二值化图像", throed);

	//偏向弧度小于安全弧度
	
	//如果检测到路径
	if (call_vf == true)
	{	
		
		//运用算法配置速度
		speedCruise = Adaptive();
		//发送控制指令移动无人机
		cmd_vel_pub.publish(speedCruise);
		
		//ros::Duration(0.05).sleep();
		
		//cmd_vel_pub.publish(speedZero);
		times=0;
		//cout<<"flying for line :巡线飞行"<<endl;
		
	}
	else
	{
		cout << "no path have been find :没有检测到路径" << endl;
		call_vf =true;
		if(times==1000)
		{
			call_vf =false;
		}
	}
	
}



//直线飞行，检测路径
void flight_test_()
{
	 ros::NodeHandle video_;   //节点句柄
	
	
	 image_transport::ImageTransport image_transport(video_);//视频发布类节点
	 image_transport::CameraSubscriber image_sub;//视频话题接收者
	 //初始化 视频话题接收者
	 image_sub = image_transport.subscribeCamera("ardrone/bottom/image_raw", 1 ,video_callback);
	
	
	 ros::Subscriber rh_navdata_sub;//导航话题接收者
		//初始化 导航数据话题接收者
	 rh_navdata_sub=video_.subscribe<ardrone_autonomy::Navdata>("ardrone/navdata",1,callback);
	 
	
	 ros::Publisher move_pub; //移动话题发布者
	 //初始化 移动话题发布者
	 move_pub = video_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	
	 cout<<"开始寻线"<<endl;
	

	
	
	//PID
	//PID_init(0.0);
	 
	
	 int keys_fd;  
	// char ret[2];  
	 struct input_event t;  
	 geometry_msgs::Twist speedadjust;
     keys_fd = open ("/dev/input/event4", O_RDONLY);  
	 if (keys_fd <= 0)  
	 {  
	  cout<<"open /dev/input/event4 device error!\n"<<endl;  
	  return ;  
	 }  
	
	// int seconds=0;
	 ros::Rate loop_rate_(10);
	 //无人机起飞调整
	 while (!start_line)
	 {
		  //读取按键控制无人机
		 if (read (keys_fd, &t, sizeof (t)) == sizeof (t))  
		 {  
			 //cout<<"按键控制"<<endl;
			 if (t.type == EV_KEY)  
					if (t.value == 1)  
				   {  
					    cout<<"按键控制"<<endl;
						// 降落
					     if(t.code==KEY_ESC)
						  break;
						 switch(t.code)
						 {
									 case KEY_O:
									{
							 			cout<<"开始起飞"<<endl;
										takeoff_();//
										break;
									}
									case KEY_L:
									{
							 		  cout<<"开始降落"<<endl;
									  land_();
									  break;
									}
									case KEY_F:
									{ 
									  start_line=true;
							 		  speedadjust.linear.x=0;
									  speedadjust.linear.y=0;
									  speedadjust.linear.z=0;
									  speedadjust.angular.z=0;
							          cout<<"开始巡线"<<endl;
									  break;
									}
								 	/*case KEY_E:
									{ 
									  start_line=false;
							          cout<<"退出巡线"<<endl;
									  break;
									}*/
								   case KEY_LEFT:
								   {
									 speedadjust.linear.y=0.2;
									 cout<<"左移"<<endl;
							 		 break;

								   }
								   case KEY_RIGHT:
								   {
									 speedadjust.linear.y=-0.2;
								     cout<<"右移"<<endl;
									 break;
								   }
									case KEY_UP_UP:
								   {
									speedadjust.linear.x=0.2;
									cout<<"前进"<<endl;
									 break;
								   }
								   case KEY_DOWN_DOWN:
								   {
								 	speedadjust.linear.x=-0.2;
									cout<<"后退"<<endl;
									 break;
								   }
								 	case KEY_D:
								   {
									speedadjust.angular.z=-0.2;
									 cout<<"右转"<<endl;
									break;
								   }
									case KEY_A:
								   {
									 speedadjust.angular.z=0.2;
									 cout<<"左转"<<endl;
									 break;
								   }
								  case KEY_W:
								  {
									speedadjust.linear.z=0.2;
									cout<<"上升"<<endl;
									 break;
								  }
								  case KEY_S:
								  {
									speedadjust.linear.z=-0.2;
								 	cout<<"下降"<<endl;
									 break;
								  }
								  case KEY_H:
								  {
									speedadjust.linear.x=0;
									 speedadjust.linear.y=0;
									 speedadjust.linear.z=0;
									 speedadjust.angular.z=0;
							 		 //Stable();
								 	 cout<<"稳定"<<endl;
									 break;
								  }
								   default:
										break;
								 
						 }
					   
						 
						 
						 move_pub.publish(speedadjust);
						 
					     speedadjust.linear.x=0;
					     speedadjust.linear.y=0;
					     speedadjust.linear.z=0;
					     speedadjust.angular.z=0;
					     // seconds=0;
						  //move_pub.publish(speedadjust);
						 //延时
					     //sleep(100);
				   }  
			      //需要稳定
			 
				  else
				  {
					 move_pub.publish(speedadjust);
					 cout<<"无人机稳定"<<endl;
				  }
		 } 
		 
	   	 //loop_rate_.sleep();
	    
	 }	
	
	
	//无人机进入自主巡线，超过十秒未寻到线降落
	if(start_line)
    {
		cout<<"自主巡线"<<endl;
		ros::Rate loop_rate(100);
		while(ros::ok())
		{	
				if(call_vf)
				{ ros::spinOnce();
				 // Stable();
				}
			    else
				land_();
				loop_rate.sleep();
		 }
	}
	 close (keys_fd);
	// return 0;
}






int main(int argc,char **argv)
{
	//初始化ROS
	ros::init(argc,argv,"cvideo");
	//ros::NodeHandle video_n;   //节点句柄
	cout<<"ros init finshed"<<endl;
	//PID
	 //pid pid_dist(87);
	// pid pid_angle(0.1);
	//创建一个类的实例，来处理所有流程
	//video cvideo;
	//ros::spin();  
  	//cout<<"dist kp"<<pid_dist.get_kp()<<endl;
	//起飞
	 //takeoff_();
	//上升定高，到达指定高度后进入下一个函数
	// Rising_heighted_();
	//稳定手动调整，把无人机调整到路径正上方，检测到路径后进入巡线函数
	//Stable();
	//开始巡线，路径消失后进入降落函数
	 flight_test_();
	//降落
	 land_();
	
	/* cout<<"System begin "<<endl;;
    PID_init(1.0);
    int count=0;
    while(count<100)
    {
        float speed=PID_realize(0.0);
         printf("%f\n",speed);;
        count++;
    }*/

	
  	return 0;  
}

