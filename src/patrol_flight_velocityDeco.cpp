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


double t = 0.0;


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


//自适应定高

double adaptive(double eheight)
{
	
	geometry_msgs::Twist speed;
	
	double k_z = 0.5;
	double pho_z = 0.001;
	double delta_z = 0.001;
	
	double e_z = eheight;
	double u_1 = -k_z*e_z-pho_z*e_z*hat_w_z;
	
	hat_w_sec_z = hat_w_z +(pho_z*e_z*e_z-delta_z*hat_w_z)*0.001;
	hat_w_z = hat_w_sec_z;
	
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
/*double adaptiveYaw(double error_yaw)
{
	
	geometry_msgs::Twist yawspeed;
	
	double k = 0.7;
	double pho = 0.001;
	double delta = 0.001;
	
	double e = -error_yaw;
	double u_2 = -k*e-pho*e*hat_w_yaw;
	
	hat_w_sec_yaw = (pho*e*e-delta*hat_w)*0.01;
	hat_w_yaw = hat_w_sec;
	
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
}*/

//自适应侧移
/*double adaptiveDist(double error_dist)
{
	
	geometry_msgs::Twist distspeed;
	
	double k = 0.005;
	double pho = 0.001;
	double delta = 0.001;
	
	double e = -error_dist;
	double u_3 = -k*e-pho*e*hat_w;
	
	hat_w_sec_dist = (pho*e*e-delta*hat_w)*0.05;
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
}*/


//速度配置函数
geometry_msgs::Twist Adaptive(double velocityX, double velocityY)
{
	
	     geometry_msgs::Twist speedpid;//速度
	
		//如果在飞行态
		if (fly_s)
		{
			speedpid.linear.x = velocityX;//0.1
			speedpid.linear.y = velocityY;//最大0.01
            speedpid.linear.z = adaptive(e_height);

			speedpid.angular.z =0.0;
			cout<<"无人机正在巡航"<<endl;
		}
		//调整态，平移和旋转
		else 
		{
			speedpid.linear.x =0;
		    speedpid.linear.y =0;
		    speedpid.linear.z =0;
		    speedpid.angular.z =0;
		    cout<<"无人机悬停"<<endl;		
		}
	//}
	
	
	cout << "线速度 x= ：" << speedpid.linear.x << endl;
	cout << "线速度 y= ：" << speedpid.linear.y << endl;
	cout << "线速度 z= ：" << speedpid.linear.z << endl;

	return speedpid;
	
}


void callback(const ardrone_autonomy::NavdataConstPtr & nav)  
{
		ros::NodeHandle n;   //节点句柄
		//移动话题发布者
		//ros::Publisher cmd_vel_pub_ada; 
		
		ros::Publisher cmd_vel_deco;
		
		//初始化 移动话题发布者
		//cmd_vel_pub_ada = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
		
		cmd_vel_deco = n.advertise<geometry_msgs::Twist>("cmd_vel",1); 
		

		float alt = nav->altd;
		float battery=nav->batteryPercent;
		cout<<"无人机剩余电量为： "<<battery<<endl;
		cout<<"无人机高度为： "<<alt<<endl;
		//ROS_INFO("the ardrone altitude is %f",alt);

		//ros::Rate loop_rate(10);
	
	        double height= alt/1000;

	        double target_height = 1.500;
            e_height = height-target_height;
		

	        cout<<"rising：height(m)"<< height <<endl;
			cout<<"rising：自适应定高测试"<<endl;
			
			
		    geometry_msgs::Twist speedCruise;
			
		if(start_line)
		{
			static ros::Time t_start = ros::Time::now();
                        t = (ros::Time::now()-t_start).toSec();
		   
			if(t<=4)
			{
				speedCruise = Adaptive(0.05,0.0);
			}
			else if((t>4)&&(t<=8))
			{
				speedCruise = Adaptive(0.05,0.05);
			}
			else
			{
				//speedCruise = Adaptive(0.0,0.0);
                                land_();
			}
		}
		
		cout<<"时间"<<t<<endl;
		
		 
		cmd_vel_deco.publish(speedCruise);	
		
		//
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
	
	
		ros::Rate loop_rate(40);
  
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
	
	ros::Rate loop_rate(40);
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


//图像回调函数
/*void video_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& infomsg)
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
	
}*/



//直线飞行，检测路径
void flight_test_()
{
	 ros::NodeHandle video_;   //节点句柄
	
	
	 image_transport::ImageTransport image_transport(video_);//视频发布类节点
	 image_transport::CameraSubscriber image_sub;//视频话题接收者
	 
	 
	 //初始化 视频话题接收者
	 //image_sub = image_transport.subscribeCamera("ardrone/bottom/image_raw", 1 ,video_callback);
	
	
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
									  fly_s = true;
							 		  speedadjust.linear.x=0;
									  speedadjust.linear.y=0;
									  speedadjust.linear.z=0;
									  speedadjust.angular.z=0;
							          cout<<"水平速度分解控制"<<endl;
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
		cout<<"水平速度分解控制"<<endl;
		ros::Rate loop_rate(100);
		while(ros::ok())
		{	
				if(call_vf)
				{ 
			      ros::spinOnce();    
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
	

  	return 0;  
}

