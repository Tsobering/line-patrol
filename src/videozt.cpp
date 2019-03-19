//常规包含
#include<iostream>  
#include<stdlib.h>  
#include<math.h>  

//linux/input.h->linux/input-event-codes.h和opencv2/opencv.hpp宏定义冲突
#include<linux/input.h>  
#include<fcntl.h>  
 

//ROS相关包含
#include<ros/ros.h>  //+
#include<image_transport/image_transport.h>//+
#include<sensor_msgs/image_encodings.h> //+
#include<cv_bridge/cv_bridge.h>//+
#include<sensor_msgs/Image.h>//+
#include<std_msgs/Empty.h>//+
#include<geometry_msgs/Twist.h>//+


//OPENCV包含
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>


#define uchar unsigned char
#define PI 3.14

using namespace std;
using namespace cv;

struct info {
	double angle;
	double distance;
};
struct info z_info={0.0,0.0};

//double angle=0.0,distance=0.0;

bool start_line=false;

Mat zt_dealpicture;
Mat image_row;

class pid
{
	private:
		float Setvalue;            //定义设定值
		float Actualvalue;         //定义实际值
		float err;                 //定义偏差值
		float err_last;            //定义上一个偏差值
		float Kp,Ki,Kd;            //定义比例、积分、微分系数
		float speed;               //定义速度（控制执行器的变量）
		float integral;            //定义积分值
  		float critical_point;      //临界点
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
				speed=Kp*err;
				return speed;
		}
	
};
pid pid_dist(0.00015);
pid pid_angle(0.636);

geometry_msgs::Twist PID()
{
	
	geometry_msgs::Twist speedpid;//速度

	
	speedpid.linear.x = 0.05;//0.1
	speedpid.linear.y =pid_dist.PID_realize(z_info.distance);//最大0.01
	speedpid.angular.z =pid_angle.PID_realize(z_info.angle);

	cout << "线速度 x= ：" << speedpid.linear.x << endl;
	cout << "线速度 y= ：" << speedpid.linear.y << endl;
	cout << "角速度 ：" << speedpid.angular.z << endl;

	return speedpid;
	
}

cv::Mat  dealpicture(const cv::Mat&img1 )	{	
		cv::cvtColor(img1,img1,COLOR_BGR2HSV);
		vector<Mat> hsvsplit;
		cv::split(img1,hsvsplit);
		cv::equalizeHist(hsvsplit[2],hsvsplit[2]);
		cv::merge(hsvsplit,img1);
	
		cv::Mat throed1;
		int ilowh=20;
		int ihighh=40;

		int ilows=43;
		int ihighs=255;

		int ilowv=46;
		int ihighv=255;
		inRange(img1,Scalar(ilowh,ilows,ilowv),Scalar(ihighh,ihighs,ihighv),throed1);
		
		cv::Mat element=getStructuringElement(MORPH_RECT,Size(5,5));
		morphologyEx(throed1,throed1,MORPH_OPEN,element);
		morphologyEx(throed1,throed1,MORPH_CLOSE,element);	
		
		return throed1;
	
}

int get_angle_distance(const cv::Mat& picture){
	
	int h=abs(picture.rows),w=abs(picture.cols),//获取的图像像素总大小
		ir=0,il=0,//找到线的第1行与第20行
		r=0,l=20;//计算偏角时横向距离与纵向距离
		
	double T=0,//偏差角弧度
		angle=0;//偏差角角度
	bool findi=false,findj=false;//两行对应的找到标志
	for(int i=0;i<=w;i++){
		if(picture.at<uchar>(h,i)==255 && picture.at<uchar>(h,i+1)==255)
				ir=i;
				findi=true;}
				//break;}
	for(int j=0;j<=w;j++){
		if(picture.at<uchar>(h-l,j)==255 && picture.at<uchar>(h-l,j+1)==255)
				il=j;
				findj=true;}
				//break;}
		if(findi && findj)		
		z_info.distance=w/2-ir;
		r=il-ir;
		T=atan2(r,l);
		z_info.angle=T/PI;
		cout<<"the distance is "<<z_info.distance<<endl;
		cout<<"the angle is"<<T/PI*180<<endl;
		//z_info.angle=T/3.1415926*180;
		return 0;
}	

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



//降落
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
void video_callback(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::CameraInfoConstPtr& infomsg){
	ros::NodeHandle video_cv;   //节点句柄
	ros::Publisher cmd_vel_pub; //移动话题发布者
	cmd_vel_pub = video_cv.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);

	cv_bridge::CvImagePtr cv_ptr; 
	geometry_msgs::Twist speedCruise;
	
	try
	{   
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		image_row = cv_ptr->image;
		zt_dealpicture=dealpicture(image_row);
		get_angle_distance(zt_dealpicture);

	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception is %s", e.what());
		return;
	}

	speedCruise = PID();
	cmd_vel_pub.publish(speedCruise);
	
}
void flight_key_ctrl()
{
	 ros::NodeHandle video_;   //节点句柄
	 image_transport::ImageTransport image_transport(video_);//视频发布类节点
	 image_transport::CameraSubscriber image_sub;//视频话题接收者
	
	 ros::Publisher move_pub; //移动话题发布者
	 ros::Publisher camera_ctrl_pub;// 移动话题发布者
	 
	
	 move_pub = video_.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);
	 camera_ctrl_pub =video_.advertise< geometry_msgs::Twist>("bebop/camera_control", 1);
	 cout<<"开始寻线"<<endl;
	
	 image_sub = image_transport.subscribeCamera("bebop/image_raw", 1 ,video_callback);//初始化 视频话题接收者
	 
	 int keys_fd;    
	 struct input_event t;  
	 geometry_msgs::Twist speedadjust,cameraadjust;
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
								 	 cout<<"稳定"<<endl;
									 break;
								  }
							 	  case KEY_I:
								 {
								 cameraadjust.angular.y=-50;
								 break;
								 }
							      case KEY_J:
								 {
								 cameraadjust.angular.y=50;
								 break;
								 }
								   default:
										break;
								 
						 }
					   
						 
						 
						 move_pub.publish(speedadjust);
						 camera_ctrl_pub.publish(cameraadjust);
					     speedadjust.linear.x=0;
					     speedadjust.linear.y=0;
					     speedadjust.linear.z=0;
					     speedadjust.angular.z=0;
					     
				   }  
			      //需要稳定
			 
				  else
				  {
					 move_pub.publish(speedadjust);
					 cout<<"无人机稳定"<<endl;
				  }
		 } 
		 
	   
	    
	 }	
	
	
	//无人机进入自主巡线，超过十秒未寻到线降落
	if(start_line)
    {
		cout<<"自主巡线"<<endl;
		ros::Rate loop_rate(10);
		while(ros::ok())
		{	
				
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	 close (keys_fd);
	// return 0;
}
int main(int argc,char **argv)
{
	ros::init(argc,argv,"cvideo");
	cout<<"ros init finshed"<<endl;
	
	flight_key_ctrl();
	land_();

}
	
