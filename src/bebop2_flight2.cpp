//常规包含
#include<iostream>  
#include<stdlib.h>  
#include<math.h>  
#include<syslog.h> 
#include <fstream>

//linux/input.h->linux/input-event-codes.h和opencv2/opencv.hpp宏定义冲突,需要更改
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
#include<ardrone_autonomy/Navdata.h>  // bebop2???

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

//float angle = 0; //箭头弧度
//float dist = 0;//箭头距离
//double e_height = 0.0;

bool fly_s = false;//飞行态和调整态
bool sta_s=true;//稳定态
int ss=0;
int rr=0;
//开始巡线标志
bool start_line=false;

//飞行合理角度
float flight_angle=0.3;
float flight_dist=300;//200mm
//创建两个窗口
//cv::namedWindow("底部机原始图像");
//cv::namedWindow("二值化图像");

int times=0;

Mat image_row; //原始图
Mat throed; //二值图
Mat image_jz; //矫正图
Mat picturezt;
Mat image_cut;   //剪切图
Mat image_rotate; //旋转图
//每个像素点的物理长度
double pixel_dist=0.0;

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


//图像畸变矫正函数
Mat jiaozheng(const Mat img){
	Size image_size;
	Mat image=img;
	image_size.width = image.cols;
	image_size.height =image.rows;
	Mat cameraMatrix=(Mat_<double>(3,3)<<28776.19244627709, 0, 652.5330511574412,
 0, 2715.441922565612, 68.03217023107095,
 0, 0, 1);	
	Mat distCoeffs=(Mat_<double>(1,5)<<21.23088641591238, 42212.6658764911, 5.821176564823733, 0.126619400207104, 12721.13193573747);
	Mat mapx = Mat(image_size,CV_32FC1);
	Mat mapy = Mat(image_size,CV_32FC1);
	Mat R = Mat::eye(3,3,CV_32F);
	initUndistortRectifyMap(cameraMatrix,distCoeffs,R,cameraMatrix,image_size,CV_32FC1,mapx,mapy);
	Mat newimage = image.clone();
	remap(image,newimage,mapx, mapy, INTER_LINEAR);
	//imshow("after_calibration",newimage);
	return newimage;

}


//图像剪切函数
Mat cut(const Mat image){
Rect rect(320, 20, 436, 204);
Mat image_roi = image(rect);
return image_roi;
}

//图片旋转函数 
void imgrotate(Mat& img, Mat& newIm, double angle){
    //int len = max(img.cols, img.rows);
   // Point2f pt(len/2.,len/2.);
    //Mat r = getRotationMatrix2D(pt,angle,1.0);
    //warpAffine(img,newIm,r,Size(len,len));
	
    //better performance : 
    Point2f pt(img.cols/2.,img.rows/2.);  //旋转中心
    Mat r = getRotationMatrix2D(pt,angle,1.0);  //1.0为缩放尺度
    warpAffine(img,newIm,r,img.size());    //旋转
}

struct errors
{
	double angle; //箭头弧度
	double dist;//箭头距离
	double e_height;//高度误差
	double true_angle;//真实角度
};

struct errors my_error={0.0,0.0,0.0,0.0};  //初始化误差结构体
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


//把数据写入文件
void dataTotext(const struct errors errors_)
{
	//我们首先在这个程序所在的文件夹下新建了一个名为data.txt的文件，并在里面写入 1,2 ,注意要在英文状态下输入
    /*int a, b; char c;     //设置三个变量分别对应上面的两个数字1,2和一个字符“,”
    ifstream fin("data.txt");   //读取文件
    if (!fin)              // 如果读取失败，打印fail
    {
        cerr << "fail" << endl;
        return -1;
    }
    fin >> a >> c >> b;   //读取的东西写入给变量
    cout << "a = " << a << endl;
    cout << "b = " << b << endl;
    fin.close();       //关闭文件
    a++; b++;*/
	char buffer[1024] = {0};
	//buffer<<errors_.angle<<"\t"<<errors_.dist<<"\t"<<errors_.true_angle<<"\n"<<endl;  
	sprintf(buffer,"%0.2f\t%0.2f\t%0.2f\n",errors_.angle,errors_.dist,errors_.true_angle);
	FILE* file = fopen("data.txt","a+");  
        fwrite(buffer,1,strlen(buffer),file);  
        fclose(file);  
	//ofstream fout("data.txt");     //创建一个data.txt的文件
    cout <<errors_.angle<<"\t"<<errors_.dist<<"\t"<<errors_.true_angle<<"\n"<<endl;
   
    //fout <<errors_.angle<<"\t"<<errors_.dist<<"\t"<<errors_.true_angle<<"\n"<<endl;  //将变量的值写入文件
    //fout.close();                  //关闭文件
	return ;
}

//日志文件，把数据写入文件中
/*void sLOG(const char* ms)  
{  
        char wzLog[1024] = {0};  
        char buffer[1024] = {0};  
       // time_t now;  
       // time(&now);  
       // struct tm *local;  
       // local = localtime(&now);  
        printf("%04d-%02d-%02d %02d:%02d:%02d %s\n", local->tm_year+1900, local->tm_mon,  local->tm_mday, local->tm_hour, local->tm_min, local->tm_sec, wzLog);  
        sprintf(buffer,"%04d-%02d-%02d %02d:%02d:%02d %s %s \n", local->tm_year+1900, local->tm_mon,local->tm_mday, local->tm_hour, local->tm_min, local->tm_sec, wzLog, ms);  
        FILE* file = fopen("control1.log","a+");  
        fwrite(buffer,1,strlen(buffer),file);  
        fclose(file);  
        return ;  
}   */

/**void fLOG(const float ms)  
{  
        char wzLog[1024] = {0};  
        char buffer[1024] = {0};  
        time_t now;  
        time(&now);  
        struct tm *local;  
        local = localtime(&now);  
        printf("%04d-%02d-%02d %02d:%02d:%02d %s\n", local->tm_year+1900, local->tm_mon,  local->tm_mday, local->tm_hour, local->tm_min, local->tm_sec, wzLog);  
        sprintf(buffer,"%04d-%02d-%02d %02d:%02d:%02d %s %s sensor-height %f\n", local->tm_year+1900, local->tm_mon,local->tm_mday, local->tm_hour, local->tm_min, local->tm_sec, wzLog, ms);  
        FILE* file = fopen("testResut.log","a+");  
        fwrite(buffer,1,strlen(buffer),file);  
        fclose(file);  
        return ;  
}  */




//起飞
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

void circle()
{
	ros::NodeHandle video_circle;   //节点句柄
	ros::Publisher cmd_vel_circle; //移动话题发布者
	//初始化 移动话题发布者
	cmd_vel_circle = video_circle.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);

	ros::Rate loop_rate(10);
	//time
	
	geometry_msgs::Twist speed_circle;
	speed_circle.linear.x=0.02;
	speed_circle.linear.y=0.0;
	speed_circle.linear.z=0.0;
	speed_circle.angular.x=0.0;
	speed_circle.angular.y=0.0;
	speed_circle.angular.z=-0.2;
	
	while(ros::ok())
	{
		//only one time can be landed
		
		
			//发送着陆指令
			cmd_vel_circle.publish(speed_circle);
			cout<<"pub land：无人机绕圈"<<endl;
		
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
//返回u_z最大0.15
double adaptive(double eheight)
{
	
	geometry_msgs::Twist speed;
	
	double k_z = 0.5;
	double pho_z = 0.001;
	double delta_z = 0.001;
	
	double e_z = eheight;
	double u_1 = -k_z*e_z-pho_z*e_z*hat_w_z;
	
	hat_w_sec_z = hat_w_z+(pho_z*e_z*e_z-delta_z*hat_w_z)*0.001;
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
//返回u_yaw最大0.8
double adaptiveYaw(double error_yaw)
{
	
	geometry_msgs::Twist yawspeed;
	
	double k_yaw = 0.7;
	double pho_yaw = 0.001;
	double delta_yaw = 0.001;
	
	double e_yaw = -error_yaw;
	double u_2 = -k_yaw*e_yaw-pho_yaw*e_yaw*hat_w_yaw;
	
	hat_w_sec_yaw = hat_w_yaw+(pho_yaw*e_yaw*e_yaw-delta_yaw*hat_w_yaw)*0.01;
	hat_w_yaw = hat_w_sec_yaw;
	
	//换成角度，增大180/3.14倍
	u_2 = u_2*3.14/180;
	
	if((u_2)>=0.8)
	{
		u_yaw = 0.8;
	}
	else if((u_2)<=-0.8)
	{
		u_yaw = -0.8;
	}
	else
	{
		u_yaw = (u_2);
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
//返回u_dist最大0.05
double adaptiveDist(double error_dist)
{
	
	geometry_msgs::Twist distspeed;
	
	double k_dist = 0.0005;
	double pho_dist = 0.0001;
	double delta_dist = 0.0001;
	
	double e_dist = -error_dist;
	double u_3 = -k_dist*e_dist-pho_dist*e_dist*hat_w_dist;
	
	hat_w_sec_dist = hat_w_dist+(pho_dist*e_dist*e_dist-delta_dist*hat_w_dist)*0.01;
	hat_w_dist = hat_w_sec_dist;
	
	if(u_3>=0.05)
	{
		u_dist = 0.05;
	}
	else if(u_3<=-0.05)
	{
		u_dist = -0.05;
	}
	else
	{
		u_dist = u_3;
	}

       // u_dist = u_3;
	
	distspeed.linear.x=0;
	distspeed.linear.y=u_dist;
	distspeed.linear.z = 0;
	distspeed.angular.x=0;
	distspeed.angular.y=0;
	distspeed.angular.z=0;
	
	//cout<<"转向控制输入angular.z"<< distspeed.angular.z<<endl;
	
	return u_dist;
}

//定高目前来说没有用
void callback(const ardrone_autonomy::NavdataConstPtr & nav)  
{
		ros::NodeHandle n;   //节点句柄
		//移动话题发布者
		ros::Publisher cmd_vel_pub_ada; 
		//初始化 移动话题发布者
		cmd_vel_pub_ada = n.advertise<geometry_msgs::Twist>("bebop/cmd_vel",1);

		float alt = nav->altd;
		float battery=nav->batteryPercent;
		cout<<"无人机剩余电量为： "<<battery<<endl;
		cout<<"无人机高度为： "<<alt<<endl;
		//ROS_INFO("the ardrone altitude is %f",alt);

		//ros::Rate loop_rate(10);
	
	        double height= alt/1000;

	        double target_height = 1.200;
           	 my_error.e_height = height-target_height;
		
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
	
		rh_navdata_sub=video_n.subscribe<ardrone_autonomy::Navdata>("bebop/navdata",1,callback);
	
	
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
  	 cmd_vel_pub=video_n.advertise<geometry_msgs::Twist>("bebop/cmd_vel",1);
	
	ros::Rate loop_rate(20);
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



//（针对正常非畸变摄像头）路径检测函数，输入图像输出路径方向和距离
void PathDetection_normal(const Mat& image_row)
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
	pixel_dist=0.0;
	double line_a = 0.0;  //扫描线宽度
	double line_b = 0.0;//真实线宽度
	
	//获取图像大小
	int icolst, irowst, imidtc, imidtr;
	icolst = throed.cols;
	irowst = throed.rows;
	cout<<"图像宽: 图像高:"<<icolst<<" "<<irowst<<endl;
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
	bool getaimcn_10=false;//右列检测标志
	bool getaimc1_10=false;//左列检测标志
	bool getaimc10 = false;//十分之一列检测标志
	bool getaimc_10 = false;//十分之一列检测标志
	
	bool getaimc = false;//列检测标志


	//float angle = 0; //箭头弧度
	//float dist = 0;//箭头距离
	//float pi = 3.14;

	//检测最后一行目标点
	//先正向扫描目标点
	for (int i = 0; i < icolst; ++i)
	{
		if (throed.at<uchar>(irowst-1, i) == 255)
		{
			iaimr1_l = i;
			break;	
		}
	}
	//再反向扫描
	for (int j = icolst - 1; j >= 0; --j)
	{
		if (throed.at<uchar>(irowst-1, j) == 255)
		{
			iaimr1_r = j;
			break;
		}
	}
	//如果检测到两端，并且距离较长，可能存在实际目标点
	if ((iaimr1_r - iaimr1_l) > 3)
	{
		//如果中间点是目标点，则检测到实际目标点
		if (throed.at<uchar>(irowst-1, (iaimr1_l + iaimr1_r) / 2) == 255)
		{
			line_a = iaimr1_r - iaimr1_l;//最后一行线宽度
			iaimr1 = (iaimr1_l + iaimr1_r) / 2;
			getaimr1 = true;
			
		}
	}
	
	//如果检测到最后一行，说明箭头在最后一行
	if (getaimr1 == true)
	{
		//再检测倒数十分之一行
		//先正向扫描目标点
		for (int i = 0; i < icolst; ++i)
		{
			if (throed.at<uchar>(irowst*0.9, i) == 255)
			{
				iaimr10_l = i;
				break;
			}
		}
		//再反向扫描
		for (int j = icolst - 1; j >= 0; --j)
		{
			if (throed.at<uchar>(irowst*0.9, j) == 255)
			{
				iaimr10_r = j;
				break;
			}
		}
		//如果检测到两端，并且距离较长，可能存在实际目标点
		if ((iaimr10_r - iaimr10_l) > 3)
		{
			//如果中间点是目标点，则检测到实际目标点
			if (throed.at<uchar>(irowst*0.9, (iaimr10_l + iaimr10_r) / 2) == 255)
			{
				line_a=(line_a + (iaimr10_r - iaimr10_l))/2;//扫描线宽度
				iaimr10 = (iaimr10_l + iaimr10_r) / 2;
				getaimr10 = true;

			}
		}
	}

	//检测到行
	if (getaimr1 && getaimr10)
	{
		
		//箭头角度
		my_error.angle = atan2(-(iaimr10-iaimr1), (irowst / 10))* (180 / PI);
			
		//每个像素的长度,总长度为60000um
		pixel_dist=60000/(line_a * cos(fabs(my_error.angle)));

		//箭头距离,精确到MM
		my_error.dist = (imidtc-iaimr1)*(pixel_dist/1000);
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
		if ((iaimc1_d - iaimc1_u) > 3)
		{
			//如果中间点是目标点，则检测到实际目标点
			if (throed.at<uchar>((iaimc1_d + iaimc1_u) / 2, 0) == 255)
			{

				iaimc1 = (iaimc1_d + iaimc1_u) / 2;
				getaimc1 = true;
			}
		}
		
		//如果检测到第一列，再检测十分之一列
		if(getaimc1 = true)
		{
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
			if ((iaimc10_d - iaimc10_u) > 3)
			{
				//如果中间点是目标点，则检测到实际目标点
				if (throed.at<uchar>((iaimc10_d + iaimc10_u) / 2, (icolst / 10)) == 255)
				{
					line_a=((iaimc1_d - iaimc1_u) + (iaimc10_d - iaimc10_u))/2;
					iaimc10 = (iaimc10_d + iaimc10_u) / 2;
					getaimc10 = true;
					getaimc1_10=true;
				}
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
		if ((iaimcn_d - iaimcn_u) > 3)
		{
			//如果中间点是目标点，则检测到实际目标点
			if (throed.at<uchar>((iaimcn_d + iaimcn_u) / 2, icolst - 1) == 255)
			{
				
				iaimcn = (iaimcn_d + iaimcn_u) / 2;
				getaimcn = true;
			}
		}
		//如果检测到最后一列，再检测后十分之一列
		if(getaimcn = true)
		{
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
			if ((iaimc_10_d - iaimc_10_u) > 3)
			{
				//如果中间点是目标点，则检测到实际目标点
				if (throed.at<uchar>((iaimc_10_d + iaimc_10_u) / 2, (icolst / 10) * 9) == 255)
				{
					if(getaimc1_10==false)
					{
						line_a=((iaimcn_d - iaimcn_u)+(iaimc_10_d - iaimc_10_u))/2;
					}
				
					iaimc_10 = (iaimc_10_d + iaimc_10_u) / 2;
					getaimc_10 = true;
					getaimcn_10=true;
				}
			}
		}

		//如果检测到
		if (getaimcn_10 == true || getaimc1_10 == true)
		{
			//如果两边都检测到
			if (getaimcn_10 == true && getaimc1_10 == true)
			{
				if (iaimc1 < iaimcn)  //算第一列
				{
					
					
					//计算箭头弧度
					my_error.angle = atan2((icolst / 10), (iaimc10-iaimc1))* (180 / PI);
					//每个像素的长度,总长度为60000um
					pixel_dist=60000/(line_a * sin(fabs(my_error.angle)));
					//计算头距离,精确到MM
					my_error.dist = (imidtc + (irowst-iaimc1) * tan(fabs(my_error.angle)))*(pixel_dist/1000);
					//得到角度和距离
					getaimc = true;
					
				
				}
				else  //算最后一列
				{
				
					
						//计算箭头弧度
						my_error.angle = atan2(-(icolst / 10), (iaimc_10 - iaimcn))*(180 / PI);
						//每个像素的长度,总长度为60000um
						pixel_dist=60000/(line_a * sin(fabs(my_error.angle)));
						//计算箭头距离
						my_error.dist = -(imidtc + (irowst-iaimcn) * tan(fabs(my_error.angle)))*(pixel_dist/1000);
						//
						getaimc = true;
					
				}

			}
			else if (getaimc1_10 == true && getaimcn_10 == false)//如果只检测到第一列
			{
				
					//计算箭头弧度
					my_error.angle = atan2((icolst / 10), (iaimc10-iaimc1))* (180 / PI);
					//每个像素的长度,总长度为60000um
					pixel_dist=60000/(line_a * sin(fabs(my_error.angle)));
					//计算头距离,精确到MM
					my_error.dist = (imidtc + (irowst-iaimc1) * tan(fabs(my_error.angle)))*(pixel_dist/1000);
					//得到角度和距离
					getaimc = true;
				
			}
			else  //如果只检测到最后一列
			{
				
					//计算箭头弧度
					my_error.angle = atan2(-(icolst / 10), (iaimc_10 - iaimcn))*(180 / PI);
					//每个像素的长度,总长度为60000um
					pixel_dist=60000/(line_a * sin(fabs(my_error.angle)));
					//计.算箭头距离
					my_error.dist = -(imidtc + (irowst-iaimcn) * tan(fabs(my_error.angle)))*(pixel_dist/1000);
					//
					getaimc = true;
				
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
		if(fabs(my_error.angle) < flight_angle && fabs(my_error.dist) < flight_dist)
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
		
		my_error.true_angle=my_error.angle+my_error.dist*0.05; //
		//返回偏角和距离
		cout<<"pixel_dist:" << (pixel_dist/1000)<<endl;
		cout << "偏角：                        " << my_error.angle << endl;
		cout << "距离：" << my_error.dist << endl;
	}
	
	//把数据写入文件
	//my_error.true_angle=my_error.angle;  //真实角度是多少？
	dataTotext(my_error);
	
}






//(非正常畸变镜头)路径检测函数，检测路径返回方向和距离
void PathDetection_unormol(const Mat& image_row)
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
	pixel_dist=0.0;
	//获取图像大小
	int icolst, irowst, imidtc, imidtr;
	icolst = throed.cols;
	irowst = throed.rows;
	cout<<"图像宽: 图像高:"<<icolst<<" "<<irowst<<endl;
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

	//检测最后一行目标点
	//先正向扫描目标点
	for (int i = 0; i < icolst; ++i)
	{
		if (throed.at<uchar>(irowst-1, i) == 255)
		{
			iaimr1_l = i;
			break;	
		}
	}
	//再反向扫描
	for (int j = icolst - 1; j >= 0; --j)
	{
		if (throed.at<uchar>(irowst-1, j) == 255)
		{
			iaimr1_r = j;
			break;
		}
	}
	//如果检测到两端，并且距离较长，可能存在实际目标点
	if ((iaimr1_r - iaimr1_l) > 5)
	{
		//如果中间点是目标点，则检测到实际目标点
		if (throed.at<uchar>(irowst-1, (iaimr1_l + iaimr1_r) / 2) == 255)
		{
			pixel_dist=60000/(iaimr1_r - iaimr1_l);//每个像素的长度,总长度为60000um
			iaimr1 = (iaimr1_l + iaimr1_r) / 2;
			getaimr1 = true;
			
		}
	}
	
	//如果检测到最后一行，说明箭头在最后一行
	if (getaimr1 == true)
	{
		//再检测倒数十分之一行
		//先正向扫描目标点
		for (int i = 0; i < icolst; ++i)
		{
			if (throed.at<uchar>(irowst*0.9, i) == 255)
			{
				iaimr10_l = i;
				break;
			}
		}
		//再反向扫描
		for (int j = icolst - 1; j >= 0; --j)
		{
			if (throed.at<uchar>(irowst*0.9, j) == 255)
			{
				iaimr10_r = j;
				break;
			}
		}
		//如果检测到两端，并且距离较长，可能存在实际目标点
		if ((iaimr10_r - iaimr10_l) > 5)
		{
			//如果中间点是目标点，则检测到实际目标点
			if (throed.at<uchar>(irowst*0.9, (iaimr10_l + iaimr10_r) / 2) == 255)
			{
				pixel_dist=(60000/(iaimr10_r - iaimr10_l)+pixel_dist)/2;
				iaimr10 = (iaimr10_l + iaimr10_r) / 2;
				getaimr10 = true;

			}
		}
	}

	//检测到行
	if (getaimr1 && getaimr10)
	{
		//箭头距离mm
		my_error.dist = (imidtc-iaimr1)*(pixel_dist/1000);
		//箭头弧度
		my_error.angle = atan2(-(iaimr10-iaimr1), (irowst / 10));
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
				pixel_dist=60000/(iaimc1_d - iaimc1_u);
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
				pixel_dist=(60000/(iaimc10_d - iaimc10_u)+pixel_dist)/2;
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
				if(getaimc1==false && getaimc10==false)
				{
					pixel_dist=60000/(iaimcn_d - iaimcn_u);
				}
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
				if(getaimc1==false && getaimc10==false)
				{
					pixel_dist=(60000/(iaimc_10_d - iaimc_10_u)+pixel_dist)/2;
				}
				
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
						my_error.angle = atan2((icolst / 10), (iaimc10-iaimc1));
						//计算箭头距离
						my_error.dist = (imidtc + iaimc1 * tan(my_error.angle))*(pixel_dist/1000);
						//
						getaimc = true;
					}
				
				}
				else  //算最后一列
				{
				
					if (getaimcn&&getaimc_10)
					{
						//计算箭头弧度
						my_error.angle = atan2(-(icolst / 10), (iaimc_10 - iaimcn));
						//计算箭头距离
						my_error.dist = -(imidtc + iaimcn * tan(my_error.angle))*(pixel_dist/1000);
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
					my_error.angle = atan2((icolst / 10), (iaimc10 - iaimc1));
					//计算箭头距离
					my_error.dist = (imidtc + iaimc1 * tan(my_error.angle))*(pixel_dist/1000);
					//
					getaimc = true;
				}
			}
			else  //如果只检测到最后一列
			{
				if (getaimcn&&getaimc_10)
				{
					//计算箭头弧度
					my_error.angle = atan2(-(icolst / 10), (iaimc_10 - iaimcn));
					//计算箭头距离
					my_error.dist = -(imidtc + iaimcn * tan(my_error.angle))*(pixel_dist/1000);
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
		if(fabs(my_error.angle) < flight_angle && fabs(my_error.dist) < flight_dist)
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
		
		//返回偏角和距离
		cout<<"pixel_dist:" << (pixel_dist/1000)<<endl;
		cout << "偏角：                        " << my_error.angle  << endl;
		cout << "距离：" << my_error.dist << endl;
	}
	
	
}


//速度配置函数
geometry_msgs::Twist AdaptiveUnnormal()
{
	//float abs_angle = fabs(angle);//绝对值
	geometry_msgs::Twist speedpid;//速度
	
	//float factorz = 0.3;//角速度乘积因子
	//float factorx = 0.287;//线速度乘积因子
	//double factory = 0.0003;
	//float addz = 0.0; //加值
	
	
	if(fabs(my_error.dist)<flight_dist){
		if(fabs(my_error.angle)<flight_angle){
			speedpid.linear.x = 0.05;//0.1
		
			speedpid.linear.y =-0.2*adaptiveDist(my_error.dist);
		
			if(adaptiveYaw(my_error.angle)>1.0)
			{
				speedpid.angular.z=1.0;
			}
			else if(adaptiveYaw(my_error.angle)<-1.0)
			{
				speedpid.angular.z=-1.0;
			}
			else			
				speedpid.angular.z =adaptiveYaw(my_error.angle);
		}
		else{
			speedpid.linear.x = 0.02;//0.1
		
			speedpid.linear.y =-0.2*adaptiveDist(my_error.dist);
		
			if(adaptiveYaw(my_error.angle)>1.0)
			{
				speedpid.angular.z=1.0;
			}
			else if(adaptiveYaw(my_error.angle)<-1.0)
			{
				speedpid.angular.z=-1.0;
			}
			else			
				speedpid.angular.z =adaptiveYaw(my_error.angle);
		}
	
		cout<<"无人机只转弯"<<endl;
	}
	else{
			speedpid.linear.y =-adaptiveDist(my_error.dist);
			speedpid.linear.x = 0.01;
			speedpid.angular.z =0.0;
			cout<<"无人机只平移"<<endl;
	}
		
	
	
	
	cout << "线速度 x= ：" << speedpid.linear.x << endl;
	cout << "线速度 y= ：" << speedpid.linear.y << endl;
	cout << "```````````````````````角速度 ：            " << speedpid.angular.z << endl;

	return speedpid;
	
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
			/*if(abs(angle)<0.15 && abs(dist)<100)
			{
				speedpid.linear.x = 0.2;//0.1
			}
			else
				speedpid.linear.x = 0.1;*/
				//speedpid.linear.x = 0.05*(1-abs(angle)/1.57);//0.1
			speedpid.linear.x = 0.1;//0.1
		
			speedpid.linear.y =0.1*adaptiveDist(my_error.dist);//以毫米为单位
			/*if(2*adaptiveYaw(angle)>1.0)
			{
				speedpid.angular.z=1.0;
			}
			else if(2*adaptiveYaw(angle)<-1.0)
			{
				speedpid.angular.z=-1.0;
			}
			else			
				speedpid.angular.z =adaptiveYaw(angle);*/
			speedpid.angular.z =adaptiveYaw(my_error.angle);
            		//speedpid.linear.z =adaptive(e_height);
			/*if(angle>0)
			{
			  double temn=0.5*adaptiveYaw(angle);
			  if(temn>1.0)
			  {
				speedpid.angular.z =1.0;
			  }
		   	  else
				speedpid.angular.z=temn;
				
			}
			else
			{
				speedpid.angular.z =0.5*adaptiveYaw(angle);
			}*/
			cout<<"无人机正在巡航"<<endl;
		}
		//调整态，平移和旋转
		else 
		{
			speedpid.linear.y =0.3*adaptiveDist(my_error.dist);
			speedpid.linear.x = 0.05;

			if(adaptiveYaw(my_error.angle)>1.0)
			{
				speedpid.angular.z=1.0;
			}
			else if(adaptiveYaw(my_error.angle)<-1.0)
			{
				speedpid.angular.z=-1.0;
			}
			else			
				speedpid.angular.z =adaptiveYaw(my_error.angle);
			//speedpid.angular.z=adaptiveYaw(angle);
           		 //speedpid.linear.z =adaptive(e_height);
			/*if(angle>0)
			{
			  double temn1=adaptiveYaw(angle);
			  if(temn1>1.0)
			  {
				speedpid.angular.z =1.0;
			  }
		   	  else
				speedpid.angular.z=temn1;
				
			}
			else
			{
				double temn2=adaptiveYaw(angle);
				
				if(temn2<(-1.0))
			        {
				  speedpid.angular.z =(-1.0);
			        }
				else
				 speedpid.angular.z =temn2;
			}*/
			
			cout<<"无人机正在转弯"<<endl;
			
		}
	//}
	
	
	cout << "线速度 x= ：" << speedpid.linear.x << endl;
	cout << "线速度 y= ：" << speedpid.linear.y << endl;
	cout << "```````````````````````角速度 ：            " << speedpid.angular.z << endl;

	return speedpid;
	
}

//图像回调函数
void video_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& infomsg)
{
	ros::NodeHandle video_cv;   //节点句柄
	ros::Publisher cmd_vel_pub; //移动话题发布者
	//初始化 移动话题发布者
	cmd_vel_pub = video_cv.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);

	//开始巡线

	 cv_bridge::CvImagePtr cv_ptr; 
	 geometry_msgs::Twist speedCruise;
	// geometry_msgs::Twist speedZero;
	// float s = 0.13; //乘积因子
	

	//转换成BGR格式的原始图像并显示
	try
	{
		//转化成CVImage    
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		image_row = cv_ptr->image;
		waitKey(5);  //必须要加这个函数，否则图像窗口显示不了
		//imshow("原始图像", image_row);

	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception is %s", e.what());
		return;
	}

	//图像进行矫正
	//picturezt=image_row;
	image_jz = jiaozheng(image_row);
	//图片进行旋转
	imgrotate(image_jz,image_rotate,-36);
	//图像进行剪切
	image_cut=cut(image_rotate);
	imshow("completed_cut",image_cut);
	
	//图像检测函数，计算方向弧度和距离偏差
	PathDetection_normal(image_cut);
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
		if(times==150)
		{
			call_vf =false;
		}
	}
	
}



//直线飞行，检测路径
void flight_test_main()
{	
	 ros::NodeHandle video_;   //节点句柄
	
	
	 image_transport::ImageTransport image_transport(video_);//视频发布类节点
	 image_transport::CameraSubscriber image_sub;//视频话题接收者
	 //初始化 视频话题接收者
	 image_sub = image_transport.subscribeCamera("bebop/image_raw", 1 ,video_callback);
	

	 ros::Subscriber rh_navdata_sub;//导航话题接收者
		//初始化 导航数据话题接收者
	 rh_navdata_sub=video_.subscribe<ardrone_autonomy::Navdata>("bebop/navdata",1,callback);
	 
	
	 ros::Publisher move_pub,camera_move; //移动话题发布者
	 //初始化 移动话题发布者
	 move_pub = video_.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);


	 camera_move = video_.advertise<geometry_msgs::Twist>("bebop/camera_control", 1);
	
	 cout<<"开始寻线"<<endl;
	

	
	
	//PID
	//PID_init(0.0);
	 
	 
	 int keys_fd;  
	// char ret[2];  
	 struct input_event t;  
	 geometry_msgs::Twist speedadjust;
	geometry_msgs::Twist  cameraadjust;
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
								 	case KEY_C:
									{
							 			cout<<"开始绕圈"<<endl;
										circle();//
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
								 case KEY_I:
								{
									cameraadjust.angular.y=50;
									cameraadjust.angular.z=0.0;
									camera_move.publish(cameraadjust);
									cameraadjust.angular.y=0.0;
									cameraadjust.angular.z=0.0;
									 cout<<"镜头向上"<<endl;
									break;
								}
								case KEY_K:
								{
									cameraadjust.angular.y=-40;
									cameraadjust.angular.z=0.0;
									camera_move.publish(cameraadjust);
									cameraadjust.angular.y=0.0;
									cameraadjust.angular.z=0.0;
									 cout<<"镜头向下"<<endl;
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
					    speedadjust.linear.x=0;
					     speedadjust.linear.y=0;
					     speedadjust.linear.z=0;
					     speedadjust.angular.z=0;
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
		ros::Rate loop_rate(15);
		while(ros::ok())
		{	
				if(call_vf)
				{ 
					ros::spinOnce();
					//imshow("completed_cut",image_cut);
				 	// Stable();
				}
			    else
				{
					land_();
				}
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
	//struct errors my_error={0.0,0.0,0.0,0.0};  //初始化误差结构体
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
	 flight_test_main();
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

