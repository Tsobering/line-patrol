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
	Drone myDrone;
	//创建巡线方法实例
	PatrolFlight myPatrolFlight;
	
	//打开键盘文件
	int keys_fd;  
	struct input_event t;  
    keys_fd = open ("/dev/input/event4", O_RDONLY);  
	if (keys_fd <= 0)  
	{  
	  	cout<<"open /dev/input/event4 device error!\n"<<endl;  
	  	return ;  
	}  
	
	 //开始手动调整
	 while (!myDrone.start_line)
	 {
		  //读取按键控制无人机
		 if (read (keys_fd, &t, sizeof (t)) == sizeof (t))  
		 {  
			 //cout<<"按键控制"<<endl;
			 if (t.type == EV_KEY)  
					if (t.value == 1)  
				   {  
					     if(t.code==KEY_ESC)
						  break;
						 switch(t.code)
						 {
									 case KEY_O:
									{
							 			cout<<"开始起飞"<<endl;
										myDrone.takeoff();//
										break;
									}
								 	case KEY_C:
									{
							 			cout<<"开始绕圈"<<endl;
										myDrone.circle();//
										break;
									}
									case KEY_L:
									{
							 		  cout<<"开始降落"<<endl;
									  myDrone.land();
									  break;
									}
									case KEY_F:
									{ 
									  myDrone.start_line=true;
							 		  myDrone.parameterInitialization();
							          cout<<"开始巡线"<<endl;
									  break;
									}
								   case KEY_LEFT:
								   {
									 myDrone.speedadjust.linear.y=0.2;
									 cout<<"左移"<<endl;
							 		 break;

								   }
								   case KEY_RIGHT:
								   {
									 myDrone.speedadjust.linear.y=-0.2;
								     cout<<"右移"<<endl;
									 break;
								   }
									case KEY_UP_UP:
								   {
									 myDrone.speedadjust.linear.x=0.2;
									 cout<<"前进"<<endl;
									 break;
								   }
								   case KEY_DOWN_DOWN:
								   {
								 	 myDrone.speedadjust.linear.x=-0.2;
									 cout<<"后退"<<endl;
									 break;
								   }
								 	case KEY_D:
								   {
									 myDrone.speedadjust.angular.z=-0.2;
									 cout<<"右转"<<endl;
									 break;
								   }
									case KEY_A:
								   {
									 myDrone.speedadjust.angular.z=0.2;
									 cout<<"左转"<<endl;
									 break;
								   }
								  case KEY_W:
								  {
									myDrone.speedadjust.linear.z=0.2;
									cout<<"上升"<<endl;
									 break;
								  }
								  case KEY_S:
								  {
									myDrone.speedadjust.linear.z=-0.2;
								 	cout<<"下降"<<endl;
									 break;
								  }
								  case KEY_H:
								  {
									 myDrone.parameterInitialization();
								 	 cout<<"稳定"<<endl;
									 break;
								  }
								 case KEY_I:
								{
									myDrone.cameraadjust.angular.y=50;
									myDrone.cameraadjust.angular.z=0.0;
									myDrone.camera_move.publish(myDrone.cameraadjust);
									myDrone.parameterInitialization();
									cout<<"镜头向上"<<endl;
									break;
								}
								case KEY_K:
								{
									myDrone.cameraadjust.angular.y=-40;
									myDrone.cameraadjust.angular.z=0.0;
									myDrone.camera_move.publish(myDrone.cameraadjust);
									myDrone.parameterInitialization();
									cout<<"镜头向下"<<endl;
									break;
								}
								   default:
										break;	 
						 }
						 myDrone.move_pub.publish(myDrone.speedadjust);
						 myDrone.parameterInitialization();
				   }  
				  else
				  {
					 myDrone.parameterInitialization();
					 myDrone.move_pub.publish(myDrone.speedadjust);
					 cout<<"无人机稳定"<<endl;
				  }
		 } 
	
	 }	
		
	//无人机进入自主巡线，超过十秒未寻到线降落
	if(myDrone.start_line)
    {
		cout<<"自主巡线"<<endl;
		ros::Rate loop_rate(15);
		while(ros::ok())
		{	
				if(myPatrolFlight.call_vf)
				{ 
					ros::spinOnce();  //执行一次回调函数队列
					
					myPatrolFlight.image_row=myDrone.imageRow; //获得原始图像	
					//PatrolFlight.image_jz = PatrolFlight.imgCorrection(PatrolFlight.image_row);//图像进行矫正
					//PatrolFlight.imgRotate(PatrolFlight.image_jz,PatrolFlight.image_rotate,-36);//图片进行旋转
					//PatrolFlight.image_cut = PatrolFlight.imgCut(PatrolFlight.image_rotate);//图像进行剪切
					imshow("原始图像", myPatrolFlight.image_row);
					
					//图像检测函数，计算方向弧度和距离偏差
					myPatrolFlight.pathDetection(myPatrolFlight.image_row);
					
					//如果检测到路径
					if (myPatrolFlight.call_vf == true)
					{	
						//运用算法配置速度
						myPatrolFlight.speedpid = myPatrolFlight.AdaptiveUnnormal();
						//发送控制指令移动无人机
						myDrone.move_pub.publish(myPatrolFlight.speedpid);
						myPatrolFlight.times=0;
					}
					else
					{
						cout << "no path have been find :没有检测到路径" << endl;
						myPatrolFlight.call_vf =true;
						if(myPatrolFlight.times==150)
						{
							myPatrolFlight.call_vf =false;
						}
					}
				}
			    else
				{
					myDrone.land();
				}
				loop_rate.sleep();
		 }
		
		close (keys_fd);	
		myDrone.land(); //降落
	}

	
	return 0;
}