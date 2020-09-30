//Discription: Robot Control only executed by P Control
//Createy by Lin
//Update: 21,Sept,2020 - update staticXForce,staticYForce,staticZForce online
//        
//Functions: #1 multi-threads to get TCPForce

#include <iostream>
#include<opencv2/opencv.hpp>
#include<algorithm>
#include <fstream>
#include<thread>

#include "windows.h"

#include "robot_data.h"
#include "robot_control.h"
#include "ur_robot_adapter.h"
#include "PIDControl.h"
//#include "fanuc_robot_adapter.h"
#include "platform/platform.h"
#include "CUR_Control.h"

#include "ImageProcessing.h"
#include "Halconcpp.h"


using namespace HalconCpp;

using namespace std;
using namespace cv;

std::mutex data_mutex;
std::condition_variable data_var;
bool flag = true;

// vector of TCPForce
vector<double> TCPForce;
double XForce = 0;
double YForce = 0;
double ZForce = 0;

// ur_control::CUR_Control URRobotObj;
hrobot::URRobotAdapter* robot_control_ptr = new hrobot::URRobotAdapter();


double GetTickCountA()
{
	__int64 Freq = 0;
	__int64 Count = 0;
	if (QueryPerformanceFrequency((LARGE_INTEGER*)&Freq)
		&& Freq > 0
		&& QueryPerformanceCounter((LARGE_INTEGER*)&Count))
	{
		//乘以1000，把秒化为毫秒
		return (double)Count / (double)Freq * 1000.0;
	}
	return 0.0;
}

void getTCPForce()
{
	vector<double> tcpForce;
	int counterNum = 5;

	double xForce;
	double yForce;
	double zForce;

	while (1)
	{
		xForce = 0;
		yForce = 0;
		zForce = 0;

		for (int i = 0; i < counterNum; i++)
		{
			tcpForce = robot_control_ptr->robot_control_.m_pUrDiver->m_pRt_Interface->m_pRobotState->getTcpForce();
			Sleep(1);
			xForce += tcpForce[0];
			yForce += tcpForce[1];
			zForce += tcpForce[2];
			cout << "Thread **************" << tcpForce[0] << " " << tcpForce[1] << " " << tcpForce[2] << endl;
		}

		std::unique_lock<std::mutex> lck(data_mutex);
		data_var.wait(lck, [] {return flag;});
		XForce = xForce / counterNum;
		YForce = yForce / counterNum;
		ZForce = zForce / counterNum;
		cout << "Average TCPForce **************" << XForce << " " << YForce << " " << ZForce << endl;
		flag = false;
		data_var.notify_one();
	}
}


int main()
{
	// static force of X,Y,Z-direction. It is highly recommended that you should remeasure the staticXForce,staticXForce and staticZForce once you 
	// restart the project, mainly because the measured 3 values are quite different.
	double staticXForce = 0;
	double staticYForce = 0;
	double staticZForce = 0;

	double XForceThreshold = 0.8;
	double YForceThreshold = 0.8;
	double ZForceThreshold = 8;

	double XForceErr = 0;
	double YForceErr = 0;
	double ZForceErr = 0;

	// The reason that P_XForce, P_YForce bigger than P_ZForce is we want the XForce and YForce control 
	double P_XForce = 0.005;
	double P_YForce = 0.005;
	double P_ZForce = 0.005;
	
	
	// Initialize Robot 
	hrobot::ToolPosition current_position;
	ur_pose::ToolPosition init_position;
	ur_pose::ToolPosition temp_speed;

	
	//robot_control_ptr->robot_contorl_ = &URRobotObj;
	if (!robot_control_ptr->StartRobot()) {
		std::cout << "failed to connected to robot!" << std::endl;
		delete robot_control_ptr;
		return 1;
	}

	Sleep(1000);
	current_position = robot_control_ptr->GetRobotTCPPosition();

	std::cout << "x = " << current_position.x_ << " "
		<< "y = " << current_position.y_ << " "
		<< "z = " << current_position.z_ << std::endl;
	std::cout << "rx = " << current_position.rx_ << " "
		<< "ry = " << current_position.ry_ << " "
		<< "rz = " << current_position.rz_ << std::endl;

	if (current_position.x_ == 0 && current_position.y_ == 0 && current_position.z_ == 0)
	{
		std::cout << "failed to connected to robot!" << std::endl;
		delete robot_control_ptr;
		return 0;
	}

	////initialize robot position
	//init_position.x = -0.120780;
	//init_position.y = 0.598140;
	//init_position.z = 0.272880;
	//init_position.rx = current_position.rx_;
	//init_position.ry = current_position.ry_;
	//init_position.rz = current_position.rz_;

	//robot_control_ptr->robot_control_.m_cArmCtrlobj.ServoJMoveToTCP(init_position, 0.3, 0.05, 200);
	//Sleep(500);

    //update staticXForce,staticYForce,staticZForce online
	for (int i = 0;i < 10; i++) {
		TCPForce = robot_control_ptr->robot_control_.m_pUrDiver->m_pRt_Interface->m_pRobotState->getTcpForce();
		staticXForce += TCPForce[0];
		staticYForce += TCPForce[1];
		staticZForce += TCPForce[2];
		cout << TCPForce[0] << "  " << TCPForce[1] << "  " << TCPForce[2] << endl;
		Sleep(2);   // UR5E update time 2s
	}
	staticXForce = staticXForce / 10;
	staticYForce = staticYForce / 10;
	staticZForce = staticZForce / 10;
	cout << "staticXForce  staticYForce  staticZForce  " << staticXForce << " " << staticYForce << " " << staticZForce << endl;

	double time1;
	double time2;

	// TH1 thread for getTCPORCE realtime
	thread TCPForceThread(getTCPForce);
	TCPForceThread.detach();
	
	while (1)
	{
		////get TCP Force
		time1 = GetTickCountA();
		
		std::unique_lock<std::mutex> lck(data_mutex);
		data_var.wait(lck, [] {return !flag;});
		XForceErr = XForce - staticXForce;
		YForceErr = YForce - staticYForce;
		ZForceErr = ZForce - staticZForce;
		flag = true;
		data_var.notify_one();
		
		cout << " XForceErr " << XForceErr << " YForceErr " << YForceErr << " ZForceErr " << ZForceErr << endl;
		//X-Direction
		if (fabs(XForceErr) > XForceThreshold)
			temp_speed.x = P_XForce * XForceErr;
		else  
			temp_speed.x = 0;

		//Y-Direction
		if (fabs(YForceErr) > YForceThreshold)
			temp_speed.y = P_YForce * YForceErr;
		else
			temp_speed.y = 0;

		//Z-Direction
		if (fabs(ZForceErr) > ZForceThreshold)
			temp_speed.z = P_ZForce * ZForceErr;
		else
			temp_speed.z = 0;

		//cout << temp_speed.x << "  " << temp_speed.y << "  " << temp_speed.z << endl;
		robot_control_ptr->robot_control_.m_cArmCtrlobj.Speedl(temp_speed, 0.1, 1);   //(temp_speed, 0.3, 1)	
		
		time2 = GetTickCountA();
		cout << "time------" << time2 - time1 << endl;
	}
	
	//robot_control_ptr->robot_control_.m_cArmCtrlobj.ServoJMoveToTCP(temp_position, 0.3, 0.05, 200);
	//robot_control_ptr->robot_control_.m_cArmCtrlobj.ToolMove(temp_position, 20, 0.5);
	//robot_control_ptr->robot_control_.m_cArmCtrlobj.ServoJMoveToTCP(temp_position, 0.1, 0.03, 300);   // basic (temp_position, 0.15, 0.03, 500)

	robot_control_ptr->StopRobot();
	delete robot_control_ptr;

	return 0;

}

