// Automatically generated realtime application source file for STEP platforms
//
// This file is part of NRMKPlatform SDK, Windows-based development tool and SDK
// for Real-time Linux Embedded EtherCAT master controller (STEP).
//
// Copyright (C) 2013-2015 Neuromeka <http://www.neuromeka.com>

//-system-/////////////////////////////////////////////////////////////////
#ifndef __XENO__
#define __XENO__
#endif
#include <time.h>//랜덤 생성을 위해
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <string.h>		// string function definitions
#include <fcntl.h>		// File control definitions
#include <errno.h>		// Error number definitions
#include <termios.h>	// POSIX terminal control definitions
#include <time.h>		// time calls
#include <sys/ioctl.h>
#include <math.h>
#include <pthread.h>
#include <errno.h>
#include <getopt.h>
#include <sys/mman.h>
#include <rtdm/rtcan.h>
#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <netinet/in.h>
#include <sys/types.h>
#include "Poco/Event.h"
#include "NRMKSocketBase.h"
#include "imx_HRRL_tcpsocket.h"
#include<string.h>
#include<stdio.h>
#include <typeinfo>
#include "NumericalTool.h"

char buf_so[2048];
struct sockaddr_in sin_so, cli_so;
int sd, ns, nss, clientlen = sizeof(cli_so);
double data1;
double data2;
double data3;
double data4;
double data5;
double data6;
double data7;
double data_tcp[6];

//#include <Eigen/Dense>
//#include <Eigen/Geometry>

//using Eigen::MatrixXd;
//using Eigen::Vector3d;
//-xenomai-///////////////////////////////////////////////////////////////
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <native/pipe.h>
#include <rtdk.h>		//The rdtk real-time printing library
#include <native/mutex.h>
/****************************************************************************/

//ROS
//#include <ros.h>
//#include <geometry_msgs/Point.h>
//ros::NodeHandle nh;
//double px,py,pz;

//EtherCAT Master ******************************************************************
#include "ecrt.h"
#include "SystemInterface_EtherCAT_Elmo_Motion_Control_test.h"
#include "EcatDataSocket.h"
#include "EcatControlSocket.h"
#include "ServoAxis.h"
#include "NRMKiMXSerial.h"//serial통신용

//my robot tool ***************************
#include "robot_tool/leg_3dof/TrajectoryGenerator_V2.h"
#include "robot_tool/leg_3dof/SaveData.h"
#include "robot_tool/leg_3dof/RobotLeg_V2.h"
#include "robot_tool/leg_3dof/NumericalTool.h"

#define NUM_AXIS	(3)		//Modify this number to indicate the actual number of motor on the network
#ifndef PI
#define PI	(3.14159265359)
#define PI2	(6.28318530718)
#endif

#define WAKEUP_TIME		(10)	// wake up timeout before really run, in second
#define NSEC_PER_SEC 			1000000000
////////// LOGGING BUFFER ///////////////
#define MAX_BUFF_SIZE 1000
static int sampling_time = 5; // Data is sampled every 5 cycles.->1로 수정
volatile int sampling_tick = 0;
struct LOGGING_PACK {
	double Time;
	INT32 ActualPos[NUM_AXIS];
	INT32 ActualVel[NUM_AXIS];
};
unsigned int frontIdx = 0, rearIdx = 0;
LOGGING_PACK _loggingBuff[MAX_BUFF_SIZE];
/////////////////////////////////////////

// Cycle time in nanosecond




unsigned int cycle_ns = 1000000; /* 1 ms */

typedef unsigned int UINT32;
typedef int32_t INT32;
typedef int16_t INT16;
typedef uint16_t UINT16;
typedef uint8_t UINT8;
typedef int8_t INT8;

// NRMKDataSocket for plotting axes data in Data Scope
EcatDataSocket datasocket;

NumericalTool::LowPassFilter foot_force_LPF_[NUM_AXIS];
NumericalTool::SimpleMovingAverage force_SMA_[NUM_AXIS];


// EtherCAT System interface object
NRMKHelper::SystemInterface_EtherCAT_Elmo_Motion_Control_test _systemInterface_EtherCAT_Elmo_Motion_Control_test;

// NRMK socket for online commands from NRMK EtherLab Configuration Tool
NRMKHelper::EcatControlSocket<NUM_AXIS> controlsocket;

// When all slaves or drives reach OP mode,
// system_ready becomes 1.
int system_ready = 0;
// Global time (beginning from zero)
double gt = 0;

/// TO DO: This is user-code.
double sine_amp = 5000, f = 0.2, period;

int InitFlag[NUM_AXIS] = { 0, 0, 0 };

// EtherCAT Data (in pulse)
INT32 ZeroPos[NUM_AXIS] = { 0, 0, 0 };
INT32 ActualPos[NUM_AXIS] = { 0, 0, 0 };
INT32 ActualVel[NUM_AXIS] = { 0, 0, 0 };
INT16 ActualTor[NUM_AXIS] = { 0, 0, 0 };
UINT32 DataIn[NUM_AXIS] = { 0, 0, 0 };
INT8 ModeOfOperationDisplay[NUM_AXIS] = { 0, 0, 0 };
INT32 AuxiliaryPositionActualValue[NUM_AXIS] = { 0, 0, 0 };

INT32 TargetPos[NUM_AXIS] = { 0, 0, 0 };
INT32 TargetVel[NUM_AXIS] = { 0, 0, 0 };
INT16 TargetTor[NUM_AXIS] = { 0, 0, 0 };

UINT32 DataOut[NUM_AXIS] = { 0, 0, 0 };
UINT16 MaxTorque[NUM_AXIS] = { 0, 0, 0 };

///// SDO Access /////////

//////////////////////////

//-추가 변수------------------------------------------------
//const double gear_ratio=51.;
//const double Abs_RadToPulse[3]={16384./PI2,65536./PI2,65536./PI2};
//const double Abs_PulseToRad[3]={1./Abs_RadToPulse[KP], 1./Abs_RadToPulse[HP], 1./Abs_RadToPulse[HR]};
//const double Inc_RadToPulse=2048.*gear_ratio/PI2;
//const double Inc_PulseToRad=1./Inc_RadToPulse;
//const double TorToCnt[3]={1000./(gear_ratio*9.06*0.212)/0.78,1000./(gear_ratio*2.85*0.159)/0.78,1000./(gear_ratio*2.85*0.159)/0.78};
//const double CntToTor[3]={1./TorToCnt[KP],1./TorToCnt[HP],1./TorToCnt[HR]};


int run_time = 0; //시간(ms)->sytem ready되면 시작
//#define X 0
//#define Y 1
//#define Z 2
//#define KP 0
//#define HP 1
//#define HR 2
int contact_estimate = 0;

void Finalize();

//IMU관련 변수
int fd; //file description for serial port
union IMU_angle// IMU(시리얼 통신값(float)을 frame(char)로 바뀐걸 다시 float로 변경)
{
	unsigned long temp;
	float theta;
};
union IMU_angle imu_r, imu_p, imu_y;
float roll,pitch,yaw;
double IMU[3]={0};

//camera 관련 변수
union Camera_Serial_data// IMU(시리얼 통신값(float)을 frame(char)로 바뀐걸 다시 float로 변경)
{
	unsigned long temp;
	float data;
};
Camera_Serial_data pointx,pointy,pointz,normalx,normaly,normalz;
double point[3]={0};double normal[3]={0};


//Optoforce 관련 변수
char canname[10]="rtcan1";//rtcan0은 고장남
double Opto1[3],Opto2[3]={0};//실시간 업데이트 되어짐 순서대로 Opto1=>Fx1, Fy1, Fz1/ Opto1=> Fx2, Fy2, Fz2
double FT[3]={0};
double Opto1_lpf[3],Opto1_sma[3];
//rtcan
static int rtcanfd=-1;			//rt socketcan handle
static struct can_frame frame;
static struct sockaddr_can to_addr;
static struct sockaddr_can recv_addr;

//시간 관련 변수
RTIME time_now;
RTIME time_previous;

//FT센서
typedef struct can_buffer {
	/** CAN ID of the frame
	 *
	 *  See @ref CAN_xxx_FLAG "CAN ID flags" for special bits.
	 */
	can_id_t can_id;

	/** Size of the payload in bytes */
	uint8_t can_dlc;

	/** Payload data bytes */
	uint8_t data[8] __attribute__ ((aligned(8)));//기본8
} can_buffer_t;
double Fx,Fy,Fz,Mx,My,Mz=0;//실시간 업데이트 되어짐
char FT_ready=0;
double contact_force_z=0;
//안전
int stop_flag=0;
//-------------------------------------------------------
/****************************************************************************/
double LimitTor[NUM_AXIS]={1000,1500,2200};

//좌표


NRMKHelper::ServoAxis Axis[NUM_AXIS];
/****************************************************************************/

// Xenomai RT tasks
RT_TASK Elmo_Motion_Control_test_task;
RT_TASK print_task;
RT_TASK plot_task;
RT_TASK control_task;
RT_TASK Serial_task;
RT_TASK RTCANSEND_task, RTCANRECV_task;//can

RT_TASK TCPIPSEND_task, TCPIPRECV_task;

//RT_TASK SAVEDATA_task;
// For RT thread management
static int run = 1;
unsigned long fault_count = 0;
long ethercat_time = 0, worst_time = 0;
#define min_time	0
#define max_time	100000//0.0001sec=0.1ms
#define hist_step	(100)
unsigned int histdata[hist_step + 1];
unsigned int interval_size = 350;
double vd=-0.7;

// ************ robot tool ************//
RobotLeg leg;
TrajectoryGenerator trajectory;

const int size=13;
std::string files[size]={	"Data_q","Data_qdot","Data_q_d","Data_qdot_d",
						"Data_XYZ","Data_XYZdot","Data_XYZ_d","Data_XYZdot_d",
						"Data_Torque","Data_Torque_track","Data_foot_force",
						"Data_foot_force_lpf","Data_foot_force_sma",

						};//"Data_Ground_Force","Data_G_XYZ","Data_XYZddot_d"
//	std::string files[17]={
//			"Data_q","Data_qdot","Data_qddot",
//			"Data_IMU","Data_OptoG_SMA","Data_FT",
//			"Data_XYZ","Data_XYZdot","Data_XYZddot",
//			"Data_desired_XYZ","Data_desired_XYZdot","Data_desired_XYZddot",
//			"Data_Torque","Data_CalOptoG","Data_Point","Data_Normal",
//			"Data_Opto"
//	};

SaveData save_data(files,size,"/home/Test18/");
TrajectoryGenerator force_trajectory;
bool termination=false;
const double Abs_RadToPulse[NUM_AXIS]={16384./PI2,65536./PI2,65536./PI2};
const double Abs_PulseToRad[NUM_AXIS]={1./Abs_RadToPulse[0], 1./Abs_RadToPulse[1], 1./Abs_RadToPulse[2]};
const double Inc_RadToPulse[NUM_AXIS]={2048.*51./PI2,2048.*51./PI2,2048.*51./PI2};
const double Inc_PulseToRad[NUM_AXIS]={1./Inc_RadToPulse[0],1./Inc_RadToPulse[1],1./Inc_RadToPulse[2]};
const double TorToCnt[NUM_AXIS]={1000./(51.*9.06*0.212),1000./(51.*2.85*0.159),1000./(51.*2.85*0.159)};///0.78
const double RadToDeg = 180/PI;
const double DegToRad = PI/180;
void DataWrite();
void TCPdata();
// Signal handler for CTRL+C
void signal_handler(int signum);
void TCPIPsend_run(void *arg);
void TCPIPrecv_run(void *arg);
unsigned char recvdata1[26];
unsigned char recvdata2[26];
unsigned char recvdata3[26];
unsigned char recvdata4[26];
unsigned char recvdata5[26];
unsigned char recvdata6[26];

unsigned char senddata[4095];
unsigned char sendData[4095];
int initAxes() {
	/*
	 const int gearRatio[NUM_AXIS] = {0,0,0};
	 const int pulsePerRevolution[NUM_AXIS] = {0,0,0};
	 const double ratedTau[NUM_AXIS] = {0,0,0};
	 const int dirQ[NUM_AXIS] = {0,0,0};
	 const int dirTau[NUM_AXIS] = {0,0,0};
	 const int zeroPos[NUM_AXIS] = {0,0,0};
	 */


	for (int i = 0; i < NUM_AXIS; i++) {

		Axis[i].setGearRatio(1);
		Axis[i].setPulsePerRevolution(1);
		Axis[i].setRatedTau(1);

		Axis[i].setDirQ(1);
		Axis[i].setDirTau(1);

		Axis[i].setConversionConstants();

		Axis[i].setTrajPeriod(period);

		Axis[i].setTarVelInCnt(0);
		Axis[i].setTarTorInCnt(0);

		_systemInterface_EtherCAT_Elmo_Motion_Control_test.setServoOn(i, true);
	}

	return 1;
}

//int deepsocket(){
//	char buf_so[2048];
//	struct sockaddr_in sin_so, cli_so;
//	int sd, ns, clientlen = sizeof(cli_so);
//    // socket 함수의 인자로 AF_INET과 SOCK_STREAM을 지정해 소켓을 생성한다. 인터넷 방식의 TCP 소켓
//    if((sd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
//        perror("socket");
//        exit(1);
//    }
//    // 서버의 IP 주소를 지정하고, 포트 번호는 9000으로 지정해 소켓 주소 구조체를 설정한다.
//    memset((char *)&sin_so, '\0', sizeof(sin_so));
//    sin_so.sin_family  = AF_INET;
//    sin_so.sin_port    = htons(PORTNUM);
//    sin_so.sin_addr.s_addr = inet_addr("192.168.137.10");
//
//    // bind 함수로 소켓의 이름을 정하고 접속 요청을 받을 준비를 마쳤음을 알린다.
//    if(bind(sd, (struct sockaddr *)&sin_so, sizeof(sin_so))) {
//        perror("bind");
//        exit(1);
//    }
//    if(listen(sd, 5)) {
//        perror("listen");
//        exit(1);
//    }
//
//    // accept 함수로 클라이언트의 요청을 수락한다.
//    if((ns = accept(sd, (struct sockaddr *)&cli_so, (socklen_t*)&clientlen)) == -1) {
//        perror("accept");
//        exit(1);
//    }
//
//
////    // 접속한 클라이언트의 IP 주소를 읽어 메시지를 작성한다.
//    sprintf(buf_so, "Your IP address is %s", inet_ntoa(cli_so.sin_addr));
////    // send 함수로 메시지를 전송한다.
//    if (send(ns, buf_so, strlen(buf_so) + 1, 0) == -1) {
//        perror("send");
//        exit(1);
//    }
//
//    // 사용을 마친 소켓을 모두 닫는다.
//    close(ns);
////    close(sd); //socket close
//
//
//	return 0;
//}


//-------can------------------
int RTCanInit(char *ifname)
{
	int ret, s;
	struct ifreq ifr;
	//Create a socket
	ret = rt_dev_socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (ret < 0) {
		fprintf(stderr, "rt_dev_socket: %s\n", strerror(-ret));
		return -1;
	}
	s = ret;

	strncpy(ifr.ifr_name, ifname, IFNAMSIZ); //apply interface (name)
	//Issue an IOCTL.
	ret = rt_dev_ioctl(s, SIOCGIFINDEX, &ifr);
	if (ret < 0) {
		fprintf(stderr, "rt_dev_ioctl: %s\n", strerror(-ret));
		goto failure;
	}
	memset(&to_addr, 0, sizeof(to_addr));

	//setup for receiving------------------------------------------
	recv_addr.can_family = AF_CAN;
	recv_addr.can_ifindex = ifr.ifr_ifindex;
	ret = rt_dev_bind(s, (struct sockaddr *)&recv_addr,
			  sizeof(struct sockaddr_can));
	if (ret < 0) {
		fprintf(stderr, "rt_dev_bind: %s\n", strerror(-ret));
		goto failure;
	}
	//--------------------------------------------------------------

	to_addr.can_ifindex = ifr.ifr_ifindex;
	to_addr.can_family = AF_CAN;

	return s;

failure:
	ret = rt_dev_close(s);
	return -1;
}
inline int RTCANSend(int cansock, uint16_t CANID, uint8_t *buf, uint8_t len)
{
	int  ret;
	struct can_frame frame;
	if (len>8) len=8;
	frame.can_id=CANID;
	frame.can_dlc=len;
	memcpy(frame.data, buf, len);
	ret = rt_dev_sendto(cansock, (void *)&frame, sizeof(can_frame_t), 0,
	                                (struct sockaddr *)&to_addr, sizeof(to_addr));
	 if (ret < 0) {
		 fprintf(stderr, "rt_dev_send: %s\n", strerror(-ret));
		 return -1;
	}
	return ret;
}
//RTCAN sending demo task
void RTCANSend_run(void *arg)
{
	rt_task_set_periodic(NULL, TM_NOW, 1e6);
	int count = 0;
	int count_nulling = 0;
//	unsigned char FT_candata_req_LPF_BNO[8]={0x30,0x24,0x0D,0,0,0,0,0};
	unsigned char FT_candata_nulling[8]={0x30,0x81,0,0,0,0,0,0};
	unsigned char FT_candata1[8]={0x01,0x00,0,0,0,0,0,0};//Request Mx,My,Fz
	unsigned char FT_candata2[8]={0x01,0x01,0,0,0,0,0,0};//Request Fx,Fy,Mz
//	unsigned char candata[8]={0,1,2,3,4,5,6,7};
	unsigned char OP_candata1[8]={170,0,50,3,3,4,255,0};//170 0 50 3 Header, 3 speed(333hz),4 cutoff f(15hz),0->(2ms)255 nulling,checksum1
	unsigned char OP_candata1_1[8]={0,0,0,0,0,0,0,0};//checksum2
//	unsigned char OP_candata2[8]={170,0,50,3,3,4,255,0};
//	unsigned char OP_candata2_1[8]={0,0,0,0,0,0,0,0};
	frame.can_dlc=8;
	int i=0;
	int checksum=0;
	int checksum1=0;
	int checksum2=0;
	int cnt=0;
	long stick=0;

	while(1)
	{
		//------------------------Request Nulling(3, 6축)
		if(count_nulling == 0)
		{
			frame.can_id=0x101;//TX 101
			for(i=0;i<7;i++) checksum+=OP_candata1[i];
			checksum1=checksum/256;
			checksum2=checksum%256;
			OP_candata1[7]=checksum1;
			OP_candata1_1[0]=checksum2;
			RTCANSend(rtcanfd, frame.can_id, OP_candata1, frame.can_dlc);
			RTCANSend(rtcanfd, frame.can_id, OP_candata1_1, frame.can_dlc);
			count_nulling = 1;
			rt_printf("send Nulling(3axis-1)\n");
		}
		else if(count_nulling ==1)
		{
			count_nulling = 2;
			frame.can_id=0x01;
			RTCANSend(rtcanfd, frame.can_id, FT_candata_nulling, frame.can_dlc);
			rt_printf("send Nulling(6axis)\n");
		}
		else if(count_nulling ==2){
//			count_nulling=3;
//			RTCANSend(rtcanfd, frame.can_id, OP_candata1, frame.can_dlc);
//			RTCANSend(rtcanfd, frame.can_id, OP_candata1_1, frame.can_dlc);
//			rt_printf("send Nulling(3axis-1)\n");
		};
//		else if(count_nulling ==3)count_nulling=4;
//		else if(count_nulling ==4){
//			frame.can_id=0x101;//TX 101
//			for(i=0;i<7;i++) checksum+=OP_candata2[i];
//			checksum1=checksum/256;
//			checksum2=checksum%256;
//			OP_candata2[7]=checksum1;
//			OP_candata2_1[0]=checksum2;
//			RTCANSend(rtcanfd, frame.can_id, OP_candata2, frame.can_dlc);
//			RTCANSend(rtcanfd, frame.can_id, OP_candata2_1, frame.can_dlc);
//			count_nulling = 5;
//			rt_printf("send Nulling(3axis-2)\n");
//		}

		//------------------------3축 세팅
		if (++cnt == 1000)
		{
			++stick;
			cnt = 0;
			if(stick==3) {FT_ready=1;rt_printf("Sensor ready!\n");}//3초되면 can데이터 통신 시작
		}
		if(FT_ready)
		{
			if(count==0)//request Mx,My,Fz
			{
				frame.can_id=0x02;
				RTCANSend(rtcanfd, frame.can_id,FT_candata1, frame.can_dlc);
			}
			if(count==1)//request Fx,Fy,Mz
			{
				frame.can_id=0x02;
				RTCANSend(rtcanfd, frame.can_id,FT_candata2, frame.can_dlc);
			}
			count++;
			if(count==3) count=0;//즉 333hz
		}else
		{
			if (cnt == 0)
			{
				rt_printf("Sensor ready%i", stick);
				for (i = 0; i < stick; ++i)
					rt_printf(".");
				rt_printf("\n");
			}
		}
		rt_task_wait_period(NULL); //wait for next cycle
	}
}
//RTCAN receiving demo task
void RTCANRecv_run(void *arg)
{
	rt_task_set_periodic(NULL, TM_NOW, 1e5);//default le5
	struct can_frame frame;
	int ret;
	struct sockaddr_can addr;
	socklen_t addrlen = sizeof(addr);

	int recv_flag=0;
//	int f=0;
//	int count=0;
//
//	int f_c=0;
//	int BNO=0;
//	double offset[6]={0};

	float Opto1_Coe_x=20.0/8648;//Nominal capacity[N]/sensitivity@NC[count]
	float Opto1_Coe_y=20.0/9293;
	float Opto1_Coe_z_c=200.0/16338;//Nominal capacity[N]/sensitivity@NC[count],compression
	float Opto1_Coe_z_t=100.0/16338;//Nominal capacity[N]/sensitivity@NC[count],compression.tension

	float Opto2_Coe_x=20.0/8817;//Nominal capacity[N]/sensitivity@NC[count]
	float Opto2_Coe_y=20.0/9224;
	float Opto2_Coe_z_c=200.0/16777;//Nominal capacity[N]/sensitivity@NC[count],compression
	float Opto2_Coe_z_t=100.0/16777;//Nominal capacity[N]/sensitivity@NC[count],compression.tension

	while (1)
	{
		if(FT_ready)
		{
//			test_now = rt_timer_read();
//			test_time=(float)(test_now - test_previous)/ 1000000 ;
//			rt_printf("time: %f (ms)\n", test_time);
//			test_previous = test_now;
			while(1)//몇번할지 모르니까 while
			{
				ret = rt_dev_recvfrom(rtcanfd, (void *)&frame, sizeof(can_frame_t), 0,
																  (struct sockaddr *)&addr, &addrlen);
				if (ret < 0)
				{
					fprintf(stderr, "rt_dev_recv: %s\n", strerror(-ret));
					continue;
				}
				else
				{
//					if(frame.can_id==0x100&&frame.data[0] == 0xAA&&frame.data[1] == 0x07
//										&&frame.data[2] == 0x08&&frame.data[3] == 0x1C
//										&&recv_flag==0)//170 7 8 28 Header //기존 힘센서
					if(frame.can_id==0x03)//170 7 8 28 Header
					{
						recv_flag=1;
						Opto1[0]=((((int16_t)(frame.data[1]))*256 + (int16_t)(frame.data[2]))-25000)/100;
						Opto1[1]=((((int16_t)(frame.data[3]))*256 + (int16_t)(frame.data[4]))-25000)/100;
						Opto1[2]=((((int16_t)(frame.data[5]))*256 + (int16_t)(frame.data[6]))-5000)/10;
//						rt_printf("%d\n",(int16_t)(frame.data[3]));

					}
//					else if(frame.can_id==0x03 &&recv_flag==1)
//					{
//						Opto1[0]=-((int16_t)(frame.data[0]<<8 | frame.data[1]))*Opto1_Coe_x;
//						Opto1[1]=((int16_t)(frame.data[2]<<8 | frame.data[3]))*Opto1_Coe_y;
//						Opto1[2]=((int16_t)(frame.data[4]<<8 | frame.data[5]));
//						if(Opto1[2]>0) Opto1[2]*= Opto1_Coe_z_t;
//						if(Opto1[2]<0) Opto1[2]*= Opto1_Coe_z_c;
//						Opto2[0]=((int16_t)(frame.data[6]<<8 | frame.data[7]))*Opto2_Coe_x;
//						recv_flag=2;
//						//rt_printf("%d",Opto1[0]);
//					}
//					else if(frame.can_id==0x03 &&recv_flag==2)
//					{
//						Opto2[1]=((int16_t)(frame.data[0]<<8 | frame.data[1]))*Opto2_Coe_y;
//						Opto2[2]=((int16_t)(frame.data[2]<<8 | frame.data[3]));
//						if(Opto2[2]>0) Opto2[2]*= Opto2_Coe_z_t;
//						if(Opto2[2]<0) Opto2[2]*= Opto2_Coe_z_c;
//						recv_flag=3;
//					}
					else if(frame.can_id==0x03 && recv_flag==1)
					{
						Opto1[0]=((((int16_t)(frame.data[1]))*256 + (int16_t)(frame.data[2]))-25000)/100;
						Opto1[1]=((((int16_t)(frame.data[3]))*256 + (int16_t)(frame.data[4]))-25000)/100;
						Opto1[2]=((((int16_t)(frame.data[5]))*256 + (int16_t)(frame.data[6]))-5000)/10;

						recv_flag=2;
						rt_printf("ok:%f",Opto1[0]);
					}
					else if(frame.can_id==0x03 &&recv_flag==2)
					{

						recv_flag=3;
					}
					else if(frame.can_id==0x03 &&recv_flag==3)
					{
						recv_flag=4;
					}
					else if(frame.can_id==0x03 &&recv_flag==4)
					{
						recv_flag=0;
						break;
					}
					if(frame.can_id==0x41&&frame.data[0] == 0x01)
					{
						Mx= 0.01 * (int16_t)( (frame.data[2]<<8) | frame.data[1] );
						My= 0.01 * (int16_t)( (frame.data[4]<<8) | frame.data[3] );
						//wrt EE
						FT[2]=Fz= -0.1 * (int16_t)( (frame.data[6]<<8) | frame.data[5] );
					}
					if(frame.can_id==0x41&&frame.data[0] == 0x02)
					{
						FT[0]=Fx= 0.1 * (int16_t)( (frame.data[2]<<8) | frame.data[1] );
						FT[1]=Fy= 0.1 * (int16_t)( (frame.data[4]<<8) | frame.data[3] );
						Mz= 0.01 * (int16_t)( (frame.data[6]<<8) | frame.data[5] );
					}
				}
			}
		}
		rt_task_wait_period(NULL); //wait for next cycle
	}
}
//-------can------------------

void saveLogData() {
	if (datasocket.hasConnection() && sampling_tick-- == 0) //여기서 안들어가지고 지나감
			{
		//rt_printf("check2\n");
		sampling_tick = sampling_time - 1; // 'minus one' is necessary for intended operation

		if (rearIdx < MAX_BUFF_SIZE) {
			_loggingBuff[rearIdx].Time = gt;
			for (int i = 0; i < NUM_AXIS; i++) {
				_loggingBuff[rearIdx].ActualPos[i] = ActualPos[i];
				_loggingBuff[rearIdx].ActualVel[i] = ActualVel[i];
				rt_printf("\e[32;1m\t ActPos: %d,  \e[0m\n",
						_loggingBuff[rearIdx].ActualPos[i]);
			}
			rearIdx++;
		}
	}
}


int compute() {
	if (system_ready) {


		double joint_angle[3];

		double joint_angular_velocity[3];
		joint_angle[RobotLeg::KP]=(double)ActualPos[0]*Inc_PulseToRad[0];
		joint_angle[RobotLeg::HP]=(double)ActualPos[1]*Inc_PulseToRad[1];
		joint_angle[RobotLeg::HR]=(double)ActualPos[2]*Inc_PulseToRad[2];
		joint_angular_velocity[RobotLeg::KP]=(double)ActualVel[0]*Inc_PulseToRad[0];
		joint_angular_velocity[RobotLeg::HP]=(double)ActualVel[1]*Inc_PulseToRad[1];
		joint_angular_velocity[RobotLeg::HR]=(double)ActualVel[2]*Inc_PulseToRad[2];
		if(!leg.IsInitialtize()) {
			double aux_joint_angle[3];
			aux_joint_angle[RobotLeg::KP]=(double)AuxiliaryPositionActualValue[0]*Abs_PulseToRad[0];
			aux_joint_angle[RobotLeg::HP]=(double)AuxiliaryPositionActualValue[1]*Abs_PulseToRad[1];
			aux_joint_angle[RobotLeg::HR]=(double)AuxiliaryPositionActualValue[2]*Abs_PulseToRad[2];
			leg.InitializeJointAngle(joint_angle,aux_joint_angle);
		}
		leg.SetJointAngle(joint_angle,joint_angular_velocity);
//		q_[0] = leg.GetPos()[0];
//		q_[1] = leg.GetPos()[1];
//		q_[2] = leg.GetPos()[2];
        if (trajectory.GetState() == 0)
        {
        	double st[3] = {-0.05, 0.1, -0.4};
            leg.SetControlMode(RobotLeg::POSITION_CONTROL);
            trajectory.SetTrajectoryType(TrajectoryGenerator::SINUSOIDAL);
            trajectory.SetInitialPosition(leg.GetEEPos()[0], leg.GetEEPos()[1], leg.GetEEPos()[2]);
            trajectory.SetBasicParameters(st[0], st[1], st[2], 1., 0);
        }
        if (trajectory.GetState() == 1)
        {
        	double st[3] = {-0.05, 0.1, -0.33};
            leg.SetControlMode(RobotLeg::POSITION_CONTROL);
            trajectory.SetTrajectoryType(TrajectoryGenerator::SINUSOIDAL);
            trajectory.SetInitialPosition(leg.GetEEPos()[0], leg.GetEEPos()[1], leg.GetEEPos()[2]);
            trajectory.SetBasicParameters(st[0], st[1], st[2], 0.5, 0.5);
        }
        if (trajectory.GetState() == 2)
        {
        	double st[3] = {-0.05, 0.1, -0.54};
            leg.SetControlMode(RobotLeg::POSITION_CONTROL);
            trajectory.SetTrajectoryType(TrajectoryGenerator::SINUSOIDAL);
            trajectory.SetInitialPosition(leg.GetEEPos()[0], leg.GetEEPos()[1], leg.GetEEPos()[2]);
            trajectory.SetBasicParameters(st[0], st[1], st[2], 0.5, 1.5);
        }
        if (trajectory.GetState() == 3)
        {
        	double st[3] = {-0.05, 0.1, -0.4};
            leg.SetControlMode(RobotLeg::POSITION_CONTROL);
            trajectory.SetTrajectoryType(TrajectoryGenerator::SINUSOIDAL);
            trajectory.SetInitialPosition(leg.GetEEPos()[0], leg.GetEEPos()[1], leg.GetEEPos()[2]);
            trajectory.SetBasicParameters(st[0], st[1], st[2], 0.5, 0);
        }
		if (trajectory.GetState() == 4)
        {
            trajectory.Reset();
            if (vd < -0.2)
                vd += 0.2;
        }

        leg.SetEndEffectorTrajectory(trajectory);



        for (int i = 0; i < NUM_AXIS; i++)
        {
        	Opto1_lpf[i] = foot_force_LPF_[i].Filter(Opto1[i], 10);
        	Opto1_sma[i] = force_SMA_[i].Filter(Opto1[i], 10);
        }



		leg.Task_space_CTC();

		DataWrite();

		leg.incre_time(); //로봇 시간 증가
		if(leg.time()==9000){//로봇 정지  landing:27000
			TargetTor[0]=0;TargetTor[1]=0;TargetTor[2]=0;
			termination=true;
		}



		TargetTor[0]=(int)leg.GetTargetTorque(RobotLeg::KP)*TorToCnt[0];
		TargetTor[1]=(int)leg.GetTargetTorque(RobotLeg::HP)*TorToCnt[1];
		TargetTor[2]=(int)leg.GetTargetTorque(RobotLeg::HR)*TorToCnt[2];
		if(stop_flag==0){
//			if(AuxiliaryPositionActualValue[KP]<-5350||AuxiliaryPositionActualValue[KP]>-650) {TargetTor[KP] =0;stop_flag=1;}//[-110,-10]->[-1.9,-0.19]
//			if(AuxiliaryPositionActualValue[HP]<-15000||AuxiliaryPositionActualValue[HP]>15000) {TargetTor[HP] =0;stop_flag=2;}//[-110,-15]->[-1.7,-0.2]
//			if(AuxiliaryPositionActualValue[HR]<-4500||AuxiliaryPositionActualValue[HR]>10000) {TargetTor[HR] =0;stop_flag=3;}//[-98,87]->[-1.7,1.5]
			if(TargetTor[0]>LimitTor[0]){TargetTor[0] =0;stop_flag=4;} if(TargetTor[0]<-LimitTor[0]) {TargetTor[0] =0;stop_flag=4;}
			if(TargetTor[1]>LimitTor[1]){TargetTor[1] =0;stop_flag=5;} if(TargetTor[1]<-LimitTor[1]) {TargetTor[1] =0;stop_flag=5;}
			if(TargetTor[2]>LimitTor[2]){TargetTor[2] =0;stop_flag=6;} if(TargetTor[2]<-LimitTor[2]) {TargetTor[2] =0;stop_flag=6;}

		}

		contact_force_z = leg.Getforcehat()[2];
		if(contact_force_z > 20){
			contact_estimate = 1;
		}
		else if(contact_force_z < 10){
			contact_estimate = 0;//swing;
		}
		TCPdata();




		run_time++;//1ms추가

//		TargetTor[0]=0;TargetTor[1]=0;TargetTor[2]=0;

	}
	else {TargetTor[0]=0;TargetTor[1]=0;TargetTor[2]=0;}

	return 0;
}

// Elmo_Motion_Control_test_task	
void Elmo_Motion_Control_test_run(void *arg) {

	unsigned int runcount = 0;

	RTIME now, previous;

	// Synchronize EtherCAT Master (for Distributed Clock Mode)
	_systemInterface_EtherCAT_Elmo_Motion_Control_test.syncEcatMaster();

	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns); //1ms

	while (run) {
		rt_task_wait_period(NULL); //wait for next cycle

		runcount++;

		if (!run) {
			break;
		}

		previous = rt_timer_read();

		/// TO DO: read data from sensors in EtherCAT system interface
		_systemInterface_EtherCAT_Elmo_Motion_Control_test.processTxDomain();
		_systemInterface_EtherCAT_Elmo_Motion_Control_test.readBuffer(
				COE_POSITION, ActualPos);
		_systemInterface_EtherCAT_Elmo_Motion_Control_test.readBuffer(
				COE_VELOCITY, ActualVel);
		_systemInterface_EtherCAT_Elmo_Motion_Control_test.readBuffer(
				COE_TORQUE, ActualTor);
		_systemInterface_EtherCAT_Elmo_Motion_Control_test.readBuffer(DATA_IN,
				DataIn);
		_systemInterface_EtherCAT_Elmo_Motion_Control_test.readBuffer(0x60610,
				ModeOfOperationDisplay);
		_systemInterface_EtherCAT_Elmo_Motion_Control_test.readBuffer(0x20a00,
				AuxiliaryPositionActualValue);

		/// TO DO: Main computation routine...
		compute();

		/// TO DO: write data to actuators in EtherCAT system interface
		_systemInterface_EtherCAT_Elmo_Motion_Control_test.writeBuffer(
				COE_POSITION, TargetPos);
		_systemInterface_EtherCAT_Elmo_Motion_Control_test.writeBuffer(
				COE_VELOCITY, TargetVel);
		_systemInterface_EtherCAT_Elmo_Motion_Control_test.writeBuffer(
				COE_TORQUE, TargetTor);
		_systemInterface_EtherCAT_Elmo_Motion_Control_test.writeBuffer(DATA_OUT,
				DataOut);
		/*
		 _systemInterface_EtherCAT_Elmo_Motion_Control_test.writeBuffer(0x60720, MaxTorque);
		 */
		_systemInterface_EtherCAT_Elmo_Motion_Control_test.processRxDomain();

		if (system_ready)
			saveLogData();

		// For EtherCAT performance statistics
		now = rt_timer_read();
		ethercat_time = (long) now - previous;

		if (_systemInterface_EtherCAT_Elmo_Motion_Control_test.isSystemReady()
				&& (runcount > WAKEUP_TIME * (NSEC_PER_SEC / cycle_ns))) {
			system_ready = 1; //all drives have been done
			gt += period; //period=0.001 (sec)
			if (worst_time < ethercat_time)
				worst_time = ethercat_time;
			if (ethercat_time > max_time) //연산이 0.1ms보다 오래걸리면
				++fault_count;
		}

	}

}

// Console cycle
// Note: You have to use rt_printf in Xenomai RT tasks
void print_run(void *arg) {
	RTIME now, previous = 0;
	int i;
	unsigned long itime = 0, step;
	long stick = 0;
	int count = 0;
	unsigned int NumSlaves = 0, masterState = 0, slaveState = 0;
	//
	//int index=0;
	//

	rt_printf(
			"\e[31;1m \nPlease WAIT at least %i (s) until the system getting ready...\e[0m\n",
			WAKEUP_TIME);

	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100ms = 0.1s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, 1e8); // 출력은 0.1초에 한번

	//test once


	while (1) {
//		rt_printf("Inc_PulseToRad %.4lf\n",Inc_PulseToRad);
//		rt_printf("Tor_TorToCnt %.4lf\n",Tor_TorToCnt[KP]);
		if (++count == 10) {
			++stick;
			count = 0;
		}
		if (system_ready) {
			now = rt_timer_read();
			step = (unsigned long) (now - previous) / 1000000;
			itime += step;
			previous = now;

//			rt_printf("Time=%d.%d s, ", itime/1000, itime % 1000);
//			rt_printf("dt= %li, worst= %li\n", ethercat_time, worst_time);
//
//			if (_systemInterface_EtherCAT_Elmo_Motion_Control_test.getMasterStatus(NumSlaves, masterState))
//				rt_printf("Master: Online - State %i - %i slave(s)\n", masterState, NumSlaves);
//			else
//				rt_printf("Master: Offline\n");
//
//			if (_systemInterface_EtherCAT_Elmo_Motion_Control_test.getRxDomainStatus())
//				rt_printf("RxDomain: Online\n");
//			else
//				rt_printf("RxDomain: Offline\n");
//
//			if (_systemInterface_EtherCAT_Elmo_Motion_Control_test.getTxDomainStatus())
//				rt_printf("TxDomain: Online\n");
//			else
//				rt_printf("TxDomain: Offline\n");
//
			for (i = 0; i < NUM_AXIS; ++i) {
//				if (_systemInterface_EtherCAT_Elmo_Motion_Control_test.getAxisEcatStatus(i, slaveState))
//					rt_printf("\e[32;1mSlave: Online %i,  \e[0m\n", slaveState);
//				else
//					rt_printf("\e[32;1mSlave: Offline,  \e[0m\n");
//
//				rt_printf("\e[32;1m Status: 0x%x,  \e[0m\n",_systemInterface_EtherCAT_Elmo_Motion_Control_test.getAxisCoEStatus(i));
				//
				//rt_printf("\e[32;1m\t ZeroPos: %i,  \e[0m\n",ZeroPos[i]);
				//
				//rt_printf("\e[32;1m\t ActPos: %d,  \e[0m\n",_loggingBuff[index].ActualPos[i]);
				//index++;
//				rt_printf("\e[32;1m\t ActPos: %f,  \e[0m", (double)ActualPos[i]*Inc_PulseToRad[i]*RadToDeg);
//				rt_printf("\e[32;1m\t ActPos: %f,  \e[0m\n", Data[data_idx][i]);
				//rt_printf("\e[32;1m\t ActVel: %f,  \e[0m", (double)ActualVel[i]);
				//rt_printf("\e[32;1m\t IncAcc: %f,  \e[0m",LPFVelocity[i]*Inc_PulseTODeg);
				//rt_printf("\e[32;1m\t CalVel: %f,  \e[0m\n", Velocity[i]*Inc_PulseTODeg[i]);
				//rt_printf("\e[32;1m\t IncAcc: %f,  \e[0m",Accel[i]*Inc_PulseTODeg);

				//rt_printf("\e[32;1m\t AbsAcc: %f,  \e[0m\n", (double)Abs_Acc[i]*Inc_PulseTODeg[i]);
				//rt_printf("\e[32;1m\t ActTor: %i,  \e[0m\n", ActualTor[i]);
				//rt_printf("\e[32;1m\t ModeOfOperationDisplay: %i,  \e[0m\n", 	 	ModeOfOperationDisplay[i]);
//				rt_printf("\e[32;1m\t AuxiliaryPositionActualValue: %f,  \e[0m\n", (double)AuxiliaryPositionActualValue[i]*Abs_PulseToRad[i]);
	//			rt_printf("ActualPos : %f\n",ActualPos[i]*Inc_PulseToRad*RadToDeg+InitPosError[i]);
				//				rt_printf("\n");
			}
			//밑에 묶인게 자주쓰는 print
//			rt_printf("run time: %d \n", run_time);
//			rt_printf("contact_force_z: %f \n", contact_force_z);
//			rt_printf("contact_estimate: %d \n", contact_estimate);

//			rt_printf("Tor[HR]: %f Tor[HP]: %f Tor[KP]: %f\n", leg.GetTargetTorque(RobotLeg::HR),leg.GetTargetTorque(RobotLeg::HP),leg.GetTargetTorque(RobotLeg::KP));
//			rt_printf("OG x: %f OG y:[HP]: %f OG z:[KP]: %f\n", leg.OptoG_SMA[X],leg.OptoG_SMA[Y],leg.OptoG_SMA[Z]);
//			rt_printf("Fp: %f \n",leg.F_p);
//			rt_printf("Fi: %f \n",leg.F_i);
//			rt_printf("Fd: %f \n",leg.F_d);
//			rt_printf("springF: %f \n",leg.springF);
//			rt_printf("px: %f py: %f pz: %f\n",px,py,pz);
//			std::cout<<leg.HT_G_wrt0<<std::endl;
//			rt_printf("Robo time: %d \n", leg.time());
//			rt_printf("ZETA_x: %f ZETA_y: %f ZETA_z: %f\n",ZETA(0,0),ZETA(1,0),ZETA(2,0));
//			rt_printf("Force_g_d_z: %f   opto_Force_g_z: %f\n",Force_g_d(2,0),Opto_EE_wrt_G(2,0));
//			rt_printf("F_p:%4f F_i:%4f F_d:%4f\n", leg.F_p,leg.F_i,leg.F_d);
//			rt_printf("optoG x:%4f optoG y:%4f \noptoG z: %4f\n", leg.G_F(X),leg.G_F(Y),leg.G_F(Z));
//			rt_printf("CalForce_G_z:%4f\n", leg.CalForce_G(Z));
//			std::cout<<"Cal Force G:\n"<<leg.CalForce_G<<std::endl;
//			std::cout<<"Rotate_G_wrt_0:\n"<<leg.Rotate_G_wrt_0<<std::endl;
//			rt_printf("springF: %f \n", leg.springF);
//			rt_printf("ZETAdot_x: %f ZETAdot_y: %f ZETAdot_z: %f\n",ZETAdot(0,0),ZETAdot(1,0),ZETAdot(2,0));
//			rt_printf("ZETAddot_x: %f ZETAddot_y: %f ZETAddot_z: %f\n",ZETAddot(0,0),ZETAddot(1,0),ZETAddot(2,0));
//			rt_printf("ZETA_g_x: %f ZETA_g_y: %f ZETA_g_z: %f\n",ZETA_g(0,0),ZETA_g(1,0),ZETA_g(2,0));
//			rt_printf("ZETA_g_d_x: %f ZETA_g_d_y: %f ZETA_g_d_z: %f\n",ZETA_g_d(0,0),ZETA_g_d(1,0),ZETA_g_d(2,0));
//			rt_printf("ZETAdot_g_x: %f ZETAdot_g_y: %f ZETAdot_g_z: %f\n",ZETAdot_g(0,0),ZETAdot_g(1,0),ZETAdot_g(2,0));
//			rt_printf("ZETAddot_g_x: %f ZETAddot_g_y: %f ZETAddot_g_z: %f\n",ZETAddot_g(0,0),ZETAddot_g(1,0),ZETAddot_g(2,0));
//			rt_printf("Tor_ctc_HR: %f Tor_ctc_HP: %f Tor_ctc_KP: %f\n",Tor_ctc(0,0),Tor_ctc(1,0),Tor_ctc(2,0));
//			rt_printf("Tor_M_HR: %f Tor_M_HP: %f Tor_M_KP: %f\n",Tor_M(0,0),Tor_M(1,0),Tor_M(2,0));
//			rt_printf("G1: %f G2: %f G3: %f\n",G(0,0),G(1,0),G(2,0));
//			rt_printf("V1: %f V2: %f V3: %f\n",Tor_C_G(0,0)-G(0,0),Tor_C_G(1,0)-G(1,0),Tor_C_G(2,0)-G(2,0));
//			rt_printf("run time: %d\n", run_time);
//			rt_printf("traject time: %d\n", Traj_time);
//			rt_printf("Control mode:%d\n",control_mode);
//			rt_printf("F_error: %f\n", F_error);
//			rt_printf("z:%f\n",z);
//			std::cout<<G_R<<std::endl;
//			std::cout<<EE_R<<std::endl;
//			rt_printf("Aux Pos[HR]: %d Aux Pos[HP]: %d Aux Pos[KP]: %d\n", AuxiliaryPositionActualValue[HR],AuxiliaryPositionActualValue[HP],AuxiliaryPositionActualValue[KP]);
//			rt_printf("Pos[HR]: %f(deg) Pos[HP]: %f Pos[KP]: %f\n",Pos[HR]*RadToDeg,Pos[HP]*RadToDeg,Pos[KP]*RadToDeg);
//			rt_printf("Tor[HR]: %4f  Tor[HP]: %4f  Tor[KP]: %4f\n",Torque[HR],Torque[HP],Torque[KP]);
//			rt_printf("Gra[HR]: %4f  Gra[HP]: %4f  Gra[KP]: %4f\n",Gravity_Com[HR],Gravity_Com[HP],Gravity_Com[KP]);
//			rt_printf("cnt[HR]: %5d  cnt[HP]: %5d  cnt[KP]: %5d\n",TargetTor[HR],TargetTor[HP],TargetTor[KP]);
//			rt_printf("Ke[HR]: %5f  Ke[HP]: %5f  Ke[KP]: %5f\n",Ke[HR],Ke[HP],Ke[KP]);
//			rt_printf("Tor_ratio[HR]: %5f  Tor_ratio[HP]: %5f  Tor_ratio[KP]: %5f\n",Tor_ratio[HR],Tor_ratio[HP],Tor_ratio[KP]);
//			rt_printf("Force_d_x: %5f  Force_d_y: %5f  Force_d_z: %5f\n",Force_d(0,0),Force_d(1,0),Force_d(2,0));
			if(stop_flag!=0) {rt_printf("!!!!!stop_flag!!!!!: %d\n",stop_flag);}
//			double *pos;pos=leg.GetPos();
//			rt_printf("%-5.5f %-5.5f %-5.5f\n", pos[HR],pos[HP],pos[KP]);
//			rt_printf("G F: %-5.5f %-5.5f %-5.5f\n", FT[0],FT[1],FT[2]);
//			rt_printf("Fz: %-5.5f\n", FT[2]);

// 			rt_printf("%-10f%-10f%-10f\n", theta_x.theta,theta_y.theta,theta_z.theta);
//			rt_printf("roll:%5.5f  pitch:%5.5f  yaw:%5.5f\n",roll,pitch,yaw);
//			rt_printf("pg_x:%5.5f  pg_y:%5.5f  pg_z:%5.5f\n",point_g.x,point_g.y,point_g.z);
//			std::cout<<EE_wrt_G_R<<std::endl;
			rt_printf("foot_x:%lf foot_y:%lf foot_z:%lf \n",Opto1[0],Opto1[1],Opto1[2]);//발끝 힘
			//std::cout<<Opto_EE_wrt_G<<std::endl;
//			rt_printf("Fx:%5.3f Fy:%5.3f Fz:%5.3f\n",Fx,Fy,Fz);
			//--------------------


//			rt_printf("Mx:%5.5f  My:%5.5f  Mz:%5.5f\n",Mx,My,Mz);
//			rt_printf("Target Pos: %d Target Tor: %d\n",TargetPos[HP],TargetTor[HP]);


			rt_printf("\n");

		} else {
			if (count == 0) {
				rt_printf("%i", stick);
				for (i = 0; i < stick; ++i)
					rt_printf(".");
				rt_printf("\n");
			}
		}

		rt_task_wait_period(NULL); //wait for next cycle

	}
}

// Command process cycle
void control_run(void *arg) {
	/*
	 * Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 1 s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, 1e7); // period = 1 (msec)//->10ms아님?

	INT8 modeOpDisp[NUM_AXIS] = { 0, 0, 0 };
	UINT16 status[NUM_AXIS] = { 0, 0, 0 };
	INT32 actPos[NUM_AXIS] = { 0, 0, 0 };
	INT32 actVel[NUM_AXIS] = { 0, 0, 0 };
	INT16 actTor[NUM_AXIS] = { 0, 0, 0 };

	INT8 modeOp[NUM_AXIS] = { 0, 0, 0 };
	float tarval[NUM_AXIS] = { 0, 0, 0 };
	float maxvel[NUM_AXIS] = { 0, 0, 0 };
	float maxacc[NUM_AXIS] = { 0, 0, 0 };
	float maxjerk[NUM_AXIS] = { 0, 0, 0 };

	while (1) {
		if (controlsocket.hasConnection()) {
			for (int i = 0; i < NUM_AXIS; i++) {
				modeOpDisp[i] =
						_systemInterface_EtherCAT_Elmo_Motion_Control_test.getModeOperation(
								i);
				status[i] =
						_systemInterface_EtherCAT_Elmo_Motion_Control_test.getAxisCoEStatus(
								i);
				actVel[i] = ActualVel[i];
				actTor[i] = ActualTor[i];
				actPos[i] = ActualPos[i];
			}

			controlsocket.sendMotionData(modeOpDisp, status, actPos, actVel,
					actTor);

			if (controlsocket.getMotionData(modeOp, tarval, maxvel, maxacc,
					maxjerk) != 0) {
				for (int index = 0; index < NUM_AXIS; index++) {
					_systemInterface_EtherCAT_Elmo_Motion_Control_test.setModeOperation(
							index, modeOp[index]);

					switch (modeOp[index]) {
					case OP_MODE_CYCLIC_SYNC_POSITION:
						Axis[index].setTrajBoundaryCond((double) maxvel[index],
								(double) maxacc[index]);
						Axis[index].setTarPosInCnt(tarval[index]);
						break;

					case OP_MODE_CYCLIC_SYNC_VELOCITY:
						Axis[index].resetTraj();
						Axis[index].setTarVelInCnt((INT32) tarval[index]);
						break;

					case OP_MODE_CYCLIC_SYNC_TORQUE:
						Axis[index].resetTraj();
						Axis[index].setTarTorInCnt((INT16) tarval[index]);
						break;

					default:
						Axis[index].setDataOut((INT32) tarval[index]);
						break;
					}
				}
			}
		}

		rt_task_wait_period(NULL);
	}
}

void plot_run(void *arg) {
	/*
	 * Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100 ms)
	 */
	//rt_task_set_periodic(NULL, TM_NOW, 1e8);	// period = 100 (msec)
	while (1) {
		/// TO DO: You have to prepare data for NRMKDataSocket
		if (datasocket.hasConnection() && system_ready) {
			if (frontIdx < rearIdx) {
				datasocket.updateControlData(_loggingBuff[frontIdx].ActualPos,
						_loggingBuff[frontIdx].ActualVel);
				datasocket.update(_loggingBuff[frontIdx].Time);

				frontIdx++;
			} else if (rearIdx == MAX_BUFF_SIZE) {
				frontIdx = rearIdx = 0;
			}
		} else {
			frontIdx = rearIdx = 0;
			usleep(1);
		}
		//rt_task_wait_period(NULL);
	}
}
//IMU rs232 read 스레드


//void messageCb( const geometry_msgs::Point& msgs){
//	px=msgs.x;py=msgs.y;pz=msgs.z;
//}
//ros::Subscriber<geometry_msgs::Point> sub("point_io", &messageCb );
void TCPIPsend_run(void *arg) {
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns); //원본 : 1e7 10ms 2e7 20ms 50Hz


	#define PORTNUM 9001
	char buf_so[200];
	struct sockaddr_in sin_so, cli_so;
	int sd, ns, clientlen = sizeof(cli_so);
	int from_client;
	int bytes;
	int i;
    // socket 함수의 인자로 AF_INET과 SOCK_STREAM을 지정해 소켓을 생성한다. 인터넷 방식의 TCP 소켓
    if((sd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("socket");
        exit(1);
    }
    // 서버의 IP 주소를 지정하고, 포트 번호는 9000으로 지정해 소켓 주소 구조체를 설정한다.
    memset((char *)&sin_so, '\0', sizeof(sin_so));
    sin_so.sin_family  = AF_INET;
    sin_so.sin_port    = htons(PORTNUM);
    sin_so.sin_addr.s_addr = inet_addr("192.168.137.10");

    // bind 함수로 소켓의 이름을 정하고 접속 요청을 받을 준비를 마쳤음을 알린다.
    if(bind(sd, (struct sockaddr *)&sin_so, sizeof(sin_so))) {
        perror("bind");
        exit(1);
    }
    if(listen(sd, 5)) {
        perror("listen");
        exit(1);
    }
	// accept 함수로 클라이언트의 요청을 수락한다.

	int j;

	while (1) {
		if(((ns = accept(sd, (struct sockaddr *)&cli_so, (socklen_t*)&clientlen)) == -1)&& (system_ready)) {
			perror("accept");
			exit(1);
		}


		TCPdata();

	    j = sprintf(buf_so, "%f:", data1);
	    j += sprintf(buf_so+j, "%f:", data2);
	    j += sprintf(buf_so+j, "%f:", data3);
	    j += sprintf(buf_so+j, "%f:", data4);
	    j += sprintf(buf_so+j, "%f:", data5);
	    j += sprintf(buf_so+j, "%f:", data6);
	    j += sprintf(buf_so+j, "%f:", data7);


//	    printf("%s",buf_so);



		if (send(ns, buf_so, strlen(buf_so) + 1, 0) == -1) {
			perror("send");
			exit(1);
		}
		close(ns);
//
//
//		if(((ns = accept(sd, (struct sockaddr *)&cli_so, (socklen_t*)&clientlen)) == -1)&& (system_ready)) {
//			perror("accept");
//			exit(1);
//		}
////		usleep(100);
//	    bytes = 0;
//	    for(i=0; i< sizeof(from_client); i+= bytes){
//	        if ((bytes = recv(ns, &from_client + i, sizeof(from_client) - i, 0)) == -1) {
//				perror("recv");
//				exit(1);
//	        }
//	    }
//	    printf(" from_client size : %d, Contents : %d\n", sizeof(from_client), from_client);
//	    close(ns);



		rt_task_wait_period(NULL);
	}
}
void TCPIPrecv_run(void *arg) {
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns); //원본 : 1e7 10ms 2e7 20ms 50Hz

	int recvlen = 0;
	unsigned char recvMData[4095] = { 0 };

	while (1) {


		rt_task_wait_period(NULL);
	}
}



void Serial_run(void *arg)
{
	rt_task_set_periodic(NULL, TM_NOW, 1e6);//0.001초=1000hz
//	nh.initNode();
//	nh.subscribe(sub);
	fd = imx_open_serial_port(COM2, B115200);
	if (fd<=0) rt_printf("IMU serial fail");
	//if (fd<=0) rt_task_delete(&IMU_task);
	int nbytes;
	char recv_flag = 0;
	unsigned char comBuf[2048]; //2^11개
	unsigned char recv_data[30];
	int recv_index = 0;


	while (1)
	{
		if(system_ready)
		{
			nbytes = read(fd, comBuf, 2047);
			int i = 0;
			//IMU읽기
			if (nbytes > 0) {
				for (i = 0; i < nbytes; i++) {
					if (comBuf[i] == 0x75 && recv_flag == 0) {
						recv_data[recv_index] = comBuf[i]; //=0x75
						recv_index = 1;
						recv_flag = 1;
						//printf("%x\n",comBuf[i]);
					} else if (comBuf[i] == 0x65 && recv_flag == 1) {
						recv_data[recv_index] = comBuf[i]; //=0x65
						recv_index = 2;
						recv_flag = 2;
						//printf("%x\n", comBuf[i]);
					} else if (recv_flag == 2) {
						recv_data[recv_index] = comBuf[i]; //3번째 칸부터
						recv_index++; //6부터 진짜 데이터~
						//printf("%x\n", comBuf[i]);
					}

					if (recv_index >= 20 && recv_flag != 0) {
						recv_flag = 0;
						recv_index = 0;
						imu_r.temp = (recv_data[6] & 0xFF) << 24;
						imu_r.temp |= (recv_data[7] & 0xFF) << 16;
						imu_r.temp |= (recv_data[8] & 0xFF) << 8;
						imu_r.temp |= (recv_data[9] & 0xFF);

						imu_p.temp = (recv_data[10] & 0xFF) << 24;
						imu_p.temp |= (recv_data[11] & 0xFF) << 16;
						imu_p.temp |= (recv_data[12] & 0xFF) << 8;
						imu_p.temp |= (recv_data[13] & 0xFF);

						imu_y.temp = (recv_data[14] & 0xFF) << 24;
						imu_y.temp |= (recv_data[15] & 0xFF) << 16;
						imu_y.temp |= (recv_data[16] & 0xFF) << 8;
						imu_y.temp |= (recv_data[17] & 0xFF);

						//printf("%-10f%-10f%-10f\n", theta_x.theta,theta_y.theta,theta_z.theta);
						//printf("\n");
						IMU[0]=roll=imu_r.theta;
						IMU[1]=pitch=imu_p.theta;
						IMU[2]=yaw=imu_y.theta;

					}
				}

			}

			//depth camera 읽기
//			if (nbytes > 0) {
//				for (i = 0; i < nbytes; i++) {
//					if (comBuf[i] == 0x02 && recv_flag == 0) {//0x23==#
//						recv_flag =1;//데이터 수신 시작
//					}
//					else if (comBuf[i] != 0x03 &&recv_flag == 1) {
//						recv_data[recv_index] = comBuf[i];
//						recv_index++;
//					}
//					else if (comBuf[i] == 0x03)recv_flag =2;//데이터 수신 완료
//
//					if (recv_flag==2) {
////						strncpy(serial_data,recv_data,recv_index);
//						recv_flag = 0;
//						recv_index = 0;
//
//						pointx.temp = (recv_data[0]  )<< 24;
//						pointx.temp |= (recv_data[1]  )<< 16;
//						pointx.temp |= (recv_data[2]  )<< 8;
//						pointx.temp |= (recv_data[3] );
//
//						pointy.temp = (recv_data[4]) << 24;
//						pointy.temp |= (recv_data[5] )<< 16;
//						pointy.temp |= (recv_data[6])<< 8;
//						pointy.temp |= (recv_data[7]);
//
//						pointz.temp = (recv_data[8] )<< 24;
//						pointz.temp |=( recv_data[9] )<< 16;
//						pointz.temp |= (recv_data[10])<< 8;
//						pointz.temp |= (recv_data[11]);
//
//						normalx.temp = (recv_data[12]  )<< 24;
//						normalx.temp |= (recv_data[13]  )<< 16;
//						normalx.temp |= (recv_data[14]  )<< 8;
//						normalx.temp |= (recv_data[15] );
//
//						normaly.temp = (recv_data[16]) << 24;
//						normaly.temp |= (recv_data[17] )<< 16;
//						normaly.temp |= (recv_data[18])<< 8;
//						normaly.temp |= (recv_data[19]);
//
//						normalz.temp = (recv_data[20] )<< 24;
//						normalz.temp |=( recv_data[21] )<< 16;
//						normalz.temp |= (recv_data[22])<< 8;
//						normalz.temp |= (recv_data[23]);
//
//						point[0]=pointx.data;
//						point[1]=pointy.data;
//						point[2]=pointz.data;
//						normal[0]=normalx.data;
//						normal[1]=normaly.data;
//						normal[2]=normalz.data;
//
//
//					}
//				}
//			}
//------------------------------

		}
		rt_task_wait_period(NULL);
	}

}
/****************************************************************************/
void signal_handler(int signum = 0) {
	close(fd);
	rt_task_delete(&plot_task);
	rt_task_delete(&control_task);
	rt_task_delete(&Elmo_Motion_Control_test_task);
	usleep(100);
	rt_task_delete(&print_task);
	rt_task_delete(&Serial_task);
	int ret;
	rt_task_delete(&RTCANSEND_task);
	rt_task_delete(&RTCANRECV_task);
//	rt_task_delete(&SAVEDATA_task);

	if (rtcanfd >= 0)
	{
		ret = rt_dev_close(rtcanfd);
		rtcanfd = -1;
		if (ret) {
			fprintf(stderr, "rt_dev_close: %s\n", strerror(-ret));
		}
		exit(EXIT_SUCCESS);
	}
	printf("Servo drives Stopped!\n");

	_systemInterface_EtherCAT_Elmo_Motion_Control_test.deinit();
	exit(1);
}

/****************************************************************************/
int main(int argc, char **argv) {

	srand(time(NULL));//랜덤 생성을 위해
	// Perform auto-init of rt_print buffers if the task doesn't do so
	rt_print_auto_init(1);

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	//serial통신에서 추가
	setvbuf(stdout, NULL, _IONBF, 0); //force stdout to be not buffered

	//rt can에서 추가
	if (argc>1) strcpy(canname, argv[1]);
	rtcanfd=RTCanInit(canname);
	if (rtcanfd<0) //Initialize rtsocketcan
		return 1;

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT | MCL_FUTURE);

	// TO DO: Specify the cycle period (cycle_ns) here, or use default value
	cycle_ns = 1000000; // nanosecond => 1ms
	period = ((double) cycle_ns) / ((double) NSEC_PER_SEC); //period in second unit->10^6/10^9=0.001초

	// Set the operation mode for EtherCAT servo drives

	// For CST (cyclic synchronous torque) control
	_systemInterface_EtherCAT_Elmo_Motion_Control_test.init(OP_MODE_CYCLIC_SYNC_TORQUE, cycle_ns);

	// For CSP (cyclic synchronous position) control
//	_systemInterface_EtherCAT_Elmo_Motion_Control_test.init(OP_MODE_CYCLIC_SYNC_POSITION, cycle_ns);

	// For trajectory interpolation
	initAxes();




	// TO DO: Create data socket server

	datasocket.setPeriod(period);
	if (datasocket.startServer(SOCK_TCP, NRMK_PORT_DATA))
		printf("Data server started at IP of : %s on Port: %d\n",
				datasocket.getAddress(), NRMK_PORT_DATA);

	printf("Waiting for Data Scope to connect...\n");
	datasocket.waitForConnection(0);

	// TO DO: Create control socket server


	if (controlsocket.startServer(SOCK_TCP, 6868))
		printf("Control server started at IP of : %s on Port: %d\n",
				controlsocket.getAddress(), 6868);

	printf("Waiting for Control Tool to connect...\n");
	controlsocket.waitForConnection(0);

	// Elmo_Motion_Control_test_task: create and start
	printf("Now running rt task ...\n");

	rt_task_create(&Elmo_Motion_Control_test_task,
			"Elmo_Motion_Control_test_task", 0, 98, 0);
	rt_task_start(&Elmo_Motion_Control_test_task, &Elmo_Motion_Control_test_run,
			NULL);


	// printing: create and start
	rt_task_create(&print_task, "printing", 0, 99, 0);
	rt_task_start(&print_task, &print_run, NULL);

	// plotting: data socket comm
	rt_task_create(&plot_task, "plotting", 0, 80, 0);
	rt_task_start(&plot_task, &plot_run, NULL);

	// controlling: control socket
	rt_task_create(&control_task, "controlling", 0, 85, 0);
	rt_task_start(&control_task, &control_run, NULL);

	//IMU쓰레드
	rt_task_create(&Serial_task, "IMU", 0, 99, 0);
	rt_task_start(&Serial_task, &Serial_run, NULL);

	//RTCAN sending task: create and start
	rt_task_create(&RTCANSEND_task, "RT CAN Sending",0, 75, 0);
	rt_task_start(&RTCANSEND_task, &RTCANSend_run, NULL);

	//RTCAN receiving task: create and start
	rt_task_create(&RTCANRECV_task, "RT CAN Receiving",0, 80, 0);
	rt_task_start(&RTCANRECV_task, &RTCANRecv_run, NULL);

//	rt_printf("tcp/ip\n");
	rt_task_create(&TCPIPSEND_task, "TCP IP sending", 0, 93, 0);
	rt_task_start(&TCPIPSEND_task, &TCPIPsend_run, NULL);

//Ethernet receiving task: create and start
	rt_task_create(&TCPIPRECV_task, "TCP IP Receiving", 0, 96, 0);
	rt_task_start(&TCPIPRECV_task, &TCPIPrecv_run, NULL);





	//data 저장
//	rt_task_create(&SAVEDATA_task, "SAVE DATA", 0, 99, 0);
//	rt_task_start(&SAVEDATA_task, &SAVEDATA_run, NULL);


	// Must pause here
	//pause();
	while (1) {
		if(termination==true) Finalize();
		usleep(10); //10마이크로초==0.01ms==0.00001초 늦춤
	}

	// Finalize
	signal_handler();

	return 0;
}

void Finalize(){


	close(fd);
	rt_task_delete(&plot_task);
	rt_task_delete(&control_task);
	rt_task_delete(&Elmo_Motion_Control_test_task);
	usleep(100);
	rt_task_delete(&print_task);
	rt_task_delete(&Serial_task);
	rt_task_delete(&RTCANSEND_task);
	rt_task_delete(&RTCANRECV_task);
//	rt_task_delete(&SAVEDATA_task);

	printf("Servo drives Stopped!\n");
	_systemInterface_EtherCAT_Elmo_Motion_Control_test.deinit();
//	printf("saving...");
//	Data.Save(File_name);//배열에 쌓아둔 데이터 저장
	save_data.FileOut();
	printf("\ncomplete!\n");
	exit(1);
}
void DataWrite(){

    save_data.Save(leg.GetPos()[0],leg.GetPos()[1],leg.GetPos()[2]);
    save_data.Save(leg.GetVel()[0],leg.GetVel()[1],leg.GetVel()[2]);
    save_data.Save(leg.GetDesPos()[0],leg.GetDesPos()[1],leg.GetDesPos()[2]);
    save_data.Save(leg.GetDesVel()[0],leg.GetDesVel()[1],leg.GetDesVel()[2]);

    save_data.Save(leg.GetEEPos()[0],leg.GetEEPos()[1],leg.GetEEPos()[2]);
    save_data.Save(leg.GetEEVel()[0],leg.GetEEVel()[1],leg.GetEEVel()[2]);
    save_data.Save(leg.GetDesEEPos()[0],leg.GetDesEEPos()[1],leg.GetDesEEPos()[2]);
    save_data.Save(leg.GetDesEEVel()[0],leg.GetDesEEVel()[1],leg.GetDesEEVel()[2]);

    save_data.Save(leg.GetTargetTorque()[0],leg.GetTargetTorque()[1],leg.GetTargetTorque()[2]);
    save_data.Save(leg.GetTorTracking()[0],leg.GetTorTracking()[1],leg.GetTorTracking()[2]);
    save_data.Save(Opto1[0],Opto1[1],Opto1[2]);
    save_data.Save(Opto1_lpf[0],Opto1_lpf[1],Opto1_lpf[2]);
    save_data.Save(Opto1_sma[0],Opto1_sma[1],Opto1_sma[2]);

//    save_data.Save(leg.GetForce()[0],leg.GetForce()[1],leg.GetForce()[2]);

//    save_data.Save(FT);
//    save_data.Save(leg.GetEEPosG()[0],leg.GetEEPosG()[1],leg.GetEEPosG()[2]);
//    save_data.Save(leg.GetDesEEAcc()[0],leg.GetDesEEAcc()[1],leg.GetDesEEAcc()[2]);


	//데이터 내보내기, 순서 중요함, file name 순서로
//	Data.DataWrite(leg.q);
//	Data.DataWrite(leg.qdot);
//	Data.DataWrite(leg.qddot);
//	Data.DataWrite(IMU);
//	Data.DataWrite(leg.OptoG_SMA);
//
//	Data.DataWrite(FT);
//	Data.DataWrite(leg.XYZ);
//	Data.DataWrite(leg.XYZdot);
//	Data.DataWrite(leg.XYZddot);
//	Data.DataWrite(leg.XYZ_d);
//
//	Data.DataWrite(leg.XYZdot_d);
//	Data.DataWrite(leg.XYZddot_d);
//	Data.DataWrite(leg.CalTor);
//	Data.DataWrite(leg.CalOptoG);
//
//	Data.DataWrite(leg.point_Camera);
//	Data.DataWrite(leg.normal_Camera);
//	Data.DataWrite(Opto1);
}
void TCPdata(){

	data1 = leg.GetTargetTorque()[0];
	data2 = leg.GetTargetTorque()[1];
	data3 = leg.GetTargetTorque()[2];
	data4 = leg.GetDesPos()[2] - leg.GetPos()[2];
	data5 = leg.GetDesPos()[2] - leg.GetVel()[2];
	data6 = leg.GetDesEEPos()[2] - leg.GetEEPos()[2];
	data7 = contact_estimate;



}
