#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <float.h>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include "std_msgs/msg/int16_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

#define _X 0
#define _Y 1
#define _THETA 2

#define PI 3.14159265359

#define ON 1
#define OFF 0

#define RESET 0

#define FAIL 0
#define SUCCESS 1

#define Abs(a) (((a)<(0)) ? -(a):(a))

#define PID_REQ_PID_DATA 4
#define PID_TQ_OFF 5
#define PID_COMMAND 10
#define PID_POSI_RESET 13
#define PID_PNT_VEL_CMD 207  // 수정됨: PID_VEL_CMD(130) -> PID_PNT_VEL_CMD(207)로 변경
#define PID_MAIN_DATA 193

#define MAX_PACKET_SIZE 26
#define MAX_DATA_SIZE 23

#define REQUEST_PNT_MAIN_DATA 2

#define DURATION 0.0001

#define TIME_50MS 1
#define TIME_100MS 2
#define TIME_1S 20
#define TIME_5S 100

typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned int DWORD;

typedef struct {
    BYTE bySndBuf[MAX_PACKET_SIZE]; //모터 드라이버로 전송할 패킷 버퍼
    BYTE byRcvBuf[MAX_PACKET_SIZE]; //수신한 패킷을 저장할 버퍼
    BYTE byPacketSize; //패킷 전체 크기
    BYTE byPacketNum; //현재까지 수신된 바이트 개수 (조립용 인덱스)
    BYTE byIn, byStep; //byStep는 FSM 상태 번호 (0~5) 이다.
    BYTE byChkSend;
    BYTE byChkRcv;
    // 현재 통신이 Idle 상태인지 여부, 패킷 정상 수신 여부 플래그, 통신 완료 플래그
    BYTE fgInIdleLine, fgPacketOK, fgComComple;
    BYTE byTotalRcvDataNum;
    BYTE fgChk;
    //현재 누적된 체크섬 값, 데이터 섹션의 예상 바이트 수, 현재까지 수신된 데이터 수
    BYTE byChkSum, byMaxDataNum, byDataNum;
    string nPort; //연결된 시리얼 포트 이름, PC ID, 상위 제어기 ID, 실제 동작하는 모터 드라이버 ID, 리모트 ID
    int nIDPC, nIDMDUI, nIDMDT, nRMID;
    // 시리얼 통신 보레이트 (bps), 바퀴 둘레, 회전 방향 보정 플래그
    int nBaudrate, nWheelLength, fgDirSign;
    // 바퀴 직경 설정값 , 바퀴 길이 설정값, 기어비 설정값
    short sSetDia, sSetWheelLen, sSetGear;
    // 목표 선속도 (mm/s 또는 RPM), 목표 각속도 (rad/s 또는 deg/s),
    int nCmdSpeed, nCmdAngSpeed;
    //가속 단계 설정값, 감속 단계 설정값
    int nSlowstart, nSlowdown;
    float nWheelDiameter; //바퀴 직경 (mm 또는 m)
    // 현재 측정된 모터 속도 (RPM), 현재 측정된 모터 위치 (엔코더 tick)
    int rpm,position;
    BYTE byChkComError;//패킷 조립 중 연속 오류 횟수 (10 넘으면 초기화)
    BYTE fgComDataChk;//정상 데이터 수신 여부
    BYTE fgInitsetting;//모터 초기화 완료 여부 (ON이면 완료됨)
}Communication; //통신 관련 상태 저장하는 구조체
extern Communication Com;

typedef struct{
    int ID, GearRatio, InitError, poles;
    BYTE InitMotor;
    short rpm;
    long position;
    //현재 위치값 (tick), 이전 위치와의 차이 (tick), 이전 위치값 (tick), 누적된 회전량 (radian), tick → 라디안 변환 계수, PPR
    float current_tick, last_diff_tick, last_tick, last_rad, Tick2RAD, PPR;
}MotorVar; // 모터의 속도, 위치, 기어비, 극수 등 정보 보관하는 구조체
extern MotorVar Motor[2];  // 수정됨: 듀얼 모터를 위해 배열로 변경 (Motor[0]: 왼쪽, Motor[1]: 오른쪽)

typedef struct {
    BYTE byLow;
    BYTE byHigh;
}IByte;

extern IByte Short2Byte(short sIn);
extern int Byte2Short(BYTE byLow, BYTE byHigh);
extern int Byte2LInt(BYTE byData1, BYTE byData2, BYTE byData3, BYTE byData4);
extern int InitSerial(void);
extern int InitSetParam(void);
extern int PutMdData(BYTE byPID, BYTE byID, int id_num, int nArray[]);
extern int MdReceiveProc(void);
extern int ReceiveDataFromController(BYTE init);
extern int AnalyzeReceivedData(BYTE byArray[], BYTE byBufNum);
