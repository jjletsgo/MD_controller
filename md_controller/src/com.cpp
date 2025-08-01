#include "md_controller/com.hpp"
#include <iostream>

serial::Serial ser;

/*
typedef unsigned char  BYTE;
typedef unsigned short WORD;
typedef unsigned int   DWORD;
*/



/*
short 는 2Byte임. 2Byte = high Byte(1 Byte) + low Byte(1 Byte)
short타입의 2Byte 데이터를 받아서 high byte와 low byte로 분리하는 함수임.

IByte는 com.hpp에 정의되어있는 구조체임
*/

IByte Short2Byte(short sIn)
{
    IByte Ret;

    Ret.byLow = sIn & 0xff; 
    //sIn(2Byte)과 0xff=0000000011111111 를 비트 and 하므로 하위 1Byte가 Ret.byLow에 저장된다.
    Ret.byHigh = sIn>>8 & 0xff;
    /*sIn을 오른쪽으로 8bit만큼 shift 한거랑 0xff=0000000011111111 를 비트 and 하므로 ,
     상위 8비트가 Ret.byHIgh에 저장된다.
    */
    return Ret;
    //이렇게 반환된 Ret에는 byLow에는 하위 8비트가, byHIgh에는 상위 8비트가 존재한다.
    
}


/*
BYTE 타입의 byLow 와 byHigh를 받아서 다시 2Byte의 short 타입 데이터로 합치는 함수
*/
// Make short data from two bytes
int Byte2Short(BYTE byLow, BYTE byHigh)
{
    return (byLow | (int)byHigh<<8);
    /*byHIgh는 BYTE타입=unsigned char 이고, 이는 1Byte이다 그래서 왼쪽으로 8번 shift하면 오류난다.
    그래서 byHigh를 int(4Byte)로 형변환 해준다음에 bit shift 하는 것이다*/
}



/*
Byte 타입의 데이터 4개 받아서 int 데이터 하나로 합쳐서 반환한다.
*/
// Make long data from four bytes
int Byte2LInt(BYTE byData1, BYTE byData2, BYTE byData3, BYTE byData4)
{
    return ((int)byData1 | (int)byData2<<8 | (int)byData3<<16 | (int)byData4<<24);
}



//Initialize serial communication in ROS
int InitSerial(void)
{
    try
    {
        ser.setPort(Com.nPort); //포트 이름 설정
        ser.setBaudrate(Com.nBaudrate); //통신 속도 설정
        serial::Timeout to = serial::Timeout::simpleTimeout(1667);  //읽기 동작 시 최대 대기 시간 설정
        //1667 when baud is 57600, 0.6ms
        //2857 when baud is 115200, 0.35ms
        ser.setTimeout(to);   //읽기 동작 시 최대 대기 시간 설정                                     
        ser.open(); //실제 시리얼 포트 열고 통신 준비
    }
    catch (serial::IOException& e)
    {
        //포트 열기 실패시 오류 발생시킴
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),"Unable to open port ");
        return -1;
    }
    if(ser.isOpen())
        //포트 열기 성공시 로그 남김
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),"Serial Port initialized");
        
    else
        return -1; //포트 열기 실패시 -1 반환
    return 0; //포트 열기 성공시 0 반환
}

//for sending the data (One ID)
/*

<실제 모터드라이버(183)으로 패킷 보내는 함수>

byPID: 어떤 명령을 보낼 것인지 (속도 명령, 위치 초기화 등)
byMID: 모터드라이버 ID
id_num: 수신자 ID
nArray[]: 실제로 모터드라이버로 전송할 데이터들 (명령값, 속도, 위치 등)
*/
int PutMdData(BYTE byPID, BYTE byMID, int id_num, int nArray[])
{
    IByte iData;
    BYTE byPidDataSize, byDataSize, i, j;
    // byPidDataSize = 최종 전송할 패킷 전체 크기 (바이트 단위)
    // byDataSize = 실제 전송할 데이터 크기 (바이트 단위)
    // i,j는 루프용 변수
    static BYTE byTempDataSum;
    // 체크섬 계산을 위한 임시 누적합 변수
    
    //Com.bySndBuf는 전송할 바이트들을 담는 버퍼이다. (최대크기 26인 BYTE타입의 배열)
    for(j = 0; j <MAX_PACKET_SIZE; j++) Com.bySndBuf[j] = 0;

    Com.bySndBuf[0] = byMID; //모터드라이버의 ID (183)
    Com.bySndBuf[1] = 184; //PC의 ID (184)
    Com.bySndBuf[2] = id_num; //제어할 모터의 ID
    Com.bySndBuf[3] = byPID; //PID

    switch(byPID)
    {
        case PID_REQ_PID_DATA: // 데이터 요청 명령 ///지금 여기가 문제임!!!비어있는거 보내는듯
                byDataSize      = 1;
                byPidDataSize   = 7;
                byTempDataSum   = 0;

                Com.bySndBuf[4] = byDataSize;
                Com.bySndBuf[5] = (BYTE)nArray[0]; //실제로 모터드라이버가 전송 받은 데이터를 Byte로 형변환해서 넣는다

                for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
                //Com.bySndBuf의 모든 요소를 다 더해서 byTempDataSum에 누적한다
                Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; 
                // bySndBuf의 마지막 요소에 byTempDataSum의 2의 보수를 넣는다. 
                // 통신 패킷의 마지막 1Byte에 checksum을 넣는 것)
		   
		   
	 	// 시리얼 전송 전에 출력 (디버깅용)
	        std::cout << "[bySndBuf] = ";
	        for (int i = 0; i < byPidDataSize; ++i)
	        {
		    printf("%02X ", Com.bySndBuf[i]);
	        }
	        std::cout << std::endl;
	        
	       
    
    
                ser.write(Com.bySndBuf, byPidDataSize);
                std::cout << "시리얼로 PID193 성공적으로 명령 보냈습니다" << std::endl;
                // ser.write(전송할 Byte 배열의 시작 주소, 보낼 Byte 수) 하면 시리얼 포트로 해당 수의 Byte를
                // 전송한다

                break;
                
        case PID_POSI_RESET: //모터의 위치를 0으로 리셋
                byDataSize    = 1;
                byPidDataSize = 7;
                byTempDataSum = 0;

                Com.bySndBuf[4]  = byDataSize;
                Com.bySndBuf[5]  = nArray[0];

                for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
                Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

                ser.write(Com.bySndBuf, byPidDataSize);

                break;

        case PID_COMMAND: //CMD값에 따라 어느 명령을 수행할지가 결정된다.
        /*모터 자연정지, 모터 1,2에 대한 브레이크 명령,,,등 여러가지가 있다*/
            byDataSize    = 1;
            byPidDataSize = 7;
            byTempDataSum = 0;

            Com.bySndBuf[4]  = byDataSize;
            Com.bySndBuf[5]  = nArray[0];

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);

            break;

        // 수정됨: PID_VEL_CMD 삭제하고 PID_PNT_VEL_CMD로 대체
        case PID_PNT_VEL_CMD: //듀얼 모터 속도 제어 명령 (207번)
            byDataSize    = 7;  // 수정됨: 7바이트 데이터
            byPidDataSize = 13; // 수정됨: 전체 패킷 크기 13바이트
            byTempDataSum = 0;

            Com.bySndBuf[4]  = byDataSize;
            
            // 수정됨: PID_PNT_VEL_CMD 데이터 구조에 맞게 패킷 구성
            Com.bySndBuf[5]  = nArray[0]; // D1: ID1 Enable
            Com.bySndBuf[6]  = nArray[1]; // D2: ID1 속도 Low
            Com.bySndBuf[7]  = nArray[2]; // D3: ID1 속도 High
            Com.bySndBuf[8]  = nArray[3]; // D4: ID2 Enable
            Com.bySndBuf[9]  = nArray[4]; // D5: ID2 속도 Low
            Com.bySndBuf[10] = nArray[5]; // D6: ID2 속도 High
            Com.bySndBuf[11] = nArray[6]; // D7: 리턴 데이터 요청

            for(i = 0; i < (byPidDataSize-1); i++) byTempDataSum += Com.bySndBuf[i];
            Com.bySndBuf[byPidDataSize-1] = ~(byTempDataSum) + 1; //check sum

            ser.write(Com.bySndBuf, byPidDataSize);
            
            // 수정됨: 디버깅용 출력
            std::cout << "PID_PNT_VEL_CMD sent - Left RPM: " << Byte2Short(nArray[1], nArray[2]) 
                      << ", Right RPM: " << Byte2Short(nArray[4], nArray[5]) << std::endl;
            break;
    }
    
    return SUCCESS;
}

////save the identified serial data to defined variable according to PID NUMBER data
/*
AnalyzeReceivedData로 패킷 유효성 검증을 끝내고 완전한 값들이 저장된
Com.byRcvBuf[] (수신버퍼)에 저장된 수신 패킷의 내용을 해석하여
모터의 속도(RPM) 와 위치(Position) 데이터를 Com 구조체에 저장함.
*/
int MdReceiveProc(void) 
{
    BYTE byRcvRMID, byRcvTMID, byRcvID, byRcvPID, byRcvDataSize;

    byRcvRMID     = Com.byRcvBuf[0];
    byRcvTMID     = Com.byRcvBuf[1];
    byRcvID       = Com.byRcvBuf[2];
    byRcvPID      = Com.byRcvBuf[3];
    byRcvDataSize = Com.byRcvBuf[4];
    // printf("in!\n");
    switch(byRcvPID)
    {
        case PID_MAIN_DATA: //PID값이 193이면 실행
            //RPM값을 파싱해서 rpm에 저장
            Com.rpm  = Byte2Short(Com.byRcvBuf[5], Com.byRcvBuf[6]);

            //모터위치 값을 파싱해서 Com.position에 저장
            Com.position = Byte2LInt(Com.byRcvBuf[15], Com.byRcvBuf[16], Com.byRcvBuf[17], Com.byRcvBuf[18]);
            
            // 수정됨: 수신된 모터 ID에 따라 각 모터 구조체에 데이터 저장
            for(int i = 0; i < 2; i++) {
                if(byRcvID == Motor[i].ID) {
                    Motor[i].rpm = Com.rpm;
                    Motor[i].position = Com.position;
                    break;
                }
            }
            // printf("%d\n",Com.rpm);
            break;
    }

    return SUCCESS;
}

int AnalyzeReceivedData(BYTE byArray[], BYTE byBufNum) //Analyze the communication data
{
    static BYTE byChkSec;
    BYTE i, j;
    int count = 0;
    // printf("0 : %d , 1 : %d \n",byArray[0],byArray[1]);
    // printf("id : %d \n",byArray[2]);


    //현재까지 수신된 바이트 개수가 최대 패킷 사이즈 넘기면 실패 후 FAIL 반환
    if(Com.byPacketNum >= MAX_PACKET_SIZE) 
    {
        Com.byStep =0; //FSM의 현재 단계
        return FAIL; //실패했음을 알림
    }
    //현재까지 수신된 바이트 개수가 최대 패킷 사이즈 안넘겼으면 아래 FSM 실행
    //최대 데이터 사이즈인 23 만큼 반복
    for(j = 0; j < byBufNum; j++)
    {
        switch(Com.byStep){
            case 0:    //Put the transmitting machin id after checking the data
                //183이 모터 드라이버를,184는 PC를 의미
                if((byArray[j] == 184) || (byArray[j] == 183))
                {
                    Com.byChkSum += byArray[j];
                    Com.byRcvBuf[Com.byPacketNum++] = byArray[j]; //수신 버퍼에 RMID와 TMID를 저장
                    Com.byChkComError = 0;
                    count++;
                    if(count == 2) Com.byStep++;  //다음 상태로 전이
                    // 수신 패킷에서 RMID와 TMID (헤더)가 둘다 183,184중
                    // 하나면 FSM이 다음 상태로 전이
                }
                else
                {
                    printf("ERROR (1)\n");
                    count = 0;
                    Com.byStep      = 0;
                    Com.fgPacketOK  = 0;
                    Com.byPacketNum = 0;
                    Com.byChkComError++;

                }
                break; //헤더가 둘다 183,184중 없으면 에러 발생
            
            //j=2 일때 아래의 케이스문 첫 실행
            //ID가 1또는 2이면 다음 상태로 전이
            case 1:    //Check ID
                if(byArray[j] == 1 | byArray[j] == 2)
                {
                    Com.byChkSum += byArray[j];
                    Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
                    Com.byStep++; //다음 상태로 전이
                    Com.byChkComError = 0;
                } 
                else
                {
                    printf("ERROR (2)\n");
                    Com.byStep = 0;
                    Com.byPacketNum = 0;
                    Com.byChkComError++;
                }
                break;
             
                //j=3 일때 아래의 케이스문 첫 실행, 그때의 byPacketNum은 2이다
             case 2:    //Put the PID number into the array
                Com.byChkSum += byArray[j]; //체크섬에 PID값 누적
                Com.byRcvBuf[Com.byPacketNum++] = byArray[j]; //Com.byRcvBuf에 PID값 넣는다
                Com.byStep++; //다음 상태로 전이
                break;

                //j=4 일때 아래의 케이스문 첫 실행, 그떄의 byPacketNum은 3이다
             case 3:    //Put the DATANUM into the array
                //수신받은 패킷 내부의 '데이터 개수'를 byMaxDataNum에 저장한다
                Com.byMaxDataNum = byArray[j]; 
                //현재까지 수신된 데이터 수 = 0 으로 초기화
                Com.byDataNum = 0; 
                // 체크섬에 데이터 개수 값 누적
                Com.byChkSum += byArray[j];
                Com.byRcvBuf[Com.byPacketNum++] = byArray[j]; //수신 버퍼에 '데이터 개수'값을 넣어준다
                Com.byStep++; //다음 상태로 전이
                break;

                //j=5 일때 아래의 케이스문 첫 실행, 그때의 byPacketNum은 4이다
             case 4:    //Put the DATA into the array
                //수신 버퍼에다가 수신된 데이터들을 1 Byte 씩 넣어준다.
                Com.byRcvBuf[Com.byPacketNum++] = byArray[j]; 
                // 체크섬에 데이터를 누적한다
                Com.byChkSum += byArray[j];
                //수신된 데이터 수를 1증가시키고 최대 데이터 사이즈보다 크면 
                // 'check 5'라는 출력과 함께 상태를 초기화시키고 함수 실행을 중지한다 (일종의 오류 식별)
                if(++Com.byDataNum >= MAX_DATA_SIZE)
                {
                    printf("check 5\n");
                    Com.byStep = 0;
                    Com.byTotalRcvDataNum = 0;
                    break;
                }
                // 수신된 데이터수가 예상되는 수신 데이터 개수와 같아지면 다음 상태로 전이한다
                if(Com.byDataNum>= Com.byMaxDataNum) Com.byStep++; //다음 상태로 전이
                break;

                //j=5+byArray[4] 일때 아래의 케이스문 첫 실행, 그때의 byPacketNum은 4+byArray[4]이다
             case 5:    //Put the check sum after Checking checksum
                Com.byChkSum += byArray[j]; //체크섬에 체크섬을 누적한다
                Com.byRcvBuf[Com.byPacketNum++] = byArray[j]; //수신 버퍼의 가장 뒤에 체크섬을 붙인다
                // printf("byChkSum : %d \n", Com.byChkSum);
                if(Com.byChkSum == 0) //체크섬이 0이면 -> 정상이니 전부 초기화
                {
                    Com.fgPacketOK   = 1;
                    Com.fgComDataChk = 1;
                    Com.byDataNum    = 0;
                    Com.byMaxDataNum = 0;
                }

                Com.byStep = 0; //상태를 0으로 되돌린다
                Com.byTotalRcvDataNum = 0;
                
                break;

            default:
                printf("check default\n");

                Com.byStep = 0;
                Com.fgComComple = ON;
                break;
        }
         //매 23반복시마다 패킷 정상 수신 여부 확인. j=0과 j=마지막 바이트 일때의 if문 확인에서 의미가있다.
        if(Com.fgPacketOK)
        {

            Com.fgPacketOK   = 0; //다음번 패킷 확인을 위해 0으로 초기화
            Com.byPacketSize = 0; //다음번 패킷 확인을 위해 0으로 초기화
            Com.byPacketNum  = 0; //다음번 패킷 확인을 위해 0으로 초기화

            if(byChkSec == 0)
            {
                byChkSec = 1;
            }
            MdReceiveProc();                                 //save the identified serial data to defined variable
        }

        //패킷 조립 중 연속 오류 횟수가 10되면 오류 발생시키고 초기화
        if(Com.byChkComError == 10) //while 50ms
        {
            printf("check error\n");
    
            Com.byChkComError = 0;
            Com.byStep = 0;
            Com.byChkSum = 0;
            Com.byMaxDataNum = 0;
            Com.byDataNum = 0;
            for(i = 0; i < MAX_PACKET_SIZE; i++) Com.byRcvBuf[i] = 0;
            j = byBufNum;
        }

    }
    return SUCCESS;
}


// 시리얼 버퍼에서 데이터 읽어와서 byRcvBuf에 저장하는 함수 
// -> AnalyzeReceivedData까지 호출해서 패킷 유효성 검사까지 진행
int ReceiveDataFromController(BYTE init) //Analyze the communication data
{
    BYTE byRcvBuf[250];
    BYTE byBufNumber;
    
    byBufNumber = ser.available(); //시리얼 버퍼에 있는 데이터를 byBufNumber에 저장

    if(byBufNumber != 0) //BufNumber에 내용이 담겨있으면, 
    {
        std::cout << "시리얼 버퍼에 내용이 담겨있네요" << std::endl;
        byBufNumber = MAX_DATA_SIZE; //23
        
        ser.read(byRcvBuf, byBufNumber); //시리얼로부터 23Byte만큼 읽어와서 byRcvBuf에 저장
        // printf("read pass\n");
        // for(int k = 0; k < 23; k++) printf("%d ", byRcvBuf[k]); cout << endl;
        if(init == ON){ //init는 모터 초기화 단계인지 여부가 저장되는 플래그이다
            /*수신된 패킷의 3번째 바이트가 내 모터의 ID와 일치하는지 확인한다
            일치하다면 모터 초기화 성공으로 판단하고 Motor.INitMotor를 OFF로 설정한다*/
            
            // 수정됨: 모터드라이버 초기화 확인
            if(byRcvBuf[2] == Motor[0].ID){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor Driver ID %d Init success!", Motor[0].ID);
                Motor[0].InitMotor=OFF;
                Motor[1].InitMotor=OFF; // 듀얼 채널이므로 둘 다 OFF
            }
        }
        else{
            AnalyzeReceivedData(byRcvBuf, byBufNumber);
        } // init 상태가 아니라면(=평상시) AnalyzeReceivedData호출해서 패킷 유효성 검사하고 byRcvBuf에 저장한다.
    }
    std::cout << "시리얼 버퍼에 내용이 없네요" << std::endl;
    return 1;
}
