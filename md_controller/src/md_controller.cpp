#include "md_controller/com.hpp" 
#include <iostream>


//전역 구조체 선언
Communication Com; 
MotorVar Motor[2];  // 수정됨: 듀얼 모터를 위해 배열로 변경

geometry_msgs::msg::TransformStamped odom_tf; //TF프레임 전송용 메시지 객체
sensor_msgs::msg::JointState joint_states; 

//ROS 메시지를 통해 받은 RPM을 저장할 변수
BYTE SendCmdRpm = OFF; //md_teleop_key_npde가 RPM명령 publish 했는지 여부 플래그
int rpm_left_ = 0;  // 수정됨: 왼쪽 모터 RPM 저장 변수
int rpm_right_ = 0; // 수정됨: 오른쪽 모터 RPM 저장 변수

// 수정됨: 왼쪽 모터 RPM 콜백 함수
void CmdRpmLeftCallBack(const std_msgs::msg::Int32::SharedPtr msg) {
    rpm_left_ = msg->data;
    SendCmdRpm = ON; //md_teleop_key_npde가 송신했는지에 대한 플래그를 ON으로 설정
}

// 수정됨: 오른쪽 모터 RPM 콜백 함수
void CmdRpmRightCallBack(const std_msgs::msg::Int32::SharedPtr msg) {
    rpm_right_ = msg->data;
    SendCmdRpm = ON; //md_teleop_key_npde가 송신했는지에 대한 플래그를 ON으로 설정
}

int main(int argc, char *argv[]) {
    
    //ROS2 "md_controller_node" 초기화
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("md_controller_node");

    // create TF
    rclcpp::Time stamp_now;
    tf2_ros::TransformBroadcaster tf_broadcaster_(node);

    // 수정됨: 듀얼 모터를 위한 좌/우 subscriber 생성
    auto rpm_left_sub = node->create_subscription<std_msgs::msg::Int32>("/cmd_rpm_left",1000, CmdRpmLeftCallBack);
    auto rpm_right_sub = node->create_subscription<std_msgs::msg::Int32>("/cmd_rpm_right",1000, CmdRpmRightCallBack);

    //Motor driver settup-------------------------------------------------------------------------------
    node->declare_parameter("MDUI", 184); //PC를 184로 세팅
    node->declare_parameter("MDT", 183); //MDT 모터드라이버를 183으로 세팅
    node->declare_parameter("Port", "/dev/ttyUSB0"); //포트 세팅
    node->declare_parameter("Baudrate", 57600); //보드레이트 57600으로 세팅
    node->declare_parameter("ID", 1); // 수정됨: 모터드라이버 ID (듀얼 채널 명령이므로 하나만 필요)
    node->declare_parameter("GearRatio", 15); // 기어비를 15로 세팅
    node->declare_parameter("poles", 10); //극수를 10으로 세팅
    node->declare_parameter("wheel_base", 0.3); // 추가됨: 바퀴 간 거리 (m)

    //위에서 파라미터 값을 선언했으니 해당 값들을 읽어와서 Com 구조체와 Motor 구조체에 저장
    node->get_parameter("MDUI", Com.nIDMDUI); 
    node->get_parameter("MDT", Com.nIDMDT);
    node->get_parameter("Port", Com.nPort);
    node->get_parameter("Baudrate", Com.nBaudrate);
    
    int motor_driver_id;
    node->get_parameter("ID", motor_driver_id);
    // 수정됨: 듀얼 채널 제어를 위해 Motor[0]에만 ID 저장 (실제로는 모터드라이버 ID)
    Motor[0].ID = motor_driver_id;
    Motor[1].ID = motor_driver_id; // 동일한 모터드라이버 사용
    
    int gear_ratio, poles;
    node->get_parameter("GearRatio", gear_ratio);
    node->get_parameter("poles", poles);
    
    // 수정됨: 양쪽 모터에 동일한 기어비와 극수 설정
    for(int i = 0; i < 2; i++) {
        Motor[i].GearRatio = gear_ratio;
        Motor[i].poles = poles;
        Motor[i].PPR       = Motor[i].poles*3*Motor[i].GearRatio;   //PPR        //poles * 3(HALL U,V,W) * gear ratio
        Motor[i].Tick2RAD  = (360.0/Motor[i].PPR)*PI / 180;   //tick → 라디안 변환 계수
    }

    double wheel_base;
    node->get_parameter("wheel_base", wheel_base);

    IByte iData; //short를 high byte와 low byte로 분리할 때 사용
    int nArray[8]; // 수정됨: PID_PNT_VEL_CMD는 7바이트 데이터 + 체크섬
    /*초기화 완료 여부, 초기화 단계 상태 변수, 통신 루프 단계 상태 변수, 50회마다 루프 처리, 초기 딜레이 카운터, byCntComStep 별 루프 카운터*/
    static BYTE fgInitsetting, byCntInitStep, byCntComStep, byCnt2500us, byCntStartDelay, byCntCase[5];
    
    byCntInitStep     = 1; //초기화 상태변수를 1로 설정
    // 수정됨: 양쪽 모터 초기화 모드 ON
    Motor[0].InitMotor   = ON; 
    Motor[1].InitMotor   = ON; 
    fgInitsetting     = OFF; // 초기 설정 완료 플래그 OFF
    // 수정됨: 양쪽 모터 초기화 오류 횟수 0으로 초기화
    Motor[0].InitError   = 0; 
    Motor[1].InitError   = 0; 
    // 수정됨: 양쪽 모터 누적 회전각과 위치 초기화
    for(int i = 0; i < 2; i++) {
        Motor[i].last_rad    = 0; 
        Motor[i].last_tick   = 0;  
    }

    InitSerial();   //com.cpp에서 위에서 파라미터로 등록한 정보로 포트 열고 시리얼 통신 초기화한다
    
    //ROS 노드 실행 중 루프 무한 반복
    //ROS2에서 /cmd_rpm 명령이 왔을 때만 제어하고, 항상 상태는 읽는 방식
    while (rclcpp::ok()) {
        // 시리얼 버퍼 확인해서 수신 여부 확인 및 패킷 유효성 검사
        // 추가로 Com 구조체의 수신 버퍼에 값들 전부 집어넣고 rpm, position도 저장한다.
        // 모터 초기화 여부 업데이트
        // 수정됨: 양쪽 모터 초기화 상태 확인
        ReceiveDataFromController(Motor[0].InitMotor || Motor[1].InitMotor); 
        if(++byCnt2500us == 50) //약 2.5ms마다 수행
        {
            byCnt2500us = 0; 
            
            if(fgInitsetting == ON) //초기화 완료된 경우
            {
                switch(++byCntComStep) //통신 루프 단계 상태 변수
                {

                //TF값 쿼터니언 형식으로 broadcast하고, 모터 위치,회전량,,,etc 업데이트 하는 단계

                case 1:{ //create tf & update motor position
                    geometry_msgs::msg::TransformStamped transformStamped; //tf관련 객체 생성
                    transformStamped.header.stamp = node->now(); //현재 시간을 타임 스탬프로 설정
                    //부모 좌표계는 world로 설정
                    transformStamped.header.frame_id = "world"; 
                    // 자식 좌표계는 motor_joint 라는 이름으로 설정
                    // 즉, 모터 조인트가 world 기준으로 어디에 있는가를 알려준다.
                    transformStamped.child_frame_id = "motor_joint";

                    //3D로 모터 조인트의 위치, 여기서는 z=0.15(바닥에서 15cm위)로 설정한다
                    transformStamped.transform.translation.x = 0.0;
                    transformStamped.transform.translation.y = 0.0;
                    transformStamped.transform.translation.z = 0.15;

                    // 수정됨: 듀얼 모터의 오도메트리 계산
                    // 양쪽 바퀴의 회전량을 평균내서 로봇의 회전각 계산
                    double avg_rad = 0;
                    for(int i = 0; i < 2; i++) {
                        //직전에 ReceiveDataFromController()에서 시리얼로 받아 Com에 저장해둔
                        // 모터 위치 tick을 Motor 구조체에 저장
                        Motor[i].current_tick     = Com.position; // TODO: 각 모터별로 position 수신 필요
                        // 이전 tick과의 차이를 구함 → 이번 루프에서 얼마나 회전했는지
                        Motor[i].last_diff_tick   = Motor[i].current_tick - Motor[i].last_tick;
                        // 현재 tick을 다음 루프에서 비교할 수 있도록 저장
                        Motor[i].last_tick        = Motor[i].current_tick;
                        //회전 차이(tick)를 라디안으로 환산하여 누적 회전량에 더함
                        Motor[i].last_rad        += Motor[i].Tick2RAD * (double)Motor[i].last_diff_tick;
                    }
                    
                    // 차동구동 로봇의 경우 양쪽 바퀴 회전량 차이로 로봇의 회전 계산
                    avg_rad = (Motor[0].last_rad + Motor[1].last_rad) / 2.0;

                    // 회전 각도를 쿼터니언으로 변환
                    tf2::Quaternion q;
                    q.setRPY(0, 0, -avg_rad);

                    //위에서 계산한 쿼터니언 형식의 회전값을 TF 메시지에 저장
                    transformStamped.transform.rotation.x = q.x();
                    transformStamped.transform.rotation.y = q.y();
                    transformStamped.transform.rotation.z = q.z();
                    transformStamped.transform.rotation.w = q.w();

                    // 실제로 TF 프레임을 발행함
                    //ROS2의 다른 노드들(tf listener)은 이걸 받아서
                    // "모터 위치가 현재 이만큼 회전했구나" 하고 계산에 씀
                    tf_broadcaster_.sendTransform(transformStamped);

                    //시간 측정용 코드
                    auto end = std::chrono::high_resolution_clock::now();
                    break;
                }

                // 모터 제어 및 모터 정보 요청하는 단계
                case 2: //Control motor & request motor info
                    if(++byCntCase[byCntComStep] == TIME_100MS) //약 100ms마다 한 번씩 실행가능
                    {
                        byCntCase[byCntComStep] = 0;
                        
                        // 다른 노드가 RPM값 publish 했는지에 대한 플래그로 명령이 들어온 적 있으면
                        // ON 되어있음
                        if(SendCmdRpm) 
                        {   
                            // 수정됨: PID_PNT_VEL_CMD (207번) 패킷 구성
                            // D1: ID1 Enable (1번 모터 활성화)
                            nArray[0] = 1;  // 수정됨: 모터 1번 Enable
                            
                            // D2,3: ID1 속도 (왼쪽 모터)
                            iData = Short2Byte(rpm_left_ * Motor[0].GearRatio); // 기어비 고려한 왼쪽 모터 RPM
                            nArray[1] = iData.byLow;
                            nArray[2] = iData.byHigh;
                            
                            // D4: ID2 Enable (2번 모터 활성화)
                            nArray[3] = 2;  // 수정됨: 모터 2번 Enable
                            
                            // D5,6: ID2 속도 (오른쪽 모터)
                            iData = Short2Byte(rpm_right_ * Motor[1].GearRatio); // 기어비 고려한 오른쪽 모터 RPM
                            nArray[4] = iData.byLow;
                            nArray[5] = iData.byHigh;
                            
                            // D7: 리턴 데이터 요청
                            nArray[6] = REQUEST_PNT_MAIN_DATA; // PID_PNT_MAIN_DATA 요청

                            // PID_PNT_VEL_CMD로 듀얼 모터 속도 제어 명령 전송
                            // 수정됨: 모터드라이버 ID로 전송
                            PutMdData(PID_PNT_VEL_CMD, Com.nIDMDT, Motor[0].ID, nArray);

                            SendCmdRpm = OFF;
                        }
                        else //모터 제어 명령 (=SendCmdRpm 플래그변수) 가 오지 않은 경우
                        {
                            //193번 PID에 대한 정보를 읽으라는 명령을 보내는 것 
                            nArray[0] = PID_MAIN_DATA; 
                            
                            // 모터 제어 명령 안 왔더라도, 모터 상태는 주기적으로 계속 읽어야 하므로 Main Data 요청은 항상 보냄
                            // 수정됨: 모터드라이버에 상태 요청
                            PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor[0].ID, nArray);  // 모터드라이버 상태 요청
                            //------------------------------------------------------------------
                        }
                        
                    }
                    byCntComStep=0; //통신 루프 상태를 다시 0으로 초기화
                    break;  
                }
            }
            else //초기화 안된 경우 -> 초기화 해야함
            {
                if(byCntStartDelay <= 200) byCntStartDelay++; 
                else //Delay 200회 (약 500ms정도) 시키고 초기화 시작
                {
                    switch (byCntInitStep)
                    {
                    case 1: //모터 연결 확인

                        //193번 PID에 대한 정보를 읽으라는 명령을 보내는 것
                        nArray[0] = PID_MAIN_DATA;
                        // 수정됨: 모터드라이버 연결 확인
                        PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor[0].ID, nArray);
                        
                        // 수정됨: 모터드라이버 초기화 확인
                        if(Motor[0].InitMotor == ON) //모터가 성공적으로 초기화됐으면 com.cpp에 의해 Motor.InitMotor이 OFF가 된다.
                            Motor[0].InitError++; //모터 초기화 안됐으면 에러 카운트 1 증가
                            
                        if(Motor[0].InitMotor == OFF)
                            byCntInitStep++; //모터드라이버 초기화 됐으면 다음 상태로 전이

                        if(Motor[0].InitError > 10){ //모터 초기화 에러 카운트가 10넘으면 에러 로그 띄우고 함수 수행 중지
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"MOTOR DRIVER ID %d INIT ERROR!!", Motor[0].ID);
                            return 0;
                        }
                    
                        break;
                    case 2: //아무 작업 안함
                        byCntInitStep++; //아무작업 안하고 다음 상태로 전이
                        break;

                    case 3: //모터에 속도0 명령 줘서 Torque ON(모터가 동작할 준비를 유도
                        // 수정됨: PID_PNT_VEL_CMD로 양쪽 모터 속도 0 명령
                        nArray[0] = 1;  // 모터 1번 Enable
                        nArray[1] = 0; 
                        nArray[2] = 0;
                        nArray[3] = 2;  // 모터 2번 Enable
                        nArray[4] = 0;
                        nArray[5] = 0;
                        nArray[6] = 0;  // 리턴 데이터 요청 없음
                        
                        PutMdData(PID_PNT_VEL_CMD, Com.nIDMDT, Motor[0].ID, nArray);

                        byCntInitStep++;
                        break;

                    case 4: //모터 위치 값을 0으로 초기화
                        nArray[0] = 0;
                        // 수정됨: 모터드라이버의 위치 초기화
                        PutMdData(PID_POSI_RESET, Com.nIDMDT, Motor[0].ID, nArray);
                        byCntInitStep++;
                        break;

                    case 5: //모터 초기화 완료 로그 남김
                        printf("========================================================\n\n");
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DUAL MOTOR INIT END\n");
                        fgInitsetting = ON; //모터 초기화 여부 플래그 ON으로 설정

                        break;

                    }
                }
            }
        }

        rclcpp::spin_some(node); //node
    }

    rclcpp::shutdown();
    return 0;
}
