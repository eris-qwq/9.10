
#include "PID_task.h"
#include "motor.h"
#include "tim.h"        
#include "ramp.h"
#include "math.h"             
#include "VCOMCOMM.h"
#include "My_code.h"
#include "usart.h"
#include "Auto.h"
#include "vision_usb.h"
#include "mechanism_motor.h"
#include "imu_task.h"

//视觉数据包
extern Data2Pack Data2;

int16_t CAN1_txdata[4];
int16_t CAN2_txdata[4];
int16_t CAN6020[4];

/* CAN2的0x200控制帧：下标0预留，当前下标1控制伸缩M2006（0x202）。 */
int16_t Mechanism_CAN2_txdata[4];

//遥控器数据
extern Remote_Handle_t Remote_Control;

//2006目标转速结构体Chassis_Motor_expect
Chassis_Motor_expect expect_wheel_2006;
Chassis_Motor_expect expect_2006;

 //电机斜坡
 int32_t expect_wheel_ramp_2006[4]={2,2,2,2};   //电机斜坡


// int32_t expect_wheel_ramp_2006[4]={50,50,50,50};   //电机斜坡
M2006_TypeDef motor_wheel_2006[4];
GM6020_TypeDef motor_rotor;
float GM6020ExAngle,GM6020NowAngle;
extern uint8_t ScreenGo;
int32_t wheel_offset[4] = {3100,0,0,0};
int32_t M2006_offset[4];
uint8_t stage = 0,put = 0;
int16_t debug = 1100;

//条件编译，目前编译上面这个
#if SPEED_MODE
/* 底盘/GM6020/Yaw PID沿用学长参数。调参时一次只改一组并保留测试记录。 */
PID PID_wheel_speed[5]={{.Kp=3.25,.Ki=0.0125,   .Kd=1,.limit=250000},//3.25 0.0125 1 250k
						
						{.Kp=0.75,.Ki=0.01,    .Kd=1,.limit=500000},
                        
                        {.Kp=1,   .Ki=0.0075,  .Kd=1,.limit=15000},
						{.Kp=1.25,.Ki=0.0075,  .Kd=1,.limit=30000},
						{.Kp=1,   .Ki=0.0075,  .Kd=1,.limit=10000}};
#else 
PID PID_wheel_speed[4]={{.Kp=2, .Ki=0, .Kd=2,.limit=5000},
						{.Kp=2, .Ki=0, .Kd=2,.limit=5000},
                        {.Kp=2, .Ki=0, .Kd=2,.limit=5000},
						{.Kp=2, .Ki=0, .Kd=2,.limit=5000},};

#endif


PID_Smis PID_wheel_position[5]={{.Kp=2, .Ki=0.001,.Kd=-2,   .limit=5000},
								{.Kp=2, .Ki=0,	  .Kd=-2.5, .limit=5000},   
                                {.Kp=2, .Ki=0, 	  .Kd=-2.5, .limit=2000}, 
                                {.Kp=2, .Ki=0,    .Kd=-2.5, .limit=2000},
								{.Kp=2, .Ki=0,    .Kd=-2.5, .limit=2000}};


PID M2006_Speed[4]={{.Kp=2,.Ki=0,.Kd=2,.limit = 5000},
					{.Kp=2,.Ki=0,.Kd=2,.limit = 5000},
					{.Kp=2,.Ki=0,.Kd=2,.limit = 5000},
					{.Kp=2,.Ki=0,.Kd=2,.limit = 5000}};

/*
 * 航向串级环：外环只用 P 生成目标角速度，内环 PI 跟踪角速度。
 * PID_Control() 的积分没有乘 dt，因此外环禁用积分，避免在 2 ms 周期下迅速饱和。
 */
PID Yaw_SpeedPid = {.Kp=2.5, .Ki=0.0015, .Kd=0.0, .limit=100000};
PID Yaw_PositionPid = {.Kp=2.5, .Ki=0.02, .Kd=0.0, .limit=5000};
//                      2          0.0015
float Now_Yaw,Exp_Yaw;

void Updatakey(Remote_Handle_t * xx)
{ //遥控器数据更新
    xx->Second = xx->First;
    xx->First = *xx->Key_Control;
}

float RAMP_self(float final, float now, float ramp ) //斜坡函数
{
    float buffer = final - now;
    
    if (buffer > 0)
    {
        if (buffer > ramp)  
                now += ramp;  
        else
                now += buffer;
    	//反正是加小的
    }		
    else
    {
        if (buffer < -ramp)
                now += -ramp;
        else
                now += buffer;
    	//反正也是减小的
    }
    return now;
}



//一次定义了两个Data1Pack类型的全局变量
Data1Pack VisionSend = {
	.head = 0x5A,
	/* car V2需要4字节状态：颜色、区域、抓取完成、放置完成。 */
	.DataLen = 4,
	.Sequence[0] = 1,
	.Sequence[1] = 1,
	.Sequence[2] = 0,
	.Sequence[3] = 0
},Sequence = {.Sequence = {RED,GREEN,BLUE,BLUE,GREEN,RED}};//未收到二维码前的RGB测试顺序




uint8_t start = 0,once[3] = {1,1,1},YawCorr,StepMotortest,test2[4],VisionTest,PosReady = 1,PosReady1 = 1;
uint32_t Delay = 70000;
int16_t ex_x,ex_y,ex_omega,tempomega,Cal_omega,Now_Omega,omegaramp = 100,GM6020Ramp = 10,GM6020Low = 100,GM6020LowLimit = 1750;
float LastAngle[4],NowAngle1[4],NowAngle2[4];
extern uint8_t GrabPutCounter;
extern int32_t StepCounter[2];
int HeightCounter,LengthCounter,TempCCR;





/**
 * @brief 每 2 ms 采样一次 PE2 常开复位按钮，并完成约 20 ms 消抖。
 * @note  按钮只负责把 start 置为 2，从而启动原自动流程；松开后可重新待命。
 */
static void StartButton_Update(void)
{
	static uint8_t low_count;
	static uint8_t armed = 1U;

	if (HAL_GPIO_ReadPin(Control_GPIO_Port, Control_Pin) == GPIO_PIN_RESET)
	{
		if (low_count < 10U)
		{
			++low_count;
		}
		if ((low_count >= 10U) && armed)
		{
			start = 2U;
			armed = 0U;
		}
	}
	else
	{
		low_count = 0U;
		armed = 1U;
	}
}


#define __UPDATE__()  LastAngle[0] = motor_wheel_2006[0].Angle_DEG,LastAngle[1] = motor_wheel_2006[1].Angle_DEG;\
					  LastAngle[2] = motor_wheel_2006[2].Angle_DEG,LastAngle[3] = motor_wheel_2006[3].Angle_DEG;
uint32_t Distance[12] = {7500,125000,42000,47000,42000,39000,15000,115000,44000,50000,42000,83000};
/* Distance按stage调用顺序保存底盘编码器目标距离；实车轮径或减速比改变后需逐段标定。 */
// {7500,120000,42000,47000,42000,39000,15000,115000,44000,50000,42000,83000};
//Y=X前 EOMEGA = Y左右 X =EOMEGA自旋 -9000逆时针旋转90° 编码器编码 24546

//
// //自动流程Task**********************************************************************
// //自动流程Task**********************************************************************
// //自动流程Task**********************************************************************
 void Remote_deal(void* param){
 	YawCorr = 1;
 	vTaskDelay(10);
 	vTaskDelete(NULL);
// 	portTickType xLastWakeTime = xTaskGetTickCount();
// 	while(1){
// 		if(start == 2)
// 		{
// 			//走出蓝框
// 			//once[0]为1时，执行一次初始化动作，之后置为0
// 			if(once[0])
// 			{
// 				//告诉上位机：暂时不请求识别指定颜色或目标类型
// 				VisionSend.Sequence[0] = 0;
// 				VisionSend.Sequence[1] = 0;
//
// 				//开启IMU航向闭环
// 				YawCorr = 1;
//
// 				//设置运动目标
// 				ex_x = 4000;
// 				ex_y = 2500;
// 				vTaskDelay(1000);//在当前1ms系统节拍下等待约1秒
// 				ex_y = 0;
// 				vTaskDelay(1000);//在当前1ms系统节拍下等待约1秒
// 				ex_x = 0;
// 				once[0] = 0;
//
// 				//宏定义，更新当前底盘电机位置，当前位置设为后续动作的起点
// 				__UPDATE__();
// 			}
//
// 			switch(stage)
// 			{
// 				case 0:
// 				//等待摄像头扫描QR完成，ScreenGo == 0 ：未完成
// 				while (ScreenGo == 0)
// 				{
// 					vTaskDelay(1);
// 				}
//
// 				//告诉上位机现在要识别什么
// 				VisionSend.Sequence[0] = 1;//识别红色目标
// 				VisionSend.Sequence[1] = 1;//识别原料区域
//
// 				//设置小车一个方向的运动速度。
// 				ex_x = -_2006_Speed;
//
// 				//AutoMove()根据M2006编码器累计运动量判断是否到达目标位置：
// 				while(!AutoMove(motor_wheel_2006,LastAngle,52500))//41636.75  到达圆盘
// 					vTaskDelay(1);
//
// 				//到位后停止运动
// 				ex_x = 0;
//
// 				//更新位置
// 				__UPDATE__();
//
// 				//圆盘夹取
// 				for(GrabPutCounter = 3;GrabPutCounter > 0;GrabPutCounter--)
// 				{
// 					VisionSend.Sequence[0] = Sequence.Sequence[3 - GrabPutCounter];
// 					GM6020ExAngle = -1930;//@
// 					vTaskDelay(500);
// 					PosReady1 = 0;
// 					while(!PosReady1)
// 						vTaskDelay(1);
// 					AutoArmGrabDrop(4 - GrabPutCounter,0,0,0);
// 				}
//
// 				ex_x = _2006_Speed;//ex_y = 3000;
// 				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[0]))//11000  后退到中间
// 					vTaskDelay(1);
// 				Exp_Yaw = 9000;
// 				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[1]))//132500到达加工区
// 					vTaskDelay(1);
// 				ex_x = 0;
// 				Exp_Yaw = 18000;
// 				VisionSend.Sequence[1] = 2;
// 				GM6020ExAngle = -1950;
// 				vTaskDelay(1500);
// 				stage++;
// 				break;
//
//
// 				case 1:
// 				put = 1;
// 				PosReady = 0;
// 				while(!PosReady)
// 					vTaskDelay(1);
// 				ex_x = 0,ex_y = 0;
//
// 				for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){//放置物料
// 					AutoArmGrabDrop(0,0,GrabPutCounter,0);
// 					AutoArmPos(Sequence.Sequence[GrabPutCounter - 1],1);
// 					AutoArmGrabDrop(0,0,0,1);
// 					vTaskDelay(250);
// 				}
//
// 				for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){
// 					AutoArmPos(Sequence.Sequence[GrabPutCounter - 1],1);
// 					AutoArmGrabDrop(0,GrabPutCounter,0,0);
// 				}
//
// 				put = 0;
// 				stage++;
// 				break;
//
//
// 				case 2:
// 				__UPDATE__();
// 				ex_x = _2006_Speed;
//
// 				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[2]))//44000  加工区前进
// 					vTaskDelay(1);
// 				Exp_Yaw = 27000;
// 				__UPDATE__();
//
// 				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[3]))//50000  到达放置区
// 					vTaskDelay(1);
// 				GM6020ExAngle = -1950;
// 				vTaskDelay(2000);
// 				ex_x = 0;
// 				put = 1;
// 				PosReady = 0;
//
// 				while(!PosReady)
// 					vTaskDelay(1);
// 				ex_x = 0,ex_y = 0;
//
// 				for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){//放置物料
// 					AutoArmGrabDrop(0,0,GrabPutCounter,0);
// 					AutoArmPos(Sequence.Sequence[GrabPutCounter - 1],1);
// 					AutoArmGrabDrop(0,0,0,1);
// 					vTaskDelay(250);
// 				}
//
// 				stage++;
// 				break;
//
//
// 				case 3:
// 				__UPDATE__();
// 				ex_x = _2006_Speed;
//
// 				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[4]))//42000	放置区前进
// 					vTaskDelay(1);
// 				Exp_Yaw = 36000;
// 				VisionSend.Sequence[1] = 1;
// 				__UPDATE__();
//
// 				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[5]))//39000	到达圆盘
// 					vTaskDelay(1);
// 				ex_x = 0;
// 				__UPDATE__();
// 				for(GrabPutCounter = 3;GrabPutCounter > 0;GrabPutCounter--){//圆盘夹取
// 					VisionSend.Sequence[0] = Sequence.Sequence[6 - GrabPutCounter];
// 					GM6020ExAngle = -1930;
// 					vTaskDelay(500);
// 					PosReady1 = 0;
// 					while(!PosReady1)
// 						vTaskDelay(1);
// 					AutoArmGrabDrop(4 - GrabPutCounter,0,0,0);
// 				}
// 				VisionSend.Sequence[1] = 2;
// 				//vTaskDelay(500);
// 				ex_x = 0;
// 				stage++;
// 				break;
//
//
// 				case 4:
// 				ex_x = _2006_Speed;//ex_y = 3000;
// 				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[6]))//15000  后退到中间
// 					vTaskDelay(1);
// 				Exp_Yaw = 45000;//9000 + 36000
// 				__UPDATE__();
// 				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[7]))//120000 到达加工区
// 					vTaskDelay(1);
// 				Exp_Yaw = 54000;//18000 + 36000
// 				GM6020ExAngle = -1930;
// 				vTaskDelay(1000);
// 				put = 1;
// 				PosReady = 0;
// 				while(!PosReady)
// 					vTaskDelay(1);
// 				ex_x = 0,ex_y = 0;
// 				for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){//放置物料
// 					AutoArmGrabDrop(0,0,GrabPutCounter,0);
// 					AutoArmPos(Sequence.Sequence[2 + GrabPutCounter],1);
// 					AutoArmGrabDrop(0,0,0,1);
// 					vTaskDelay(250);
// 				}
// 				for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){
// 					AutoArmPos(Sequence.Sequence[2 + GrabPutCounter],1);
// 					AutoArmGrabDrop(0,4 - GrabPutCounter,0,0);
// 				}
// 				VisionSend.Sequence[0] = GREEN;
// 				VisionSend.Sequence[1] = 1;
// 				ex_x = 0;
// 				stage ++;
// 				break;
//
//
// 				case 5:
// 				__UPDATE__();
// 				ex_x = _2006_Speed;
// 				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[8]))	//42000加工区前进
// 					vTaskDelay(1);
// 				Exp_Yaw = 63000;//27000 + 36000
// 				__UPDATE__();
// 				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[9]))	//70000到放置区
// 					vTaskDelay(1);
// 				GM6020ExAngle = -1930;
// 				vTaskDelay(1000);
// 				put = 1;
// 				PosReady = 0;
// 				while(!PosReady)
// 					vTaskDelay(1);
// 				ex_x = 0,ex_y = 0;
// 				for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){//放置物料
// 					AutoArmGrabDrop(0,0,GrabPutCounter,0);
// 					AutoArmPos(Sequence.Sequence[2 + GrabPutCounter],2);
// 					AutoArmGrabDrop(0,0,0,2);
// 				}
// 				stage ++;
// 				break;
//
//
// 				case 6:
// 				__UPDATE__();
// 				ex_x = _2006_Speed;
// 				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[10]))//42000	放置区前进
// 					vTaskDelay(1);
// 				Exp_Yaw = 72000;//36000 + 36000
// 				__UPDATE__();
// 				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[11]))	//110000回到起始区
// 					vTaskDelay(1);
// 				 ex_x = -4000;ex_y = -2500;
// 				vTaskDelay(1000);
// 				ex_y = 0;
// 				ex_x = 0;
// 				stage = 100;
// 				break;
// 				default:break;
// 			}
// 		}
// 		else if(!start)
// 		{
// 			ex_x = 0,ex_y = 0,ex_omega = 0, Remote_Control.Ex = 0,Remote_Control.Ey = 0;
// 		}
// 		if(StepMotortest)
// 		{
// 			TempCCR = StepCounter[0] - HeightCounter;
// 			if(TempCCR < 0)
// 			{
// 				__Down__(-TempCCR);
// 			}
// 			else if(TempCCR > 0)
// 			{
// 				__Up__(TempCCR);
// 			}
//
// 			TempCCR = StepCounter[1] - LengthCounter;
// 			if(TempCCR < 0)
// 			{
// 				__Length__(-TempCCR);
// 			}
// 			else if(TempCCR > 0)
// 			{
// 				__Shorten__(TempCCR);
// 			}
// 			StepMotortest = 0;
// 		}
//
// 		if(VisionTest == 1)
// 			AutoArm(Data2.ex,Data2.ey,Data2.head + Data2.ID);
// 		else if(VisionTest == 2){
// 			AutoPosAim(Data2.ex,Data2.ey,&ex_x,&ex_y,Data2.head + Data2.ID);
// 			Remote_Control.Ex = ex_omega,Remote_Control.Ey = ex_x,Remote_Control.Eomega = ex_y;
// 		}
// 		else if(VisionTest == 3){
// 			AutoArm1(Data2.ex,Data2.ey,&ex_x,Data2.head + Data2.ID);
// 			Remote_Control.Ex = ex_omega,Remote_Control.Ey = ex_x,Remote_Control.Eomega = ex_y;
// 		}
// 		else if(VisionTest == 4){
// 				__UPDATE__();
// 				ex_x = -_2006_Speed;
// 				while(!AutoMove(motor_wheel_2006,LastAngle,52500))//41636.75  到达圆盘
// 					vTaskDelay(1);
// 				ex_x = 0;
// //			for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){//放置物料
// //					AutoArmGrabDrop(0,0,GrabPutCounter,0);
// //					AutoArmPos(Sequence.Sequence[GrabPutCounter - 1],1);
// //					AutoArmGrabDrop(0,0,0,1);
// //					vTaskDelay(250);
// //				}
// //				for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){
// //					AutoArmPos(Sequence.Sequence[GrabPutCounter - 1],1);
// //					AutoArmGrabDrop(0,GrabPutCounter,0,0);
// //				}
// 				VisionTest = 100;
// 		}
// 		else if(VisionTest == 5){
// 					__UPDATE__();
// 					ex_x = _2006_Speed;//ex_y = 3000;
// 				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[0]))//11000  后退到中间
// 					vTaskDelay(1);
// 				Exp_Yaw = 9000;
// 				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[1]))//132500到达加工区
// 					vTaskDelay(1);
// 				ex_x = 0;
// 				Exp_Yaw = 18000;
// 				VisionSend.Sequence[1] = 2;
// 				GM6020ExAngle = -1950;
// 				vTaskDelay(1500);
// 				VisionTest = 100;
// //				for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){//放置物料
// //					AutoArmGrabDrop(0,0,GrabPutCounter,0);
// //					AutoArmPos(Sequence.Sequence[2 + GrabPutCounter],1);
// //					AutoArmGrabDrop(0,0,0,1);
// //					vTaskDelay(250);
// //				}
// //				for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){
// //					AutoArmPos(Sequence.Sequence[2 + GrabPutCounter],1);
// //					AutoArmGrabDrop(0,4 - GrabPutCounter,0,0);
// //				}
// 		}
// 		else if(VisionTest == 6){
// //			for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){//放置物料
// //					AutoArmGrabDrop(0,0,GrabPutCounter,0);
// //					AutoArmPos(Sequence.Sequence[GrabPutCounter - 1],1);
// //					AutoArmGrabDrop(0,0,0,1);
// //					vTaskDelay(250);
// //				}
// 		}
// 		else if(VisionTest == 7){
// //			for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){//放置物料
// //					AutoArmGrabDrop(0,0,GrabPutCounter,0);
// //					AutoArmPos(Sequence.Sequence[2 + GrabPutCounter],2);
// //					AutoArmGrabDrop(0,0,0,2);
// //				}
// 		}
// 		else if(!VisionTest)
// 			ex_x = 0,ex_y = 0;
// 		AutoArmGrabDrop(test2[0],test2[1],test2[2],test2[3]);
// 		test2[0] = 0,test2[1] = 0,test2[2] = 0,test2[3] = 0;
// 		vTaskDelayUntil(&xLastWakeTime,2);
// 	}
}

//电机PID控制task**********************************************************************
//电机PID控制task**********************************************************************
//电机PID控制task**********************************************************************
void PID_Task(void *pvParameters)
 {

	portTickType xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		/* PE2 复位式按钮替代原遥控器启动按键，并在这里完成消抖。 */
		// StartButton_Update();
		// if(!PosReady)
		// 	PosReady = AutoPosAim(Data2.ex,Data2.ey,&ex_x,&ex_y,Data2.ID);
		// else
		// 	PID_wheel_speed[2].error_inter = 0,PID_wheel_speed[3].error_inter = 0;
		// if(!PosReady1)
		// 	PosReady1 = AutoArm1(Data2.ex,Data2.ey,&ex_x,Data2.head+Data2.ID);
		// else
		// 	PID_wheel_speed[4].error_inter = 0;
		//
		//
		//
		//航向角闭环
		if(YawCorr && ImuReady)
		{
			Exp_Yaw=9000;
			// Exp_Yaw=0;
			PID_Control(Now_Yaw,Exp_Yaw,&Yaw_PositionPid);
			Cal_omega = Yaw_PositionPid.pid_out;
			/* 最大目标角速度为 18 deg/s，降低接近目标时的惯性过冲。 */
			// limit(Cal_omega,4000,-4000);
			// Cal_omega = 0;
			PID_Control(Now_Omega,Cal_omega,&Yaw_SpeedPid);
			limit(Yaw_SpeedPid.pid_out,4000,-4000);
			ex_omega=Yaw_SpeedPid.pid_out;
			Remote_Control.Eomega = ex_omega;//
		}
		else if (!ImuReady)
		{
			ex_omega = 0;
			Remote_Control.Eomega = 0;
		}

		/* 默认每100ms发可读文本；收到图二上位机合法帧后自动切换原二进制协议。 */
		{
			static uint8_t yaw_telemetry_divider;
			if (Vision_USB_IsBinaryMode() != 0U)
			{
				if (++yaw_telemetry_divider >= 10U)
				{
					yaw_telemetry_divider = 0U;
					(void)Vision_USB_TransmitYaw(Now_Yaw, Now_Omega, HAL_GetTick(),
					                              IMU_GetTemperature(), IMU_GetHeaterPwm(),
					                              Yaw_PositionPid.Kp, Yaw_PositionPid.Ki, Yaw_PositionPid.Kd,
					                              Yaw_SpeedPid.Kp, Yaw_SpeedPid.Ki, Yaw_SpeedPid.Kd);
				}
				else
				{
					(void)Vision_USB_Transmit((uint8_t*)&VisionSend, sizeof(Data1Pack));
				}
			}
			else if (++yaw_telemetry_divider >= 50U)
			{
				yaw_telemetry_divider = 0U;
				/* 暂停原来的IMU/温度/Yaw/PID文本，改看四个底盘M2006的数据流。 */
				/*
				(void)Vision_USB_TransmitTextStatus(
					Now_Yaw, Exp_Yaw, Now_Omega, IMU_GetTemperature(), IMU_GetHeaterPwm(),
					Yaw_PositionPid.Kp, Yaw_PositionPid.Ki, Yaw_PositionPid.Kd,
					Yaw_SpeedPid.Kp, Yaw_SpeedPid.Ki, Yaw_SpeedPid.Kd,
					IMU_GetCalibrationState(), IMU_GetCalibrationProgress(),
					IMU_GetGyroBiasZDegPerSecond());
				*/
				(void)Vision_USB_TransmitChassisStatus(
					expect_2006.expect_1, expect_2006.expect_2,
					expect_2006.expect_3, expect_2006.expect_4,
					expect_wheel_2006.expect_1, expect_wheel_2006.expect_2,
					expect_wheel_2006.expect_3, expect_wheel_2006.expect_4,
					motor_wheel_2006[0].Speed, motor_wheel_2006[1].Speed,
					motor_wheel_2006[2].Speed, motor_wheel_2006[3].Speed,
					M2006_Speed[0].pid_out, M2006_Speed[1].pid_out,
					M2006_Speed[2].pid_out, M2006_Speed[3].pid_out);
			}
		}
		Updatakey(&Remote_Control);

		// if(start == 2)
		// {
		// 	Remote_Control.Ex = ex_omega;
		// 	Remote_Control.Ey = -ex_x;
		// 	Remote_Control.Eomega = ex_y;
		// }
		// if(Remote_Control.First.Right_Key_Up && !Remote_Control.Second.Right_Key_Up)
		// {
		//    start++;
		// }
		// else if(Remote_Control.First.Right_Key_Down && !Remote_Control.Second.Right_Key_Down)
		// {
		// 	start = 1;
		// 	Remote_Control.Ex = 0;
		// 	Remote_Control.Ey = 0;
		// 	Remote_Control.Eomega = 0;
		// 	once[0] = 1;
		// }
		// else if(Remote_Control.First.Right_Key_Right && !Remote_Control.Second.Right_Key_Right)
		// {
		// 	if(YawCorr)
		// 		YawCorr = 0,ex_omega = 0;
		// 	else
		// 		YawCorr = 1;
		// }

		wheel_cal(&expect_2006,100,0,0);
		// wheel_cal(&expect_2006,0,0,Remote_Control.Eomega);
		// wheel_cal(&expect_2006,Remote_Control.Ex,Remote_Control.Ey,Remote_Control.Eomega);

		/*
		 * 实车安装方向修正：单 CAN 测试已确认 ID1、ID2 与底盘定义反向。
		 * 在目标速度进入闭环前取反，使目标、反馈和输出仍构成负反馈。
		 */
		expect_2006.expect_1 = -expect_2006.expect_1;
		expect_2006.expect_2 = -expect_2006.expect_2;

		expect_wheel_2006.expect_1 = RAMP_self(expect_2006.expect_1,expect_wheel_2006.expect_1,expect_wheel_ramp_2006[0]);
		PID_Control(-motor_wheel_2006[0].Speed,expect_wheel_2006.expect_1,&M2006_Speed[0]);
		limit(M2006_Speed[0].pid_out,M2006_LIMIT,-M2006_LIMIT);
		CAN2_txdata[0]=M2006_Speed[0].pid_out;

		expect_wheel_2006.expect_2 = RAMP_self(expect_2006.expect_2,expect_wheel_2006.expect_2,expect_wheel_ramp_2006[1]);
		PID_Control(-motor_wheel_2006[1].Speed,expect_wheel_2006.expect_2,&M2006_Speed[1]);
		limit(M2006_Speed[1].pid_out,M2006_LIMIT,-M2006_LIMIT);
		CAN2_txdata[1]=M2006_Speed[1].pid_out;

		expect_wheel_2006.expect_3 = RAMP_self(expect_2006.expect_3,expect_wheel_2006.expect_3,expect_wheel_ramp_2006[2]);
		PID_Control(-motor_wheel_2006[2].Speed,expect_wheel_2006.expect_3,&M2006_Speed[2]);
		limit(M2006_Speed[2].pid_out,M2006_LIMIT,-M2006_LIMIT);
		CAN2_txdata[2]=M2006_Speed[2].pid_out;

		expect_wheel_2006.expect_4 = RAMP_self(expect_2006.expect_4,expect_wheel_2006.expect_4,expect_wheel_ramp_2006[3]);
		PID_Control(-motor_wheel_2006[3].Speed,expect_wheel_2006.expect_4,&M2006_Speed[3]);
		limit(M2006_Speed[3].pid_out,M2006_LIMIT,-M2006_LIMIT);
		CAN2_txdata[3]=M2006_Speed[3].pid_out;

		MotorSend(&hcan1, 0x200, CAN2_txdata);

		/* 伸缩M2006的位置环、速度环在同一个2ms周期内计算并经CAN2发送。 */
		Mechanism_CAN2_txdata[0] = 0;
		Mechanism_CAN2_txdata[1] = 0;
		Mechanism_CAN2_txdata[2] = 0;
		Mechanism_CAN2_txdata[3] = 0;
		Mechanism_ControlTick(Mechanism_CAN2_txdata);
		MotorSend(&hcan2, 0x200, Mechanism_CAN2_txdata);

		if(!once[1])
		{
			GM6020NowAngle = RAMP_self(GM6020ExAngle,GM6020NowAngle,GM6020Ramp);
			limit(GM6020NowAngle,2048,-4096);
			PID_Control_Smis((motor_rotor.Angle - wheel_offset[0]) * 5,GM6020NowAngle * 5,&PID_wheel_position[0],motor_rotor.Speed);
			limit(PID_wheel_position[0].pid_out,GM6020_LIMIT,-GM6020_LIMIT);

			PID_Control(motor_rotor.Speed,PID_wheel_position[0].pid_out,&PID_wheel_speed[0]);
			limit(PID_wheel_speed[0].pid_out,12000,-12000);
			if(put)
			{
				limit(PID_wheel_speed[0].pid_out,4500,-4500);
			}

			CAN6020[0]=PID_wheel_speed[0].pid_out;
		}

		/* GM6020反馈ID为0x205，对应0x1FF控制帧第一个电流值，且位于CAN2。 */
		MotorSend(&hcan2,0x1FF,CAN6020);

		vTaskDelayUntil(&xLastWakeTime,2);
  }
}



void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	uint8_t *buffer = (hcan->Instance == FDCAN1) ? CAN1_buff : CAN2_buff;
	uint16_t id = CAN_Receive_DataFrame(hcan, buffer);

	if (hcan->Instance == FDCAN1) {
		/* CAN1只接四个底盘M2006。 */
		switch (id) {
			case 0x201: M2006_Receive(&motor_wheel_2006[0], buffer); break;
			case 0x202: M2006_Receive(&motor_wheel_2006[1], buffer); break;
			case 0x203: M2006_Receive(&motor_wheel_2006[2], buffer); break;
			case 0x204: M2006_Receive(&motor_wheel_2006[3], buffer); break;
			default: break;
		}
	} else if (hcan->Instance == FDCAN2) {
		/* CAN2接伸缩M2006和GM6020；升降仍由PA0步进脉冲控制。 */
		switch (id) {
			case MECH_HEIGHT_CAN_ID:
				Mechanism_OnM2006Feedback(MECH_AXIS_HEIGHT, buffer);
				break;
			case MECH_LENGTH_CAN_ID:
				Mechanism_OnM2006Feedback(MECH_AXIS_LENGTH, buffer);
				break;
			case MECH_GM6020_CAN_ID:
				GM6020_Receive(&motor_rotor, buffer);
				if (once[1]) once[1] = 0;
				break;
			default: break;
		}
	}
}

/* ==================== 阅读导航 ====================
 * 下一步：先看 MDK-ARM/Auto.h，再看 MDK-ARM/Auto.c 的 AutoMove、AutoArmPos、AutoArmGrabDrop。
 * ================================================== */
//9000
