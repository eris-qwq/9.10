#include "PID_task.h"
#include "motor.h"
#include "tim.h"        
#include "ramp.h"
#include "math.h"             
#include "VCOMCOMM.h"
#include "My_code.h"
#include "usart.h"
#include "Auto.h"

extern  Data2;
int16_t CAN1_txdata[4];
int16_t CAN2_txdata[4];
int16_t CAN6020[4];
extern Remote_Handle_t Remote_Control;
Chassis_Motor_expect expect_wheel_2006;
Chassis_Motor_expect expect_2006;
int32_t expect_wheel_ramp_2006[4]={50,50,50,50};   //电机斜坡
M2006_TypeDef motor_wheel_2006[4];
GM6020_TypeDef motor_rotor;
float GM6020ExAngle,GM6020NowAngle;
extern uint8_t ScreenGo;
int32_t wheel_offset[4] = {3100,0,0,0};
int32_t M2006_offset[4];
uint8_t stage = 0,put = 0;
int16_t debug = 1100;
#if SPEED_MODE
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

PID Yaw_SpeedPid = {.Kp=0.67,  .Ki=0.0015,.Kd=2.5,.limit = 100000};
PID Yaw_PositionPid ={.Kp=2,.Ki=0.0015,.Kd=3,  .limit = 50000};
float Now_Yaw,Exp_Yaw;
										
void Updatakey(Remote_Handle_t * xx) { //遥控器数据更新
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
    }		
    else
    {
        if (buffer < -ramp)
                now += -ramp;
        else
                now += buffer;
    }
    return now;
}
Data1Pack VisionSend = {
	.head = 0x5A,
	.DataLen = 2,
	.Sequence[0] = 1,
	.Sequence[1] = 1
},Sequence = {.Sequence = {1,2,3,3,2,1}};//颜色1R,2G,3B;物料/色环1,2;二维码扫描0 0
uint8_t start = 0,once[3] = {1,1,1},YawCorr,StepMotortest,test2[4],VisionTest,PosReady = 1,PosReady1 = 1;
uint32_t Delay = 70000;
int16_t ex_x,ex_y,ex_omega,tempomega,Cal_omega,Now_Omega,omegaramp = 100,GM6020Ramp = 10,GM6020Low = 100,GM6020LowLimit = 1750;
float LastAngle[4],NowAngle1[4],NowAngle2[4];
extern uint8_t GrabPutCounter;
extern int32_t StepCounter[2];
int HeightCounter,LengthCounter,TempCCR;
#define __UPDATE__()  LastAngle[0] = motor_wheel_2006[0].Angle_DEG,LastAngle[1] = motor_wheel_2006[1].Angle_DEG;\
					  LastAngle[2] = motor_wheel_2006[2].Angle_DEG,LastAngle[3] = motor_wheel_2006[3].Angle_DEG;
uint32_t Distance[12] = {7500,125000,42000,47000,42000,39000,15000,115000,44000,50000,42000,83000};
// {7500,120000,42000,47000,42000,39000,15000,115000,44000,50000,42000,83000};
//Y=X前 EOMEGA = Y左右 X =EOMEGA自旋 -9000逆时针旋转90° 编码器编码 24546
void Remote_deal(void* param){
	portTickType xLastWakeTime = xTaskGetTickCount();
	while(1){
		if(start == 2){
			if(once[0]){//走出蓝框
			 VisionSend.Sequence[0] = 0,VisionSend.Sequence[1] = 0;
		     YawCorr = 1;
			 ex_x = 4000;ex_y = 2500;
			 vTaskDelay(1000);
			 ex_y = 0;
			 vTaskDelay(1000);
			 ex_x = 0;
			 once[0] = 0;
			 __UPDATE__();
			}
			switch(stage){
				case 0:
				while(!ScreenGo)
					vTaskDelay(1);//等待摄像头扫描QR完成
				VisionSend.Sequence[0] = 1;
				VisionSend.Sequence[1] = 1;
				ex_x = -_2006_Speed;
				while(!AutoMove(motor_wheel_2006,LastAngle,52500))//41636.75  到达圆盘
					vTaskDelay(1);
				ex_x = 0;
				__UPDATE__();
				for(GrabPutCounter = 3;GrabPutCounter > 0;GrabPutCounter--){//圆盘夹取
					VisionSend.Sequence[0] = Sequence.Sequence[3 - GrabPutCounter];
					GM6020ExAngle = -1930;
					vTaskDelay(500);
					PosReady1 = 0;
					while(!PosReady1)
						vTaskDelay(1);
					AutoArmGrabDrop(4 - GrabPutCounter,0,0,0);
				}
				ex_x = _2006_Speed;//ex_y = 3000;
				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[0]))//11000  后退到中间
					vTaskDelay(1);
				Exp_Yaw = 9000;
				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[1]))//132500到达加工区
					vTaskDelay(1);
				ex_x = 0;
				Exp_Yaw = 18000;
				VisionSend.Sequence[1] = 2;
				GM6020ExAngle = -1950;
				vTaskDelay(1500);
				stage++;
				break;
				case 1:
				put = 1;
				PosReady = 0;
				while(!PosReady)
					vTaskDelay(1);
				ex_x = 0,ex_y = 0;
				for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){//放置物料
					AutoArmGrabDrop(0,0,GrabPutCounter,0);
					AutoArmPos(Sequence.Sequence[GrabPutCounter - 1],1);
					AutoArmGrabDrop(0,0,0,1);
					vTaskDelay(250);
				}
				for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){
					AutoArmPos(Sequence.Sequence[GrabPutCounter - 1],1);
					AutoArmGrabDrop(0,GrabPutCounter,0,0);
				}
				put = 0;
				stage++;
				break;
				case 2:
				__UPDATE__();
				ex_x = _2006_Speed;
				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[2]))//44000  加工区前进
					vTaskDelay(1);
				Exp_Yaw = 27000;
				__UPDATE__();
				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[3]))//50000  到达放置区
					vTaskDelay(1);
				GM6020ExAngle = -1950;
				vTaskDelay(2000);
				ex_x = 0;
				put = 1;
				PosReady = 0;
				while(!PosReady)
					vTaskDelay(1);
				ex_x = 0,ex_y = 0;
				for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){//放置物料
					AutoArmGrabDrop(0,0,GrabPutCounter,0);
					AutoArmPos(Sequence.Sequence[GrabPutCounter - 1],1);
					AutoArmGrabDrop(0,0,0,1);
					vTaskDelay(250);
				}
				
				stage++;
				break;
				case 3:
				__UPDATE__();
				ex_x = _2006_Speed;
				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[4]))//42000	放置区前进
					vTaskDelay(1);
				Exp_Yaw = 36000;
				VisionSend.Sequence[1] = 1;
				__UPDATE__();
				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[5]))//39000	到达圆盘
					vTaskDelay(1);
				ex_x = 0;
				__UPDATE__();
				for(GrabPutCounter = 3;GrabPutCounter > 0;GrabPutCounter--){//圆盘夹取
					VisionSend.Sequence[0] = Sequence.Sequence[6 - GrabPutCounter];
					GM6020ExAngle = -1930;
					vTaskDelay(500);
					PosReady1 = 0;
					while(!PosReady1)
						vTaskDelay(1);
					AutoArmGrabDrop(4 - GrabPutCounter,0,0,0);
				}
				VisionSend.Sequence[1] = 2;
				//vTaskDelay(500);
				ex_x = 0;
				stage++;
				break;
				case 4:
				ex_x = _2006_Speed;//ex_y = 3000;
				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[6]))//15000  后退到中间
					vTaskDelay(1);
				Exp_Yaw = 45000;//9000 + 36000
				__UPDATE__();
				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[7]))//120000 到达加工区
					vTaskDelay(1);
				Exp_Yaw = 54000;//18000 + 36000
				GM6020ExAngle = -1930;
				vTaskDelay(1000);
				put = 1;
				PosReady = 0;
				while(!PosReady)
					vTaskDelay(1);
				ex_x = 0,ex_y = 0;
				for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){//放置物料
					AutoArmGrabDrop(0,0,GrabPutCounter,0);
					AutoArmPos(Sequence.Sequence[2 + GrabPutCounter],1);
					AutoArmGrabDrop(0,0,0,1);
					vTaskDelay(250);
				}
				for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){
					AutoArmPos(Sequence.Sequence[2 + GrabPutCounter],1);
					AutoArmGrabDrop(0,4 - GrabPutCounter,0,0);
				}
				VisionSend.Sequence[0] = 2;
				VisionSend.Sequence[1] = 1;
				ex_x = 0;
				stage ++;
				break;
				case 5:
				__UPDATE__();
				ex_x = _2006_Speed;
				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[8]))	//42000加工区前进
					vTaskDelay(1);
				Exp_Yaw = 63000;//27000 + 36000
				__UPDATE__();
				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[9]))	//70000到放置区
					vTaskDelay(1);
				GM6020ExAngle = -1930;
				vTaskDelay(1000);
				put = 1;
				PosReady = 0;
				while(!PosReady)
					vTaskDelay(1);
				ex_x = 0,ex_y = 0;
				for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){//放置物料
					AutoArmGrabDrop(0,0,GrabPutCounter,0);
					AutoArmPos(Sequence.Sequence[2 + GrabPutCounter],2);
					AutoArmGrabDrop(0,0,0,2);
				}
				stage ++;
				break;
				case 6:
				__UPDATE__();
				ex_x = _2006_Speed;
				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[10]))//42000	放置区前进
					vTaskDelay(1);
				Exp_Yaw = 72000;//36000 + 36000
				__UPDATE__();
				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[11]))	//110000回到起始区
					vTaskDelay(1);
				 ex_x = -4000;ex_y = -2500;
				vTaskDelay(1000);
				ex_y = 0;
				ex_x = 0;
				stage = 100;
				break;
				default:break;
			}
		}
		else if(!start){
			ex_x = 0,ex_y = 0,ex_omega = 0, Remote_Control.Ex = 0,Remote_Control.Ey = 0;
		}
		if(StepMotortest){
			TempCCR = StepCounter[0] - HeightCounter;
			if(TempCCR < 0){
				__Down__(-TempCCR);
			}
			else if(TempCCR > 0){
				__Up__(TempCCR);
			}
			
			TempCCR = StepCounter[1] - LengthCounter;
			if(TempCCR < 0){
				__Length__(-TempCCR);
			}
			else if(TempCCR > 0){
				__Shorten__(TempCCR);
			}
			StepMotortest = 0;
		} 

		if(VisionTest == 1)
			AutoArm(Data2.ex,Data2.ey,Data2.head + Data2.ID);
		else if(VisionTest == 2){
			AutoPosAim(Data2.ex,Data2.ey,&ex_x,&ex_y,Data2.head + Data2.ID);
			Remote_Control.Ex = ex_omega,Remote_Control.Ey = ex_x,Remote_Control.Eomega = ex_y;
		}
		else if(VisionTest == 3){
			AutoArm1(Data2.ex,Data2.ey,&ex_x,Data2.head + Data2.ID);
			Remote_Control.Ex = ex_omega,Remote_Control.Ey = ex_x,Remote_Control.Eomega = ex_y;
		}
		else if(VisionTest == 4){
				__UPDATE__();
				ex_x = -_2006_Speed;
				while(!AutoMove(motor_wheel_2006,LastAngle,52500))//41636.75  到达圆盘
					vTaskDelay(1);
				ex_x = 0;
//			for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){//放置物料
//					AutoArmGrabDrop(0,0,GrabPutCounter,0);
//					AutoArmPos(Sequence.Sequence[GrabPutCounter - 1],1);
//					AutoArmGrabDrop(0,0,0,1);
//					vTaskDelay(250);
//				}
//				for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){
//					AutoArmPos(Sequence.Sequence[GrabPutCounter - 1],1);
//					AutoArmGrabDrop(0,GrabPutCounter,0,0);
//				}
				VisionTest = 100;
		}
		else if(VisionTest == 5){
					__UPDATE__();
					ex_x = _2006_Speed;//ex_y = 3000;
				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[0]))//11000  后退到中间
					vTaskDelay(1);
				Exp_Yaw = 9000;
				while(!AutoMove(motor_wheel_2006,LastAngle,Distance[1]))//132500到达加工区
					vTaskDelay(1);
				ex_x = 0;
				Exp_Yaw = 18000;
				VisionSend.Sequence[1] = 2;
				GM6020ExAngle = -1950;
				vTaskDelay(1500);
				VisionTest = 100;
//				for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){//放置物料
//					AutoArmGrabDrop(0,0,GrabPutCounter,0);
//					AutoArmPos(Sequence.Sequence[2 + GrabPutCounter],1);
//					AutoArmGrabDrop(0,0,0,1);
//					vTaskDelay(250);
//				}
//				for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){
//					AutoArmPos(Sequence.Sequence[2 + GrabPutCounter],1);
//					AutoArmGrabDrop(0,4 - GrabPutCounter,0,0);
//				}
		}
		else if(VisionTest == 6){
//			for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){//放置物料
//					AutoArmGrabDrop(0,0,GrabPutCounter,0);
//					AutoArmPos(Sequence.Sequence[GrabPutCounter - 1],1);
//					AutoArmGrabDrop(0,0,0,1);
//					vTaskDelay(250);
//				}
		}		
		else if(VisionTest == 7){
//			for(GrabPutCounter = 1;GrabPutCounter < 4;GrabPutCounter++){//放置物料
//					AutoArmGrabDrop(0,0,GrabPutCounter,0);
//					AutoArmPos(Sequence.Sequence[2 + GrabPutCounter],2);
//					AutoArmGrabDrop(0,0,0,2);
//				}
		}
		else if(!VisionTest)
			ex_x = 0,ex_y = 0;
		AutoArmGrabDrop(test2[0],test2[1],test2[2],test2[3]);
		test2[0] = 0,test2[1] = 0,test2[2] = 0,test2[3] = 0;
		vTaskDelayUntil(&xLastWakeTime,2);
	}
}
#if SPEED_MODE					

void PID_Task(void *pvParameters){ //电机pid控制任务    
	
	portTickType xLastWakeTime = xTaskGetTickCount();
	
	for(;;){
		if(!HAL_GPIO_ReadPin(Control_GPIO_Port,Control_Pin))
			start = 2;
		if(!PosReady)
			PosReady = AutoPosAim(Data2.ex,Data2.ey,&ex_x,&ex_y,Data2.ID);
		else
			PID_wheel_speed[2].error_inter = 0,PID_wheel_speed[3].error_inter = 0;
		if(!PosReady1)
			PosReady1 = AutoArm1(Data2.ex,Data2.ey,&ex_x,Data2.head+Data2.ID);
		else
			PID_wheel_speed[4].error_inter = 0;
		if(YawCorr){
		PID_Control(-Now_Yaw,Exp_Yaw,&Yaw_PositionPid);
		Cal_omega = Yaw_PositionPid.pid_out;
		PID_Control(Now_Omega,Cal_omega,&Yaw_SpeedPid);
		limit(Yaw_SpeedPid.pid_out,4000,-4000);
		ex_omega=Yaw_SpeedPid.pid_out;
		Remote_Control.Ex = ex_omega;
		}
		
		HAL_UART_Transmit(&huart2,(uint8_t*)&VisionSend,sizeof(Data1Pack) - 2,0x1F);
		Updatakey(&Remote_Control);
		if(start == 2)
			Remote_Control.Ex = ex_omega,Remote_Control.Ey = -ex_x,Remote_Control.Eomega = ex_y;
		if(Remote_Control.First.Right_Key_Up && !Remote_Control.Second.Right_Key_Up){
		   start++;
		}
		else if(Remote_Control.First.Right_Key_Down && !Remote_Control.Second.Right_Key_Down){
		   start = 1;Remote_Control.Ex = 0,Remote_Control.Ey = 0,Remote_Control.Eomega = 0,once[0] = 1;
		}
		else if(Remote_Control.First.Right_Key_Right && !Remote_Control.Second.Right_Key_Right){
			if(YawCorr)
				YawCorr = 0,ex_omega = 0;
			else
				YawCorr = 1;
		}
			
		wheel_cal(&expect_2006,Remote_Control.Ex,Remote_Control.Ey,Remote_Control.Eomega);
		
		expect_wheel_2006.expect_1 = RAMP_self(expect_2006.expect_1,expect_wheel_2006.expect_1,expect_wheel_ramp_2006[0]);
		PID_Control(motor_wheel_2006[0].Speed,expect_wheel_2006.expect_1,&M2006_Speed[0]);
		limit(M2006_Speed[0].pid_out,M2006_LIMIT,-M2006_LIMIT);
		CAN2_txdata[0]=M2006_Speed[0].pid_out;
		
		expect_wheel_2006.expect_2 = RAMP_self(expect_2006.expect_2,expect_wheel_2006.expect_2,expect_wheel_ramp_2006[1]);
		PID_Control(motor_wheel_2006[1].Speed,expect_wheel_2006.expect_2,&M2006_Speed[1]);
		limit(M2006_Speed[1].pid_out,M2006_LIMIT,-M2006_LIMIT);
		CAN2_txdata[1]=M2006_Speed[1].pid_out;
			
		expect_wheel_2006.expect_3 = RAMP_self(expect_2006.expect_3,expect_wheel_2006.expect_3,expect_wheel_ramp_2006[2]);
		PID_Control(motor_wheel_2006[2].Speed,expect_wheel_2006.expect_3,&M2006_Speed[2]);
		limit(M2006_Speed[2].pid_out,M2006_LIMIT,-M2006_LIMIT);
		CAN2_txdata[2]=M2006_Speed[2].pid_out;
		
		expect_wheel_2006.expect_4 = RAMP_self(expect_2006.expect_4,expect_wheel_2006.expect_4,expect_wheel_ramp_2006[3]);
		PID_Control(motor_wheel_2006[3].Speed,expect_wheel_2006.expect_4,&M2006_Speed[3]);
		limit(M2006_Speed[3].pid_out,M2006_LIMIT,-M2006_LIMIT);
		CAN2_txdata[3]=M2006_Speed[3].pid_out;
		
		MotorSend(&hcan2, 0x200, CAN2_txdata);
		
		if(!once[1]){
				GM6020NowAngle = RAMP_self(GM6020ExAngle,GM6020NowAngle,GM6020Ramp);
			limit(GM6020NowAngle,2048,-4096);
			PID_Control_Smis((motor_rotor.Angle - wheel_offset[0]) * 5,GM6020NowAngle * 5,&PID_wheel_position[0],motor_rotor.Speed);
			limit(PID_wheel_position[0].pid_out,GM6020_LIMIT,-GM6020_LIMIT);
			PID_Control(motor_rotor.Speed,PID_wheel_position[0].pid_out,&PID_wheel_speed[0]);				
			limit(PID_wheel_speed[0].pid_out,12000,-12000);
			if(put)
				limit(PID_wheel_speed[0].pid_out,4500,-4500);
			
			CAN6020[0]=PID_wheel_speed[0].pid_out;
		}
		MotorSend(&hcan2,0x1FF,CAN6020);
		
		vTaskDelayUntil(&xLastWakeTime,2);
  }
}
#else
void PID_Task(void *pvParameters){ //电机pid控制任务
	
    wheel_offset[0] = motor_wheel_3508[0].Angle;//   纠偏
    wheel_offset[1] = motor_wheel_3508[1].Angle;
    wheel_offset[2] = motor_wheel_3508[2].Angle;
	wheel_offset[3] = motor_wheel_3508[3].Angle;
    
	M2006_offset[0] = motor_wheel_2006[0].Angle;//   纠偏
    M2006_offset[1] = motor_wheel_2006[1].Angle;
	M2006_offset[2] = motor_wheel_2006[2].Angle;
	M2006_offset[3] = motor_wheel_2006[3].Angle;
    
	portTickType xLastWakeTime = xTaskGetTickCount();
	
	for(;;){
#if RM3510_MODE
		//3510位置模式控制
		expect_wheel_3508.expect_1 = RAMP_self(expect_angle_3508.expect_1,expect_wheel_3508.expect_1,expect_wheel_ramp_3508[0]);
		PID_Control_Smis(motor_wheel_3510[0].Angle - wheel_offset[0],expect_wheel_3508.expect_1,&PID_wheel_position[0],motor_wheel_3510[0].Speed);
		PID_Control(motor_wheel_3510[0].Speed,PID_wheel_position[0].pid_out,&PID_wheel_speed[0]);
		limit(PID_wheel_speed[0].pid_out,RM3510_LIMIT,-RM3510_LIMIT);
		CAN1_txdata[0]=PID_wheel_speed[0].pid_out;
		
		expect_wheel_3508.expect_2 = RAMP_self(expect_angle_3508.expect_2,expect_wheel_3508.expect_2,expect_wheel_ramp_3508[1]);
		PID_Control_Smis(motor_wheel_3510[1].Angle - wheel_offset[1],expect_wheel_3508.expect_2,&PID_wheel_position[1],motor_wheel_3510[1].Speed);
		PID_Control(motor_wheel_3510[1].Speed,PID_wheel_position[1].pid_out,&PID_wheel_speed[1]);
		limit(PID_wheel_speed[1].pid_out,RM3510_LIMIT,-RM3510_LIMIT);
		CAN1_txdata[1]=PID_wheel_speed[1].pid_out;
		
		expect_wheel_3508.expect_3 = RAMP_self(expect_angle_3508.expect_3,expect_wheel_3508.expect_3,expect_wheel_ramp_3508[2]);
		PID_Control_Smis(motor_wheel_3510[2].Angle - wheel_offset[2],expect_wheel_3508.expect_3,&PID_wheel_position[2],motor_wheel_3510[2].Speed);
		PID_Control(motor_wheel_3510[2].Speed,PID_wheel_position[2].pid_out,&PID_wheel_speed[2]);
		limit(PID_wheel_speed[2].pid_out,RM3510_LIMIT,-RM3510_LIMIT);
		CAN1_txdata[2]=PID_wheel_speed[2].pid_out;
		
		expect_wheel_3508.expect_4 = RAMP_self(expect_angle_3508.expect_4,expect_wheel_3508.expect_4,expect_wheel_ramp_3508[]);
		PID_Control_Smis(motor_wheel_3510[3].Angle - wheel_offset[3],expect_wheel_3508.expect_4,&PID_wheel_position[3],motor_wheel_3510[3].Speed);
		PID_Control(motor_wheel_3510[3].Speed,PID_wheel_position[3].pid_out,&PID_wheel_speed[3]);
		limit(PID_wheel_speed[3].pid_out,RM3510_LIMIT,-RM3510_LIMIT);
		CAN1_txdata[]=PID_wheel_speed[3].pid_out;
#else
		//3508位置模式控制
		expect_wheel_3508.expect_1 = RAMP_self(expect_angle_3508.expect_1,expect_wheel_3508.expect_1,expect_wheel_ramp_3508[0]);
		PID_Control_Smis(motor_wheel_3508[0].Angle - wheel_offset[0],expect_wheel_3508.expect_1,&PID_wheel_position[0],motor_wheel_3508[0].Speed);
		PID_Control(motor_wheel_3508[0].Speed,PID_wheel_position[0].pid_out,&PID_wheel_speed[0]);
		limit(PID_wheel_speed[0].pid_out,RM3508_LIMIT,-RM3508_LIMIT);
		CAN1_txdata[0]=PID_wheel_speed[0].pid_out;
		
		expect_wheel_3508.expect_2 = RAMP_self(expect_angle_3508.expect_2,expect_wheel_3508.expect_2,expect_wheel_ramp_3508[1]);
		PID_Control_Smis(motor_wheel_3508[1].Angle - wheel_offset[1],expect_wheel_3508.expect_2,&PID_wheel_position[1],motor_wheel_3508[1].Speed);
		PID_Control(motor_wheel_3508[1].Speed,PID_wheel_position[1].pid_out,&PID_wheel_speed[1]);
		limit(PID_wheel_speed[1].pid_out,RM3508_LIMIT,-RM3508_LIMIT);
		CAN1_txdata[1]=PID_wheel_speed[1].pid_out;
		
		expect_wheel_3508.expect_3 = RAMP_self(expect_angle_3508.expect_3,expect_wheel_3508.expect_3,expect_wheel_ramp_3508[2]);
		PID_Control_Smis(motor_wheel_3508[2].Angle - wheel_offset[2],expect_wheel_3508.expect_3,&PID_wheel_position[2],motor_wheel_3508[2].Speed);
		PID_Control(motor_wheel_3508[2].Speed,PID_wheel_position[2].pid_out,&PID_wheel_speed[2]);
		limit(PID_wheel_speed[2].pid_out,RM3508_LIMIT,-RM3508_LIMIT);
		CAN1_txdata[2]=PID_wheel_speed[2].pid_out;
		
		expect_wheel_3508.expect_4 = RAMP_self(expect_angle_3508.expect_4,expect_wheel_3508.expect_4,expect_wheel_ramp_3508[3]);
		PID_Control_Smis(motor_wheel_3508[3].Angle - wheel_offset[3],expect_wheel_3508.expect_4,&PID_wheel_position[3],motor_wheel_3508[3].Speed);
		PID_Control(motor_wheel_3508[3].Speed,PID_wheel_position[3].pid_out,&PID_wheel_speed[3]);
		limit(PID_wheel_speed[3].pid_out,RM3508_LIMIT,-RM3508_LIMIT);
		CAN1_txdata[3]=PID_wheel_speed[3].pid_out;
#endif		
		MotorSend(&hcan1,0x200,CAN1_txdata);
		
		//2006位置模式控制
		expect_wheel_2006.expect_1 = RAMP_self(expect_angle_2006.expect_1,expect_wheel_2006.expect_1,expect_wheel_ramp_2006[0]);
		PID_Control_Smis(motor_wheel_2006[0].Angle - M2006_offset[0],expect_wheel_2006.expect_1,&M2006_Angle[0],motor_wheel_2006[0].Speed);
 		limit(M2006_Angle[0].pid_out,M2006_LIMIT,-M2006_LIMIT);		
		PID_Control(motor_wheel_2006[0].Speed,M2006_Angle[0].pid_out,&M2006_Speed[0]);
		limit(M2006_Speed[0].pid_out,M2006_LIMIT,-M2006_LIMIT);
		CAN2_txdata[0]=M2006_Speed[0].pid_out;
		
		expect_wheel_2006.expect_2 = RAMP_self(expect_angle_2006.expect_2,expect_wheel_2006.expect_2,expect_wheel_ramp_2006[1]);
		PID_Control_Smis(motor_wheel_2006[1].Angle - M2006_offset[1],expect_wheel_2006.expect_2,&M2006_Angle[1],motor_wheel_2006[1].Speed);
 		limit(M2006_Angle[1].pid_out,M2006_LIMIT,-M2006_LIMIT);		
		PID_Control(motor_wheel_2006[1].Speed,M2006_Angle[0].pid_out,&M2006_Speed[1]);
		limit(M2006_Speed[1].pid_out,M2006_LIMIT,-M2006_LIMIT);
		CAN2_txdata[1]=M2006_Speed[1].pid_out;
		
		expect_wheel_2006.expect_3 = RAMP_self(expect_angle_2006.expect_3,expect_wheel_2006.expect_3,expect_wheel_ramp_2006[2]);
		PID_Control_Smis(motor_wheel_2006[2].Angle - M2006_offset[2],expect_wheel_2006.expect_3,&M2006_Angle[2],motor_wheel_2006[2].Speed);
 		limit(M2006_Angle[2].pid_out,M2006_LIMIT,-M2006_LIMIT);		
		PID_Control(motor_wheel_2006[2].Speed,M2006_Angle[2].pid_out,&M2006_Speed[2]);
		limit(M2006_Speed[2].pid_out,M2006_LIMIT,-M2006_LIMIT);
		CAN2_txdata[2]=M2006_Speed[2].pid_out;
		
		expect_wheel_2006.expect_4 = RAMP_self(expect_angle_2006.expect_4,expect_wheel_2006.expect_4,expect_wheel_ramp_2006[3]);
		PID_Control_Smis(motor_wheel_2006[3].Angle - M2006_offset[3],expect_wheel_2006.expect_4,&M2006_Angle[3],motor_wheel_2006[3].Speed);
 		limit(M2006_Angle[3].pid_out,M2006_LIMIT,-M2006_LIMIT);		
		PID_Control(motor_wheel_2006[3].Speed,M2006_Angle[3].pid_out,&M2006_Speed[3]);
		limit(M2006_Speed[3].pid_out,M2006_LIMIT,-M2006_LIMIT);
		CAN2_txdata[3]=M2006_Speed[3].pid_out;
		MotorSend(&hcan2, 0x200, CAN2_txdata);// ID:201~204
		
		vTaskDelay(1);
		vTaskDelayUntil(&xLastWakeTime,2);
  }
}
#endif



void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	
	uint16_t ID1 = CAN_Receive_DataFrame(&hcan2,CAN2_buff);     
	switch(ID1){             
		case 0x201: 
			M2006_Receive(&motor_wheel_2006[0], CAN2_buff);
			break;
		case 0x202: 
			M2006_Receive(&motor_wheel_2006[1], CAN2_buff);
			break;
		case 0x203: 
			M2006_Receive(&motor_wheel_2006[2], CAN2_buff);
			break;
		case 0x204: 
			M2006_Receive(&motor_wheel_2006[3], CAN2_buff);
			break;
		case 0x205:
            GM6020_Receive(&motor_rotor,  CAN2_buff);
			if(once[1]){
//				wheel_offset[0] = motor_rotor.Angle;
				once[1] = 0;
			}
            break;
		default: break;
	}
	
				
}
