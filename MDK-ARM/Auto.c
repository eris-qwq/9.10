
#include "Auto.h"

int16_t GMoffset = -1930,YawLow = 10,YawLimit = 350;
uint8_t GrabPutCounter = 3,test1 = 1;
uint8_t Dead = 10,StepStart[2],Deadt = 30;
int8_t Dir[2] = {1,1};
int32_t StepCounter[2],Stable;
Auto_t AutoSequence;
extern float GM6020ExAngle;
extern uint8_t test2[4];
int16_t RedLengt = 2765,GreenLengt = 800, BlueLengt = 1460,RedAngle = -1035,GreenAngle = -1930,BlueAngle = -2940;

uint8_t AutoArm(int16_t Ex_x,int16_t Ex_y,uint8_t is_Valid){
	  if(!is_Valid){
	  PID_Control(-Ex_x,0,&PID_wheel_speed[1]);	 
	 
	  GM6020ExAngle = PID_wheel_speed[1].pid_out + GMoffset;
		
		if(Ex_y > Dead && StepCounter[1] <= 6400)
			Mechanism_JogLength(1);
		else if(Ex_y < -Dead && StepCounter[1] >= 0)
			Mechanism_JogLength(-1);
		else
			Mechanism_JogLength(0);
	
	if(abs(Ex_y) < Deadt && abs(Ex_x) < 15){
		Stable ++;
	}
	else
		Stable = 0;
	
	if(Stable > 20){
		Stable = 0;
		return 1;
	}
	else
		return 0;
	}
	  else
		  return 0;
}
uint16_t AutoDead = 100;
uint8_t AutoArm1(int16_t Ex_x,int16_t Ex_y,int16_t* ex,uint8_t is_Valid){
	 if(is_Valid == 0x5A){
	  PID_Control(Ex_x * 10,90,&PID_wheel_speed[4]);	
	  limit(PID_wheel_speed[4].pid_out,2000,-2000);
	  *ex = PID_wheel_speed[4].pid_out;
	  GM6020ExAngle = GMoffset;
		 
		if(Ex_y > Dead && StepCounter[1] <= 6400)
			Mechanism_JogLength(1);
		else if(Ex_y < -Dead && StepCounter[1] >= 0)
			Mechanism_JogLength(-1);
		else
			Mechanism_JogLength(0);
	
	
	if(abs(Ex_y) < Deadt && abs(Ex_x) < 30){
		Stable ++;
	}
	else
		Stable = 0;
	
	if(Stable > AutoDead){
		Stable = 0;
		*ex = 0;
		return 1;
	}
	else{
		return 0;
	}
	}
	*ex = 0;
	return 0; 
}

void AutoArmPos(uint8_t Color,uint8_t height){
	int16_t GM6020Position = 0,Temp0 = 0,TempLength = 0,Stable0 = 0;
	switch(Color){
	case RED:	GM6020Position = RedAngle;TempLength = RedLengt;
	break;
	case GREEN: GM6020Position = GreenAngle;TempLength = GreenLengt;
	break;
	case BLUE:  GM6020Position = BlueAngle;TempLength = BlueLengt;
	break;
	/* 黄、黑、浅蓝尚未取得机械角度和伸缩长度，禁止按默认0位置误动作。 */
	default:return;
	}
	
	GM6020ExAngle = GM6020Position;
	while(1){
		if(fabs(PID_wheel_position[0].error_now) < 5)
			Stable0 ++;
		else
			Stable0 = 0;
		if(Stable0 > 10)
			break;
		vTaskDelay(1);
	}
	if(Color == RED){
		__Length__(250);
	}
	if(height == 1)
	Temp0 = GroundHeight - StepCounter[0];
	else if(height == 2)
	Temp0 = Put2Height - StepCounter[0];
	/* 原来两路步进同时计时；换成M2006后按两个绝对目标闭环到位。 */
	Mechanism_MovePairTo((float)(StepCounter[0] + Temp0), (float)TempLength);
}

void AutoArmGrabDrop(uint8_t GrabPlace1,uint8_t GroundGrab,uint8_t CarGrab,uint8_t PutPlace){
	int32_t GM6020Pos = 0,Length = 0,TempLength = 0,TempHeight = 0,TempPutHeight = 0,Stable1 = 0;
	if(GrabPlace1){//������ȡ��鲢���õ�����
		switch(GrabPlace1){
			case 1:
			GM6020Pos = Place1,Length = DropLen1;
			break;
			case 2:
			GM6020Pos = Place2,Length = DropLen2;
			break;
			case 3:
			GM6020Pos = Place3,Length = DropLen3;
			break;
			default:break;
		}
		TempHeight = StepCounter[0] - GrabHeight;
		if(TempHeight < 0){
			__Down__(-TempHeight);
		}
		else if(TempHeight > 0){
			__Up__(TempHeight);
		}
		vTaskDelay(250);
		__Grab__();
		TempLength = StepCounter[1];
		if(TempLength < 0){
			__Length__(-TempLength);
		}
		else if(TempLength > 0){
			__Shorten__(TempLength);
		}
		if(GrabPlace1 == 1){
		TempHeight = StepCounter[0] - CarDropHeight;
		if(TempHeight < 0){
			__Down__(-TempHeight);
		}
		else if(TempHeight > 0){
			__Up__(TempHeight);
		}
		}
		else{
			__Up__(StepCounter[0]);
		}
		GM6020ExAngle = GM6020Pos;
		while(1){
		if(fabs(PID_wheel_position[0].error_now) < 25)
			Stable1 ++;
		else
			Stable1 = 0;
		if(Stable1 > 5)
			break;
		vTaskDelay(1);
	}
		TempLength = StepCounter[1] - Length;
		if(TempLength < 0){
			__Length__(-TempLength);
		}
		else if(TempLength > 0){
			__Shorten__(TempLength);
		}
		TempHeight = StepCounter[0] - CarDropHeight;
		if(TempHeight < 0){
			__Down__(-TempHeight);
		}
		else if(TempHeight > 0){
			__Up__(TempHeight);
		}
		__Release__();
		
		
			/* 完成一次取放后，两轴回到原流程的逻辑零位。 */
			Mechanism_MovePairTo(0.0f, 0.0f);
	}
	else if(GroundGrab){//�ӵ����ȡ��鲢�ŵ�����
		switch(GroundGrab){
			case 1:
			GM6020Pos = Place1,Length = DropLen1;
			break;
			case 2:
			GM6020Pos = Place2,Length = DropLen2;
			break;
			case 3:
			GM6020Pos = Place3,Length = DropLen3;
			break;
			default:break;
		}
		TempHeight = StepCounter[0] - GroundHeight;
		if(TempHeight < 0){
			__Down__(-TempHeight);
		}
		else if(TempHeight > 0){
			__Up__(TempHeight);
		}
		vTaskDelay(250);
		__Grab__();
		__Up__(StepCounter[0]);
		GM6020ExAngle = GM6020Pos;
		while(1){
		if(fabs(PID_wheel_position[0].error_now) < 25)
			Stable1 ++;
		else
			Stable1 = 0;
		if(Stable1 > 5)
			break;
		vTaskDelay(1);
	}
		TempLength = StepCounter[1] - Length;
		if(TempLength < 0){
			__Length__(-TempLength);
		}
		else if(TempLength > 0){
			__Shorten__(TempLength);
		}
		__Down__(CarDropHeight);
		__Release__();
		vTaskDelay(150);
			Mechanism_MovePairTo(0.0f, 0.0f);
	}
	else if(CarGrab){//�ӳ���ȡ���
		switch(CarGrab){
			case 1:
			GM6020Pos = Place1,Length = DropLen1;
			break;
			case 2:
			GM6020Pos = Place2,Length = DropLen2;
			break;
			case 3:
			GM6020Pos = Place3,Length = DropLen3;
			break;
			default:break;
		}
		if(CarGrab != 1){
		TempHeight = StepCounter[0];
		if(TempHeight < 0){
			__Down__(-TempHeight);
		}
		else if(TempHeight > 0){
			__Up__(TempHeight);
	    }
		}
		GM6020ExAngle = GM6020Pos;
		while(1){
		if(fabs(PID_wheel_position[0].error_now) < 25)
			Stable1 ++;
		else
			Stable1 = 0;
		if(Stable1 > 5)
			break;
		vTaskDelay(1);
	}
		
		TempLength = StepCounter[1] - Length;
		if(TempLength < 0){
			__Length__(-TempLength);
		}
		else if(TempLength > 0){
			__Shorten__(TempLength);
		}
	
		TempHeight = StepCounter[0] - CarGrabHeight;
		if(TempHeight < 0){
			__Down__(-TempHeight);
		}
		else if(TempHeight > 0){
			__Up__(TempHeight);
		}
		
		vTaskDelay(250);
		__Grab__();
		if(CarGrab == 1){
		TempHeight = StepCounter[0] - CarDropHeight;
		if(TempHeight < 0){
			__Down__(-TempHeight);
		}
		else if(TempHeight > 0){
			__Up__(TempHeight);
		}
		}
		else{
			__Up__(StepCounter[0]);
		}
	}
	else if(PutPlace){//�����������/��������
		switch(PutPlace){
			case 1:
			Length = GroundHeight;
			break;
			case 2:
			Length = 1950;
			break;
			default:break;
		}
		
		TempHeight = StepCounter[0] - Length;
		if(TempHeight < 0) __Down__(-TempHeight);
		else if(TempHeight > 0) __Up__(TempHeight);
		__Release__();
		vTaskDelay(250);
		TempHeight = StepCounter[0] - CarDropHeight;
		if(TempHeight < 0) __Down__(-TempHeight);
		else if(TempHeight > 0) __Up__(TempHeight);
		
		__Shorten__(DropLen1);
		
	}
}

void PutDouble(){
		/* 原代码没有给Height赋值；该函数目前未被调用，先用安全的车辆放置高度。 */
		int16_t TempHeight,Height = CarDropHeight;
		
		TempHeight = StepCounter[0] - Height;
		if(TempHeight < 0){
			__Down__(-TempHeight);
		}
		else if(TempHeight > 0){
			__Up__(TempHeight);
		}
		__Release__();
		vTaskDelay(250);
		TempHeight = StepCounter[0] - CarDropHeight;
		if(TempHeight < 0){
			__Down__(-TempHeight);
		}
		else if(TempHeight > 0){
			__Up__(TempHeight);
		}

}

int8_t a = -90;

uint8_t AutoPosAim(int16_t Ex_x,int16_t Ex_y,int16_t* ex,int16_t* ey,uint8_t is_Valid)
{
	if(!is_Valid)
	{
		PID_Control(Ex_x * 10,a,&PID_wheel_speed[2]);
		
		*ex = PID_wheel_speed[2].pid_out;
		
		PID_Control(Ex_y * 10,900,&PID_wheel_speed[3]);
		
		*ey = PID_wheel_speed[3].pid_out;

		if(fabs(PID_wheel_speed[2].error_now) < 10 && 86 < Ex_y && Ex_y < 94)
			Stable++;
		else
			Stable = 0;

		if(Stable > 15)
		{
			Stable = 0;
			*ex = 0;
			return 1;
		}
		else
			return 0;
 	}
	else
		return 0;
}

//uint8_t AutoPosAimY(int16_t Ex_x,int16_t Ex_y,int16_t* ey){
//	 PID_Control(-Ex_y * 5,-90 * 5,&PID_wheel_speed[3]);
//	 *ey = -PID_wheel_speed[3].pid_out;
//	 if(86 < Ex_y && Ex_y < 94)
//		 Stable++;
//	 else
//		 Stable = 0;
//	 if(Stable > 1000){
//		 Stable = 0;
//		 *ey = 0;
//		 return 1;
//	 }
//	 else
//		 return 0;
//}

uint8_t AutoMove(M2006_TypeDef* motor,float* Last,float Des)
{
	float TempDes[4];
	TempDes[0] = motor[0].Angle_DEG - Last[0];
	TempDes[1] = motor[1].Angle_DEG - Last[1];
	TempDes[2] = motor[2].Angle_DEG - Last[2];
	TempDes[3] = motor[3].Angle_DEG - Last[3];
	if((fabs(TempDes[0]) + fabs(TempDes[1]) + fabs(TempDes[2]) + fabs(TempDes[3])) / 4 >= Des)
		return 1;
	else
		return 0;
}

/* ==================== 阅读导航 ====================
 * 下一步依次看 Robot/Src/PID.c、Robot/Src/Chassis.c、Robot/Src/motor.c，理解误差怎样变成电机命令。
 * ================================================== */
