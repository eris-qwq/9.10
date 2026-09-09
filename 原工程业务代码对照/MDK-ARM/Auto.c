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
		
	if(Ex_y > Dead && StepCounter[1] <= 6400){
		Dir[1] = 1;
		__StepMotor2_Dir(0);
			TIM1->CCR2 = 39;
	}
	else if(Ex_y < -Dead && StepCounter[1] >= 0){
		Dir[1] = -1;
		__StepMotor2_Dir(1);
			TIM1->CCR2 = 39;
	}
	else{
		TIM1->CCR2 = 0;
	}
	
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
		 
	if(Ex_y > Dead && StepCounter[1] <= 6400){
		Dir[1] = 1;
		__StepMotor2_Dir(0);
			TIM1->CCR2 = 39;
	}
	else if(Ex_y < -Dead && StepCounter[1] >= 0){
		Dir[1] = -1;
		__StepMotor2_Dir(1);
			TIM1->CCR2 = 39;
	}
	else{
		TIM1->CCR2 = 0;
	}
	
	
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
	int16_t GM6020Position = 0,Temp0,Temp1,TempLength = 0,Stable0 = 0;
	switch(Color){
	case RED:	GM6020Position = RedAngle;TempLength = RedLengt;
	break;
	case GREEN: GM6020Position = GreenAngle;TempLength = GreenLengt;
	break;
	case BLUE:  GM6020Position = BlueAngle;TempLength = BlueLengt;
	break;
	default:break;
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
	Temp1 = TempLength - StepCounter[1];
	if(height == 1)
	Temp0 = GroundHeight - StepCounter[0];
	else if(height == 2)
	Temp0 = Put2Height - StepCounter[0];
	if(Temp0 < 0){
			__StepMotor1_Dir(0);
			Dir[0] = -1;
			Temp0 = -Temp0;
		}
		else if(Temp0 > 0){
			__StepMotor1_Dir(1);
			Dir[0] = 1;
		}
		
		if(Temp1 > 0){
			__StepMotor2_Dir(0);
			Dir[1] = 1;
		}
		else if(Temp1 < 0){
			__StepMotor2_Dir(1);
			Dir[1] = -1;
			Temp1 = -Temp1;
		}
	
	if(Temp0 - Temp1 < 0){
		TIM1->CCR1 = 39,TIM1->CCR2 = 39;
		vTaskDelay(Temp0);
		TIM1->CCR1 = 0;
		vTaskDelay(Temp1 - Temp0);
		TIM1->CCR2 = 0;
	}
	else if(Temp0 - Temp1 > 0){
		TIM1->CCR1 = 39,TIM1->CCR2 = 39;
		vTaskDelay(Temp1);
		TIM1->CCR2 = 0;
		vTaskDelay(Temp0 - Temp1);
		TIM1->CCR1 = 0;
	}
}

void AutoArmGrabDrop(uint8_t GrabPlace1,uint8_t GroundGrab,uint8_t CarGrab,uint8_t PutPlace){
	int32_t GM6020Pos = 0,Length = 0,TempLength = 0,TempHeight = 0,TempPutHeight = 0,Stable1 = 0;
	if(GrabPlace1){//从托盘取物块并放置到车上
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
		
		
//		if(GrabPlace1 != 1){
			__StepMotor1_Dir(0);
			__StepMotor2_Dir(1);
			Dir[0] = -1;
			Dir[1] = -1;
			if((StepCounter[0] - StepCounter[1]) > 0){
				TIM1->CCR1 = 39;
				TIM1->CCR2 = 39;
				vTaskDelay(StepCounter[1]);
				TIM1->CCR2 = 0;
				vTaskDelay(StepCounter[0] - StepCounter[1]);
				TIM1->CCR1 = 0;
			}
			else if((StepCounter[0] - StepCounter[1]) < 0){
				TIM1->CCR1 = 39;
				TIM1->CCR2 = 39;
				vTaskDelay(StepCounter[0]);
				TIM1->CCR1 = 0;
				vTaskDelay(StepCounter[1] - StepCounter[0]);
				TIM1->CCR2 = 0;
			}
//		}
	}
	else if(GroundGrab){//从地面夹取物块并放到车上
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
		//if(GroundGrab != 1){
			__StepMotor1_Dir(0);
			__StepMotor2_Dir(1);
			Dir[0] = -1;
			Dir[1] = -1;
			TIM1->CCR1 = 39;
			TIM1->CCR2 = 39;
			if(Length - CarDropHeight > 0){
				vTaskDelay(CarDropHeight);
				TIM1->CCR1 = 0;
				vTaskDelay(Length - CarDropHeight);
				TIM1->CCR2 = 0;
			}
			else if(Length - CarDropHeight < 0){
				vTaskDelay(Length);
				TIM1->CCR2 = 0;
				vTaskDelay(CarDropHeight - Length);
				TIM1->CCR1 = 0;
			}
		//}
	}
	else if(CarGrab){//从车里取物块
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
	else if(PutPlace){//向地面放置物块/叠加物料
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
		if(TempHeight < 0){
					   __StepMotor1_Dir(1);		 
					   Dir[0] = 1;				 
					   htim1.Instance->CCR1 = 39;
					   vTaskDelay(-TempHeight);  	 		 
					   htim1.Instance->CCR1 = 0; 
		}
		else if(TempHeight > 0){
					   __StepMotor1_Dir(0);		 
					   Dir[0] = -1;				 
					   htim1.Instance->CCR1 = 39;
					   vTaskDelay(TempHeight);  	 		 
					   htim1.Instance->CCR1 = 0; 
		}
		__Release__();
		vTaskDelay(250);
		TempHeight = StepCounter[0] - CarDropHeight;
		if(TempHeight < 0){
						__StepMotor1_Dir(1);		 
					   Dir[0] = 1;				 
					   htim1.Instance->CCR1 = 39;
					   vTaskDelay(-TempHeight);  	 		 
					   htim1.Instance->CCR1 = 0; 
		}
		else if(TempHeight > 0){
					   __StepMotor1_Dir(0);		 
					   Dir[0] = -1;				 
					   htim1.Instance->CCR1 = 39;
					   vTaskDelay(TempHeight);  	 		 
					   htim1.Instance->CCR1 = 0; 
		}
		
		__Shorten__(DropLen1);
		
	}
}

void PutDouble(){
		int16_t TempHeight,Height;
		
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
uint8_t AutoPosAim(int16_t Ex_x,int16_t Ex_y,int16_t* ex,int16_t* ey,uint8_t is_Valid){
	if(!is_Valid){
	 PID_Control(Ex_x * 10,a,&PID_wheel_speed[2]);
	
	 *ex = PID_wheel_speed[2].pid_out;
	
	 PID_Control(Ex_y * 10,900,&PID_wheel_speed[3]);
	
	 *ey = PID_wheel_speed[3].pid_out;

	 if(fabs(PID_wheel_speed[2].error_now) < 10 && 86 < Ex_y && Ex_y < 94)
		 Stable++;
	 else
		 Stable = 0;
	 if(Stable > 15){
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

uint8_t AutoMove(M2006_TypeDef* motor,float* Last,float Des){
	float TempDes[4];
	TempDes[0] = motor[0].Angle_DEG - Last[0];TempDes[1] = motor[1].Angle_DEG - Last[1];
	TempDes[2] = motor[2].Angle_DEG - Last[2];TempDes[3] = motor[3].Angle_DEG - Last[3];
	if((fabs(TempDes[0]) + fabs(TempDes[1]) + fabs(TempDes[2]) + fabs(TempDes[3])) / 4 > Des)
		return 1;
	else
		return 0;
}
