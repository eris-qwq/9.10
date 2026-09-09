#ifndef __MY_CODE_H_
#define __MY_CODE_H_

#ifdef __cplusplus
extern "C" {
#endif

void setup(void);
void loop(void);	
void pub_msg(void);
typedef struct
{
 float Expect_Speed_X;
 float Expect_Speed_Y;
 float Expect_Speed_Yaw;
 int flag;
} Expect_Speed_Typedef;

typedef struct {
	int X;
	int Y;
	int Size;
	int Distance;
} Vision_Typedef;

extern Vision_Typedef Vision_Tx;
extern Expect_Speed_Typedef Expect_Tx;
	
#ifdef __cplusplus
}
#endif

#endif