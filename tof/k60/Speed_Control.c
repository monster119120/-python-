#include "include.h"
#include "math.h"
#include "project.h"

#include "Speed_Control.h"

#define QEP_time 5000   //编码器定时5ms

//时钟变量
unsigned long runtime = 0;

float Kp = 60;//167
float Ki = 25;//25
float Kd = 0;//25
float Ipart=0;



 int  x_error = 0,deltax=0,x_error1 =0 ;
 int  theta_error = 0,theta=0,theta_error1 =0 ;     
/********************
左右电机相关变量
********************/
//int MotorSpeed=80; //设定赛车的目标平均速度 
int NowSpeed;   //赛车的当前平均速度
int speed_forward;

int set_speed;
int now_speed;
int speed_error;
int speed_error1;
int speed_error2;
int PWM_out=0;
int p_quad[10]=0;
int p_velo_l;
int p_velo_r;
int p_velo_real;
/********************
冲出赛道标志位
*********************/
int stop_of_outroad = 0;



void SpeedControl(void)
{
    
     int PWM1_out=0;
     int PWM2_out=0;
     int PWM3_out=0;
     int PWM4_out=0;
      
   //   x_error2 = x_error1;
      x_error1 = x_error;
      x_error = deltax;
      
   //   theta_error2 = theta_error1;
      theta_error1= theta_error;
      theta_error = theta;
      
//      if(left_speed_error>=20)
//         left_PWM_out = 5000;
//      else if(left_speed_error<=-20)
//         left_PWM_out = 0;
//      else
//      x_Ipart += Ki*x_error;
//       if(x_Ipart>200)
//        x_Ipart = 200;
//       theta_Ipart += Ki*theta_error;
//       if(theta_Ipart>200)
//        theta_Ipart = 200;
  
     if(straight==1)
     {
      
      PTB0_OUT   = 1;
      PTB1_OUT   = 0;  //前向
      PTB6_OUT   = 1;                    
      PTB7_OUT   = 0;
      
      theta_error1= theta_error;
      theta_error = theta;
      
      PWM1_out =(int)(Kp*theta_error + Kd * (theta_error1- theta_error))/2+PWM1_centervalue;
      PWM4_out =-(int)(Kp*theta_error + Kd * (theta_error1- theta_error))/2+PWM4_centervalue;  //角度回正
      
      if(x_error>0)
      {
      PTB2_OUT   = 1;
      PTB3_OUT   = 0;
      PTB4_OUT   = 1;                    
      PTB5_OUT   = 0;
      
      PWM2_out =(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM2_least;
      PWM3_out =(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM3_least;  //位置回正 向左
      }
       if(x_error<0)
      {
      PTB2_OUT   = 0;
      PTB3_OUT   = 1;
      PTB4_OUT   = 0;
      PTB5_OUT   = 1;
      
      PWM2_out =-(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM2_least;
      PWM3_out =-(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM3_least;  //位置回正 向右
      }
      if(x_error==0&&x_error1==0)
      {
      PTB2_OUT   = 0;
      PTB3_OUT   = 0;
      PTB4_OUT   = 0;                    
      PTB5_OUT   = 0;
      }
      
     }
     if(back==1)
     {
      PTB0_OUT   = 0;
      PTB1_OUT   = 1;  //反向
      PTB6_OUT   = 0;                    
      PTB7_OUT   = 1;
       
      
      theta_error1= theta_error;
      theta_error = theta;
      
      PWM1_out =(int)(Kp*theta_error + Kd * (theta_error1-theta_error))/2+PWM1_centervalue;
      PWM4_out =-(int)(Kp*theta_error + Kd * (theta_error1-theta_error))/2+PWM4_centervalue;  //角度回正
      
      if(x_error>0)
      {
      PTB2_OUT   = 0;
      PTB3_OUT   = 1;
      PTB4_OUT   = 0;                    
      PTB5_OUT   = 1;
      
      PWM2_out =(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM2_least;
      PWM3_out =(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM3_least;  //位置回正 向右
      }
       if(x_error<0)
      {
      PTB2_OUT   = 1;
      PTB3_OUT   = 0;
      PTB4_OUT   = 1;
      PTB5_OUT   = 0;
      
      PWM2_out =-(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM2_least;
      PWM3_out =-(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM3_least;  //位置回正 向左
      }
      if(x_error==0&&x_error1==0)
      {
      PTB2_OUT   = 0;
      PTB3_OUT   = 0;
      PTB4_OUT   = 0;                    
      PTB5_OUT   = 0;
      }
      
     }
      if(left==1)
     {
      PTB2_OUT   = 1;
      PTB3_OUT   = 0;  //左向
      PTB4_OUT   = 1;                    
      PTB5_OUT   = 0;
       
      
      theta_error1= theta_error;
      theta_error = theta;
      
      PWM2_out =(int)(Kp*theta_error + Kd * (theta_error1-theta_error))/2+PWM2_centervalue;
      PWM3_out =-(int)(Kp*theta_error + Kd * (theta_error1-theta_error))/2+PWM3_centervalue;  //角度回正
      
      if(x_error>0)
      {
      PTB0_OUT   = 1;
      PTB1_OUT   = 0;
      PTB6_OUT   = 1;                    
      PTB7_OUT   = 0;
      
      PWM1_out =(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM1_least;
      PWM4_out =(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM4_least;  //位置回正 向右
      }
       if(x_error<0)
      {
      PTB0_OUT   = 0;
      PTB1_OUT   = 1;
      PTB6_OUT   = 0;
      PTB7_OUT   = 1;
      
      PWM1_out =-(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM1_least;
      PWM4_out =-(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM4_least;  //位置回正 向左
      }
      if(x_error==0&&x_error1==0)
      {
      PTB0_OUT   = 0;
      PTB1_OUT   = 0;
      PTB6_OUT   = 0;                    
      PTB7_OUT   = 0;
      }
      
     }
     if(right==1)
     {
      PTB2_OUT   = 0;
      PTB3_OUT   = 1;  //右向
      PTB4_OUT   = 0;                    
      PTB5_OUT   = 1;
       
      
      theta_error1= theta_error;
      theta_error = theta;
      
      PWM2_out =(int)(Kp*theta_error + Kd * (theta_error1-theta_error))/2+PWM2_centervalue;
      PWM3_out =-(int)(Kp*theta_error + Kd * (theta_error1-theta_error))/2+PWM3_centervalue;  //角度回正
      
      if(x_error>0)
      {
      PTB0_OUT   = 0;
      PTB1_OUT   = 1;
      PTB6_OUT   = 0;                    
      PTB7_OUT   = 1;
      
      PWM1_out =(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM1_least;
      PWM4_out =(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM4_least;  //位置回正 向右
      }
       if(x_error<0)
      {
      PTB0_OUT   = 1;
      PTB1_OUT   = 0;
      PTB6_OUT   = 1;
      PTB7_OUT   = 0;
      
      PWM1_out =-(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM1_least;
      PWM4_out =-(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM4_least;  //位置回正 向左
      }
      if(x_error==0&&x_error1==0)
      {
      PTB0_OUT   = 0;
      PTB1_OUT   = 0;
      PTB6_OUT   = 0;                    
      PTB7_OUT   = 0;
      }
      
     }

}    



/////////////////////////////////////////////
//卡尔曼滤波器

int16 FILTRATE_Kalman(int16 MsrValue)
{
static float PrioriErr = 0; //先验误差
static int16 FltValue = 0; //滤波值，作为滤波器输出，更接近真实值。
static float KalmanGain = 0.0; //卡尔曼增益
static float EstimateCov = 0.0; //估计协方差
static float MeasureCov = 0.20; //测量协方差
PrioriErr = EstimateCov + (float)0.05; //先验误差 = 估计协方差 + 过程方差
//计算卡尔曼增益
KalmanGain = PrioriErr / (PrioriErr + MeasureCov);
//计算本次滤波估计值 
//估计值 = 上次估计值 + Kalman 增益 * (测量值 - 上次估计值)
FltValue = (int16)(FltValue + KalmanGain * (MsrValue - FltValue));
//更新估计协方差
EstimateCov = (float)((1-KalmanGain)*PrioriErr);
return FltValue; //返回估计值，即滤波值
}

void ENCODER_GetVelo( int16 *p_quad,int16 *p_velo_l,int16 *p_velo_r,int *p_velo_real  )
{
//获取脉冲数
p_quad[9] = p_quad[8];
p_quad[8] = p_quad[7];
p_quad[7] = p_quad[6];
p_quad[6] = p_quad[5];
p_quad[5] = p_quad[4];
p_quad[4] = p_quad[3];
p_quad[3] = p_quad[2];
p_quad[2] = p_quad[1];
p_quad[1] = p_quad[0];
p_quad[0] = ftm_quad_get(FTM2); //获取 FTM 正交解码的脉冲数（负数表示反方向）
ftm_quad_clean(FTM2); //清 FTM 正交解码的脉冲数

// 滤波，速度估计
*p_velo_l = FILTRATE_Kalman(p_quad[0]);
*p_velo_r = (p_quad[0] + p_quad[1] +p_quad[2]+p_quad[3]+p_quad[4]+p_quad[5]+p_quad[6]+p_quad[7]+p_quad[8]+ p_quad[9]) /10;
//左右轮速度平均值 作为小车当前速度
*p_velo_real = *p_velo_r ;
}