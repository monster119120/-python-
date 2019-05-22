#include "include.h"
#include "math.h"
#include "project.h"

#include "Speed_Control.h"

#define QEP_time 5000   //��������ʱ5ms

//ʱ�ӱ���
unsigned long runtime = 0;

float Kp = 60;//167
float Ki = 25;//25
float Kd = 0;//25
float Ipart=0;



 int  x_error = 0,deltax=0,x_error1 =0 ;
 int  theta_error = 0,theta=0,theta_error1 =0 ;     
/********************
���ҵ����ر���
********************/
//int MotorSpeed=80; //�趨������Ŀ��ƽ���ٶ� 
int NowSpeed;   //�����ĵ�ǰƽ���ٶ�
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
���������־λ
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
      PTB1_OUT   = 0;  //ǰ��
      PTB6_OUT   = 1;                    
      PTB7_OUT   = 0;
      
      theta_error1= theta_error;
      theta_error = theta;
      
      PWM1_out =(int)(Kp*theta_error + Kd * (theta_error1- theta_error))/2+PWM1_centervalue;
      PWM4_out =-(int)(Kp*theta_error + Kd * (theta_error1- theta_error))/2+PWM4_centervalue;  //�ǶȻ���
      
      if(x_error>0)
      {
      PTB2_OUT   = 1;
      PTB3_OUT   = 0;
      PTB4_OUT   = 1;                    
      PTB5_OUT   = 0;
      
      PWM2_out =(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM2_least;
      PWM3_out =(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM3_least;  //λ�û��� ����
      }
       if(x_error<0)
      {
      PTB2_OUT   = 0;
      PTB3_OUT   = 1;
      PTB4_OUT   = 0;
      PTB5_OUT   = 1;
      
      PWM2_out =-(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM2_least;
      PWM3_out =-(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM3_least;  //λ�û��� ����
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
      PTB1_OUT   = 1;  //����
      PTB6_OUT   = 0;                    
      PTB7_OUT   = 1;
       
      
      theta_error1= theta_error;
      theta_error = theta;
      
      PWM1_out =(int)(Kp*theta_error + Kd * (theta_error1-theta_error))/2+PWM1_centervalue;
      PWM4_out =-(int)(Kp*theta_error + Kd * (theta_error1-theta_error))/2+PWM4_centervalue;  //�ǶȻ���
      
      if(x_error>0)
      {
      PTB2_OUT   = 0;
      PTB3_OUT   = 1;
      PTB4_OUT   = 0;                    
      PTB5_OUT   = 1;
      
      PWM2_out =(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM2_least;
      PWM3_out =(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM3_least;  //λ�û��� ����
      }
       if(x_error<0)
      {
      PTB2_OUT   = 1;
      PTB3_OUT   = 0;
      PTB4_OUT   = 1;
      PTB5_OUT   = 0;
      
      PWM2_out =-(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM2_least;
      PWM3_out =-(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM3_least;  //λ�û��� ����
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
      PTB3_OUT   = 0;  //����
      PTB4_OUT   = 1;                    
      PTB5_OUT   = 0;
       
      
      theta_error1= theta_error;
      theta_error = theta;
      
      PWM2_out =(int)(Kp*theta_error + Kd * (theta_error1-theta_error))/2+PWM2_centervalue;
      PWM3_out =-(int)(Kp*theta_error + Kd * (theta_error1-theta_error))/2+PWM3_centervalue;  //�ǶȻ���
      
      if(x_error>0)
      {
      PTB0_OUT   = 1;
      PTB1_OUT   = 0;
      PTB6_OUT   = 1;                    
      PTB7_OUT   = 0;
      
      PWM1_out =(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM1_least;
      PWM4_out =(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM4_least;  //λ�û��� ����
      }
       if(x_error<0)
      {
      PTB0_OUT   = 0;
      PTB1_OUT   = 1;
      PTB6_OUT   = 0;
      PTB7_OUT   = 1;
      
      PWM1_out =-(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM1_least;
      PWM4_out =-(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM4_least;  //λ�û��� ����
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
      PTB3_OUT   = 1;  //����
      PTB4_OUT   = 0;                    
      PTB5_OUT   = 1;
       
      
      theta_error1= theta_error;
      theta_error = theta;
      
      PWM2_out =(int)(Kp*theta_error + Kd * (theta_error1-theta_error))/2+PWM2_centervalue;
      PWM3_out =-(int)(Kp*theta_error + Kd * (theta_error1-theta_error))/2+PWM3_centervalue;  //�ǶȻ���
      
      if(x_error>0)
      {
      PTB0_OUT   = 0;
      PTB1_OUT   = 1;
      PTB6_OUT   = 0;                    
      PTB7_OUT   = 1;
      
      PWM1_out =(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM1_least;
      PWM4_out =(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM4_least;  //λ�û��� ����
      }
       if(x_error<0)
      {
      PTB0_OUT   = 1;
      PTB1_OUT   = 0;
      PTB6_OUT   = 1;
      PTB7_OUT   = 0;
      
      PWM1_out =-(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM1_least;
      PWM4_out =-(int)(Kp*x_error + Kd * (theta_error1- theta_error))+PWM4_least;  //λ�û��� ����
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
//�������˲���

int16 FILTRATE_Kalman(int16 MsrValue)
{
static float PrioriErr = 0; //�������
static int16 FltValue = 0; //�˲�ֵ����Ϊ�˲�����������ӽ���ʵֵ��
static float KalmanGain = 0.0; //����������
static float EstimateCov = 0.0; //����Э����
static float MeasureCov = 0.20; //����Э����
PrioriErr = EstimateCov + (float)0.05; //������� = ����Э���� + ���̷���
//���㿨��������
KalmanGain = PrioriErr / (PrioriErr + MeasureCov);
//���㱾���˲�����ֵ 
//����ֵ = �ϴι���ֵ + Kalman ���� * (����ֵ - �ϴι���ֵ)
FltValue = (int16)(FltValue + KalmanGain * (MsrValue - FltValue));
//���¹���Э����
EstimateCov = (float)((1-KalmanGain)*PrioriErr);
return FltValue; //���ع���ֵ�����˲�ֵ
}

void ENCODER_GetVelo( int16 *p_quad,int16 *p_velo_l,int16 *p_velo_r,int *p_velo_real  )
{
//��ȡ������
p_quad[9] = p_quad[8];
p_quad[8] = p_quad[7];
p_quad[7] = p_quad[6];
p_quad[6] = p_quad[5];
p_quad[5] = p_quad[4];
p_quad[4] = p_quad[3];
p_quad[3] = p_quad[2];
p_quad[2] = p_quad[1];
p_quad[1] = p_quad[0];
p_quad[0] = ftm_quad_get(FTM2); //��ȡ FTM �����������������������ʾ������
ftm_quad_clean(FTM2); //�� FTM ���������������

// �˲����ٶȹ���
*p_velo_l = FILTRATE_Kalman(p_quad[0]);
*p_velo_r = (p_quad[0] + p_quad[1] +p_quad[2]+p_quad[3]+p_quad[4]+p_quad[5]+p_quad[6]+p_quad[7]+p_quad[8]+ p_quad[9]) /10;
//�������ٶ�ƽ��ֵ ��ΪС����ǰ�ٶ�
*p_velo_real = *p_velo_r ;
}