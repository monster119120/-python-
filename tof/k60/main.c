
#include "common.h"
#include "include.h"
#include "math.h"
#include "project.h"
#include "picture.h"
#include "Speed_Control.h"

#define S3010_FTM   FTM0
#define S3010_CH    FTM_CH0                            //舵机初始化PTA
#define S3010_HZ    (100)


char STR[200]=0;
//int NUM[14]=0;
int num=0;
int PWM1=0;
int PWM2=0;
int PWM3=0;
int PWM4=0;

int straight;
int back;
int left;
int right; // 四个方向

void guide(int left,int right,int straight,int back);
void PWM_control();


void  main(void)//CNST
{
  int i=0;

    ftm_pwm_init(FTM0, FTM_CH0,30000,0);   //电机初始化
    ftm_pwm_init(FTM0, FTM_CH1,30000,0);
    ftm_pwm_init(FTM0, FTM_CH2,30000,0);   //电机初始化
    ftm_pwm_init(FTM0, FTM_CH3,30000,0);

//    ftm_pwm_duty(FTM0,FTM_CH0,700);
//    ftm_pwm_duty(FTM0,FTM_CH1,700);
//    ftm_pwm_duty(FTM0,FTM_CH2,700);
//    ftm_pwm_duty(FTM0,FTM_CH3,700);
    
    gpio_init(PTB0,GPO,0);                        //初始化转向
    gpio_init(PTB1,GPO,0);                      
    gpio_init(PTB2,GPO,0);                       
    gpio_init(PTB3,GPO,0);                      
    gpio_init(PTB4,GPO,0);                         
    gpio_init(PTB5,GPO,0);                         
    gpio_init(PTB6,GPO,0);                      
    gpio_init(PTB7,GPO,0);                      

 
    while(1)
    {

     char temps[4];
     temps[3]='\0';
    num = uart_querystr (VCAN_PORT, STR, 100);
      if( num != 0 )
          {
//            { for(i=0;i<14;i++)
//           
//              NUM[i] = i(STR[i]);
//              
//            }
               
            for(i=0;i<3;i++)
            {
              temps[i]=STR[i];
              
            }
            PWM1=atoi(temps);
            
             for(i=0;i<3;i++)
            {
              temps[i]=STR[i+3];
              
            }
            PWM2=atoi(temps);
             for(i=0;i<3;i++)
            {
              temps[i]=STR[i+6];
              
            }
            PWM3=atoi(temps);
            for(i=0;i<3;i++)
            {
              temps[i]=STR[i+9];
              
            }
            PWM4=atoi(temps);
            
        //    PWM_control();    调试用
            SpeedControl();
            }
          }
      
   
  
     // guide();
     DELAY_MS(10);
    }
    
    


//
//void guide(int left,int right,int straight,int back)
//{
//
//  if(left==1)
//  {
//    gpio_init(PTB0,GPO,1);                         // 1轮正转
//    gpio_init(PTB1,GPO,0);                         //
//    gpio_init(PTB2,GPO,1);                         // 2轮正转
//    gpio_init(PTB3,GPO,0);                         //
//    gpio_init(PTB4,GPO,0);                         // 3轮停
//    gpio_init(PTB5,GPO,0);                         //
//    gpio_init(PTB6,GPO,0);                         // 4轮停
//    gpio_init(PTB7,GPO,0);                         //
//  }
//  else if(right==1)
//  {
//    gpio_init(PTB0,GPO,0);                         // 1轮反转
//    gpio_init(PTB1,GPO,1);                         //
//    gpio_init(PTB2,GPO,0);                         // 2轮反转
//    gpio_init(PTB3,GPO,1);                         //
//    gpio_init(PTB4,GPO,0);                         // 3轮停
//    gpio_init(PTB5,GPO,0);                         //
//    gpio_init(PTB6,GPO,0);                         // 4轮停
//    gpio_init(PTB7,GPO,0);
//  }
//  else if(straight==1)
//  {
//    gpio_init(PTB0,GPO,0);                         // 1轮停
//    gpio_init(PTB1,GPO,0);                         //
//    gpio_init(PTB2,GPO,0);                         // 2轮停
//    gpio_init(PTB3,GPO,0);                         //
//    gpio_init(PTB4,GPO,1);                         // 3轮正转
//    gpio_init(PTB5,GPO,0);                         //
//    gpio_init(PTB6,GPO,1);                         // 4轮正转
//    gpio_init(PTB7,GPO,0);
//  }
//  else if(back==1)
//  {
//    gpio_init(PTB0,GPO,0);                         // 1轮停
//    gpio_init(PTB1,GPO,0);                         //
//    gpio_init(PTB2,GPO,0);                         // 2轮停
//    gpio_init(PTB3,GPO,0);                         //
//    gpio_init(PTB4,GPO,0);                         // 3轮反转
//    gpio_init(PTB5,GPO,1);                         //
//    gpio_init(PTB6,GPO,0);                         // 4轮反转
//    gpio_init(PTB7,GPO,1);
//  }
//}




/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */
void PWM_control()
{
    ftm_pwm_duty(FTM0,FTM_CH0,PWM1);
    ftm_pwm_duty(FTM0,FTM_CH1,PWM2);
    ftm_pwm_duty(FTM0,FTM_CH2,PWM3);
    ftm_pwm_duty(FTM0,FTM_CH3,PWM4);
  
}