/*
  ******************************************************************************
  * @file    main.c
  * @author  dingxu
  * @version V1.0
  * @date    2018-05-17
  * @brief   �¿�
  ******************************************************************************
  */

#include "stm8l15x.h"//STM8L051/151���ÿ⺯��
#include <stdlib.h>
#include "math.h"  
#include "stdbool.h"

//����LED�������˿�
#define LED_PORT  GPIOA
#define LED_PINS  GPIO_Pin_2
#define LED1_PORT  GPIOA
#define LED1_PINS  GPIO_Pin_3
#define HOT_PORT  GPIOD
#define HOT_PINS  GPIO_Pin_0

#define KEY_PORT  GPIOB
#define KEY_PINS  GPIO_Pin_1

#define KEY_PORTADD  GPIOB
#define KEY_PINSADD  GPIO_Pin_2

                 // 0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19               28
//�¶�����
//int TemperTable[]= {100,95,80,53,45,40,36,32,29,26,24,22,20,19,17,16,15,13,12,9,8,6,5,4,3,2,1,0,0,-1,-2,-2,-3,-4,-4,-5,-6,-6,-7,-7,-8,-9,-10,-11,-12,-13,-14};

//        MF52E 10K at 25, B = 3950, ADC = 12 bits
u16 temp_table[]={
                1905,        //;-40        0
                1834,        //;-39        1
                1756,        //;-38        2
                1676,        //;-37        3
                1595,        //;-36        4
                1515,        //;-35        5
                1438,        //;-34        6
                1364,        //;-33        7
                1293,        //;-32        8
                1226,        //;-31        9
                1163,        //;-30        10
                1104,        //;-29        11
                1048,        //;-28        12
                995,        //;-27        13
                946,        //;-26        14
                900,        //;-25        15
                856,        //;-24        16
                815,        //;-23        17
                777,        //;-22        18
                740,        //;-21        19
                705,        //;-20        20
                672,        //;-19        21
                641,        //;-18        22
                612,        //;-17        23
                584,        //;-16        24
                557,        //;-15        25
                531,        //;-14        26
                507,        //;-13        27
                484,        //;-12        28
                462,        //;-11        29
                441,        //;-10        30
                421,        //;-9        31
                402,        //;-8        32
                383,        //;-7        33
                366,        //;-6        34
                350,        //;-5        35
                334,        //;-4        36
                320,        //;-3        37
                306,        //;-2        38
                292,        //;-1        39
                280,        //;0        40
                268,        //;1        41
                256,        //;2        42
                246,        //;3        43
                236,        //;4        44
                226,        //;5        45
                217,        //;6        46
                208,        //;7        47
                200,        //;8        48
                192,        //;9        49
                185,        //;10        50
                184,        //;11        51
                181,        //;12        52
                176,        //;13        53
                169,        //;14        54
                162,        //;15        55
                155,        //;16        56
                147,        //;17        57
                140,        //;18        58
                133,        //;19        59
                126,        //;20        60
                120,        //;21        61
                114,        //;22        62
                109,        //;23        63
                104,        //;24        64
                100,        //;25        65
                95,        //;26        66
                91,        //;27        67
                88,        //;28        68
                84,        //;29        69
                81,        //;30        70
                78,        //;31        71
                75,        //;32        72
                73,        //;33        73
                70,        //;34        74
                68,        //;35        75
                65,        //;36        76
                63,        //;37        77
                61,        //;38        78
                59,        //;39        79
                57,        //;40        80
                55,        //;41        81
                53,        //;42        82
                51,        //;43        83
                49,        //;44        84
                48,        //;45        85
                46,        //;46        86
                45,        //;47        87
                43,        //;48        88
                42,        //;49        89
                40,        //;50        90
                39,        //;51        91
                37,        //;52        92
                36,        //;53        93
                35,        //;54        94
                34,        //;55        95
                32,        //;56        96
                31,        //;57        97
                30,        //;58        98
                29,        //;59        99
                28,        //;60        100
                27         //;61        101 
};


//�������ʾ
uint8_t HexTable[]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

#define uchar  unsigned char  
 unsigned char  fseg[]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90};
 unsigned char  segbit[]={0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};
 unsigned char  disbuf[4]={0,0,0,0};
 
void LED4_Display (void);			// LED��ʾ
void LED_OUT(uchar X);				// LED���ֽڴ�����λ����

unsigned char LED_0F[];		// LED��ģ��
 
unsigned char  LED_0F[] = 
{// 0	 1	  2	   3	4	 5	  6	   7	8	 9	  A	   b	C    d	  E    F    -
	0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90,0x8C,0xBF,0xC6,0xA1,0x86,0xFF,0xbf
};

uchar LED[8];	//����LED��8λ��ʾ����

#define DIO_PORT  GPIOB
#define DIO_PINS  GPIO_Pin_3

#define RCLK_PORT  GPIOB
#define RCLK_PINS  GPIO_Pin_4

#define SCLK_PORT  GPIOB
#define SCLK_PINS  GPIO_Pin_5

#define DIOHigh  GPIO_SetBits(DIO_PORT, DIO_PINS)
#define DIOLow   GPIO_ResetBits(DIO_PORT, DIO_PINS)

#define RCLKHigh  GPIO_SetBits(RCLK_PORT, RCLK_PINS)
#define RCLKLow   GPIO_ResetBits(RCLK_PORT, RCLK_PINS)

#define SCLKHigh  GPIO_SetBits(SCLK_PORT, SCLK_PINS)
#define SCLKLow   GPIO_ResetBits(SCLK_PORT, SCLK_PINS)

/*******************************************************************************
****��ڲ�������
****���ڲ�������
****������ע������ȷ��ʱ����
*******************************************************************************/
void Delay(__IO uint16_t nCount)
{
    /* Decrement nCount value */
    while (nCount != 0)
    {
        nCount--;
    }
}
/*******************************************************************************
****��ڲ�������Ҫ���͵��ַ���
****���ڲ�������
****������ע��USART���ͺ���
*******************************************************************************/
void USART1_SendStr(unsigned char *Str) 
{
        while(*Str!=0)//��Ϊ����
        {
            USART_SendData8(USART1,*Str);     //�������� 
            while(!USART_GetFlagStatus (USART1,USART_FLAG_TXE));//�ȴ��������
            Str++;//��һ������
        }
}
/*******************************************************************************
****��ڲ�������Ҫ���͵�16������
****���ڲ�������
****������ע��USART����16���ƺ���
*******************************************************************************/
void USART1_SendHex(unsigned char dat)
{
      USART_SendData8(USART1,'0');
      while(!USART_GetFlagStatus (USART1,USART_FLAG_TXE));//�ȴ��������
      USART_SendData8(USART1,'x');
      while(!USART_GetFlagStatus (USART1,USART_FLAG_TXE));//�ȴ��������
      USART_SendData8(USART1,HexTable[dat>>4]);
      while(!USART_GetFlagStatus (USART1,USART_FLAG_TXE));//�ȴ��������
      USART_SendData8(USART1,HexTable[dat&0x0f]);
      while(!USART_GetFlagStatus (USART1,USART_FLAG_TXE));//�ȴ��������
      USART_SendData8(USART1,' ');
      while(!USART_GetFlagStatus (USART1,USART_FLAG_TXE));//�ȴ��������
}

//----------------------------------------------------------
//adcת��
void  ChangeAD()
{

   ADC_SoftwareStartConv (ADC1);//�������ת��
              
   while(!ADC_GetFlagStatus (ADC1,ADC_FLAG_EOC));//�ȴ�ת������
       ADC_ClearFlag (ADC1,ADC_FLAG_EOC);//�����ر�ʶ
	
}

//------------------------------------------------------------
//��ӡ16����
void DisplayData(u16  data)
{
     USART1_SendHex((data>>8));  
     USART1_SendHex((data&0xff));
     USART1_SendStr("\r\n");

}

//---------------------------------------------------------------------------
//�������ʾ

void LED4_Display (void)
{
	unsigned char *led_table;          // ���ָ��
	uchar i;
	//��ʾ��3λ
	led_table = LED_0F + LED[2];
	i = *led_table;

	LED_OUT(i);	
	LED_OUT(0x01);		

	RCLKLow;
	RCLKHigh;
        
       //Delay(1000);
        
	
	//��ʾ��2λ
	led_table = LED_0F + LED[1];
	i = *led_table;

	LED_OUT(i);		
	LED_OUT(0x02);		
	
	RCLKLow;
	RCLKHigh;
       // Delay(1000);
        
               
	//��ʾ��1λ
	led_table = LED_0F + LED[0];
	i = *led_table;

	LED_OUT(i);			
	LED_OUT(0x04);	

	RCLKLow;
	RCLKHigh;
        //Delay(1000);
        
        
#if 0 
	//��ʾ��4λ
	led_table = LED_0F + LED[0];
	i = *led_table;

	LED_OUT(i);			
	LED_OUT(0x08);		

	RCLKLow;
	RCLKHigh;
#endif
        //Delay(50000);

}

//---------------------------------------------------------------------------
//�������ʾoff

void LED4_DisplayOff (void)
{
	unsigned char *led_table;          // ���ָ��
	uchar i;
	//��ʾ��3λ
	led_table = LED_0F + LED[2];
	i = *led_table;

	LED_OUT(i);	
	LED_OUT(0x00);		

	RCLKLow;
	RCLKHigh;
        
       //Delay(1000);
        
	
	//��ʾ��2λ
	led_table = LED_0F + LED[1];
	i = *led_table;

	LED_OUT(i);		
	LED_OUT(0x00);		
	
	RCLKLow;
	RCLKHigh;
       // Delay(1000);
        
               
	//��ʾ��1λ
	led_table = LED_0F + LED[0];
	i = *led_table;

	LED_OUT(i);			
	LED_OUT(0x00);	

	RCLKLow;
	RCLKHigh;
        //Delay(1000);
        

}
//------------------------------------------
//74HC595��ÿһ����λ

void LED_OUT(uchar X)
{
	uchar i;
	for(i=8;i>=1;i--)
	{
		if (X&0x80) DIOHigh; else DIOLow;
		X<<=1;
		SCLKHigh;
		SCLKLow;
	}
}


//----------------------------------------
//
void  DisplayTemperInsider(int u16_adc1_value)
{
        
     if (u16_adc1_value > 0)
     LED[0] = 0;
     else
     LED[0] = 16;
     
     u16_adc1_value = abs(u16_adc1_value);
     
     LED[1] = u16_adc1_value/10;
     LED[2] = u16_adc1_value%10;
     
}

//------------------------------------------
//��Ͳ�¶�
void DisplayTemperSide(int u16_adc2_value)
{
        
     if (u16_adc2_value > 0)
     LED[0] = 0;
     else
     LED[0] = 16;

     u16_adc2_value = abs(u16_adc2_value);
     LED[1] = u16_adc2_value/10;
     LED[2] = u16_adc2_value%10;

}

//-----------------------------------------------
//��ֵ�¶�
void DisplayTemperDiff(u8 DifTemp)
{
      LED[0] = 0;
       
     LED[1] = DifTemp/10;
     LED[2] = DifTemp%10;

}

//------------------------------------------------
//�����¶�
u8  FindDex(u16 TemResister)
{
  u8 i = 0;
  //DisplayData(temp_table[49]);
  for (i = 1;i < 102; i++ )
  {
    if ( (TemResister <= temp_table[i-1]) && (TemResister >= temp_table[i]))
    {     
	  //DisplayData(temp_table[i-1]);
      break;
    } 
  }  
  //DisplayData(i);
  return i;
}



//-------------------------------------------------
//��ʱ��2���ã�1�������һ���жϣ�����ϵͳ��ʱ

void TIM2_Init(void)   

{ 
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM2,ENABLE);//����ʱ���ź��͸���ʱ��4(Lϵ�е�Ƭ������)
  TIM2_TimeBaseInit(TIM2_Prescaler_16,TIM2_CounterMode_Up,100);            //100us   16M/16*100
  TIM2_SetCounter(250);                                //�Ĵ����洢��ʼֵ
  TIM2_ITConfig(TIM2_IT_Update,ENABLE);
  TIM2_ARRPreloadConfig(ENABLE);
  TIM2_Cmd(ENABLE);                                    //������ʹ�ܣ���ʼ����   
}
/*******************************************************************************
****����˵����������
****��ڲ���:��
****���ڲ���:��
****������ע:PB0(adc1-18)��ΪADC����ڣ�����ͨ���Ű��߽�3.3V��GND�����˿ڣ�ADCת�����ͨ��
             USART���
********************************************************************************/

extern u16 CounterDisplay;
extern bool FlagDate;
extern u16 CounterFlag;
void main(void)
{
  u16 u16_adc1_value; 
  u16 u16_adc2_value;
  u8 DifTemp = 0;
  u8 SetTemp = 0;
  int temperInsider;
  int temperOutsider;
 
  
  GPIO_Init(LED_PORT,LED_PINS,GPIO_Mode_Out_PP_Low_Slow);//��ʼ��LED�˿�
  GPIO_Init(LED1_PORT,LED1_PINS,GPIO_Mode_Out_PP_Low_Slow);//��ʼ��LED�˿�
  
  GPIO_Init(HOT_PORT,HOT_PINS,GPIO_Mode_Out_PP_Low_Slow);
  
  GPIO_Init(KEY_PORT,KEY_PINS,GPIO_Mode_In_PU_No_IT);//��ʼ��KEY�˿ڣ����������룬�����ж�
  GPIO_Init(KEY_PORTADD,KEY_PINSADD,GPIO_Mode_In_PU_No_IT);
  
  GPIO_Init(DIO_PORT,DIO_PINS,GPIO_Mode_Out_PP_High_Fast);//��ʼ��DIO�˿�
  GPIO_Init(RCLK_PORT,RCLK_PINS,GPIO_Mode_Out_PP_High_Fast);//��ʼ��RCLK�˿�
  GPIO_Init(SCLK_PORT,SCLK_PINS,GPIO_Mode_Out_PP_High_Fast);//��ʼ��RCLK�˿�
  
  //SYSCFG_REMAPPinConfig(REMAP_Pin_USART1TxRxPortA,ENABLE);//�˿���ӳ�䣬ȥ��ע��֮��USART1ΪPA2-TX��PA3-RX��ע��֮��USART1ΪTX-PC5��RX-PC6����λ֮��USART���Զ��ָ���PC5��PC6
  
  CLK_PeripheralClockConfig (CLK_Peripheral_USART1,ENABLE);//����USARTʱ�� 
  USART_Init(USART1,9600,USART_WordLength_8b,USART_StopBits_1,USART_Parity_No,USART_Mode_Tx|USART_Mode_Rx);//����USART����9600��8N1������/����
  USART_ITConfig (USART1,USART_IT_RXNE,ENABLE);//ʹ�ܽ����ж�
  USART_Cmd (USART1,ENABLE);//ʹ��USART
  
  CLK_PeripheralClockConfig (CLK_Peripheral_ADC1,ENABLE);//����ADCʱ��
  ADC_Init (ADC1,ADC_ConversionMode_Single,ADC_Resolution_12Bit,ADC_Prescaler_1);//ADC1�����β�����12λ��1��Ƶ
  ADC_Cmd(ADC1,ENABLE);//ADC1ʹ��
  //ADC_ChannelCmd (ADC1,ADC_Channel_18,ENABLE);//ADC1 18ͨ��ʹ��
  
   LED[0] = 0;
   LED[1] = 0;
   LED[2] = 0;
   
   TIM2_Init();
   
   asm("rim"); //���ж�
   
  while (1)
  {
       #if 1
    
       ADC_ChannelCmd (ADC1,ADC_Channel_18,ENABLE);//ADC1 18ͨ��ʹ��
       USART1_SendStr("ADC1ת�����Ϊ: ");
       ChangeAD();
       u16_adc1_value=ADC_GetConversionValue (ADC1);//��ȡת��ֵ
       ADC_ChannelCmd (ADC1,ADC_Channel_18,DISABLE);//ADC1 18ͨ��ʹ��

       //Delay(0xFFFF);
       u16_adc1_value = (u16_adc1_value*33000)>>12;//��ѹ����10��
       u16_adc1_value = u16_adc1_value/1000;
       u16_adc1_value = (100*u16_adc1_value)/(33-u16_adc1_value); //����ֵ
       //DisplayData(u16_adc1_value);
       temperInsider = FindDex(u16_adc1_value) - 40;
 
             
       ADC_ChannelCmd (ADC1,ADC_Channel_4,ENABLE);//ADC1 17ͨ��ʹ��
       USART1_SendStr("ADC2ת�����Ϊ: "); 
       ChangeAD();
       u16_adc2_value=ADC_GetConversionValue (ADC1);//��ȡת��ֵ
       ADC_ChannelCmd (ADC1,ADC_Channel_4,DISABLE);//ADC1 17ͨ��ʹ��

       //Delay(0xFFFF);
       u16_adc2_value = (u16_adc2_value*33000)>>12;//��ѹ����10��
       u16_adc2_value = u16_adc2_value/1000;
       u16_adc2_value = (100*u16_adc2_value)/(33-u16_adc2_value);//����ֵ
      // DisplayData(u16_adc2_value);
       temperOutsider = FindDex(u16_adc2_value) - 40;
       #endif

       
       
       if (GPIO_ReadInputDataBit(KEY_PORT,KEY_PINS)==0)//��GPB1����״̬
       {
           Delay(4000);  //�������,20ms   
           if (GPIO_ReadInputDataBit(KEY_PORT,KEY_PINS)==0)
           {
               while (GPIO_ReadInputDataBit(KEY_PORT,KEY_PINS)==0);
                GPIO_ToggleBits(LED_PORT, LED_PINS);//��תLED���״̬
                CounterFlag = 0;
               if (FlagDate && (SetTemp < 30))
                	SetTemp++;
                else if(FlagDate && (SetTemp >= 30))
                 	SetTemp = 0;
           }             
       }
     
            
       
       if (GPIO_ReadInputDataBit(KEY_PORTADD,KEY_PINSADD)==0)//��GPB1����״̬
       {          
            
             Delay(4000);  //�������,20ms
             if(GPIO_ReadInputDataBit(KEY_PORTADD,KEY_PINSADD)==0)  //��GPB1����״̬
             {
                while (GPIO_ReadInputDataBit(KEY_PORTADD,KEY_PINSADD)==0);
               
               GPIO_ToggleBits(LED1_PORT, LED1_PINS);//��תLED���״̬ 
               CounterFlag = 0;
		if ( FlagDate && (SetTemp > 0))
                	SetTemp--;
                else if ((FlagDate) && (SetTemp <= 0))
                	SetTemp = 30;
             }
               
       }
        
 
      if ((CounterDisplay <= 800) && (CounterDisplay >= 500))
      {
          
          if (FlagDate == false)
            DisplayTemperInsider(temperInsider);
  
      }
      else if((CounterDisplay <= 2500) && (CounterDisplay >= 2200))
     {
          //��ʾ��Ͳ�¶�
            if (FlagDate == false)
              DisplayTemperSide(temperOutsider);

      }
      
      if (FlagDate == true)
            DisplayTemperDiff(SetTemp);
      
      DifTemp = temperInsider + SetTemp;
      if (DifTemp > temperOutsider)
        GPIO_SetBits(HOT_PORT, HOT_PINS); 
      else
        GPIO_ResetBits(HOT_PORT, HOT_PINS); 
  
  }
}





#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
