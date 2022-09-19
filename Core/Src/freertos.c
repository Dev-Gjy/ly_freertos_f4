/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */


volatile uint8_t drive_Switch=0x77;         //驱动板开关机：开0x77；关0x88
volatile float brightness=0;                //LED灯输出亮度0.0-100.0%
volatile uint16_t color_Temperature=4600;      //LED灯输出色温2700k-6500k
volatile uint8_t fan_Ratio=0x64;              //风扇输出参数范围0x00-0x64对应DC0-12V
volatile uint8_t drive_TxData[16]={0};      //发送给驱动板的数组
volatile uint8_t drive_RxData[16]={0};      //从驱动板接受的数组

volatile uint16_t first_electric_current=0; //第一路电流值0000-1000（等效为0.0%-100.0%）
volatile uint16_t second_electric_current=0;//第二路电流值0000-1000（等效为0.0%-100.0%）
volatile uint16_t third_electric_current=0; //第三路电流值0000-1000（等效为0.0%-100.0%）
volatile uint16_t fourth_electric_current=0;//第四路电流值0000-1000（等效为0.0%-100.0%）
volatile uint16_t sum_electric_current=0;   //总的电流值0000-4000
volatile uint16_t set_LED_power=0;          //设定的LED功率值

volatile uint8_t drive_State_Update=pdTRUE;      //驱动状态更新

//test
uint8_t gpio_State=0;
uint32_t t=0;
/* USER CODE END Variables */
/* Definitions for depdFALSETask */
osThreadId_t depdFALSETaskHandle;
const osThreadAttr_t depdFALSETask_attributes = {
  .name = "depdFALSETask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for button */
osThreadId_t buttonHandle;
const osThreadAttr_t button_attributes = {
  .name = "button",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for driveUpdata */
osThreadId_t driveUpdataHandle;
const osThreadAttr_t driveUpdata_attributes = {
  .name = "driveUpdata",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for canRxTx */
osThreadId_t canRxTxHandle;
const osThreadAttr_t canRxTx_attributes = {
  .name = "canRxTx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
unsigned char even_parity(uint8_t *data, uint16_t start_i, uint16_t start_j);
/* USER CODE END FunctionPrototypes */

void StartDepdFALSETask(void *argument);
void buttonTask01(void *argument);
void driveUpdataTask01(void *argument);
void canRxTxTask01(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of depdFALSETask */
  depdFALSETaskHandle = osThreadNew(StartDepdFALSETask, NULL, &depdFALSETask_attributes);

  /* creation of button */
  buttonHandle = osThreadNew(buttonTask01, NULL, &button_attributes);

  /* creation of driveUpdata */
  driveUpdataHandle = osThreadNew(driveUpdataTask01, NULL, &driveUpdata_attributes);

  /* creation of canRxTx */
  canRxTxHandle = osThreadNew(canRxTxTask01, NULL, &canRxTx_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDepdFALSETask */
/**
  * @brief  Function implementing the depdFALSETask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDepdFALSETask */
void StartDepdFALSETask(void *argument)
{
  /* USER CODE BEGIN StartDepdFALSETask */
  /* Infinite loop */
  for(;;)
  {
    //SPI1_TxData();

//		LCD_Fill(0,0,LCD_W,LCD_H,BLACK);
//		LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
		
		LCD_ShowString(0,40,"brightness:",WHITE,BLACK,24,0);
		uint16_t bn=0;
		uint16_t bf=0;
		bn=(brightness+0.0001)*100; //0.0001为了进位
		bf=(uint16_t)((brightness*100+0.05-bn)*10);

		
		LCD_ShowIntNum(136,40,bn,3,WHITE,BLACK,24);
		LCD_ShowString(176,40,".",WHITE,BLACK,24,0);
		LCD_ShowIntNum(184,40,bf,1,WHITE,BLACK,24);
		LCD_ShowString(196,40,"%",WHITE,BLACK,24,0);

		LCD_ShowString(0,80,"color_Temperature:",WHITE,BLACK,24,0);
		LCD_ShowIntNum(220,80,color_Temperature,4,WHITE,BLACK,24);
		LCD_ShowString(276,80,"K",WHITE,BLACK,24,0);
		

    osDelay(1);
  }
  /* USER CODE END StartDepdFALSETask */
}

/* USER CODE BEGIN Header_buttonTask01 */
/**
* @brief Function implementing the buttonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_buttonTask01 */
void buttonTask01(void *argument)
{
  /* USER CODE BEGIN buttonTask01 */
  /* Infinite loop */	

  for(;;)
  {
    //uint8_t gpio_State=0;
    
		if(!HAL_GPIO_ReadPin(GPIOE, KEY0_Pin))//KEY0_Pin为低电平有效 增加色温
    {
      osDelay(20);//按键消抖
      if(!HAL_GPIO_ReadPin(GPIOE, KEY0_Pin))//确认KEY0_Pin被按下
      {
        if(color_Temperature<=6450)
        {
          color_Temperature =  color_Temperature + 50;//增加色温
          drive_State_Update = pdTRUE;
        }
      }
    }

    if(!HAL_GPIO_ReadPin(GPIOE, KEY2_Pin))//KEY2_Pin为低电平有效 降低色温
    {
      osDelay(20);//按键消抖
      if(!HAL_GPIO_ReadPin(GPIOE, KEY2_Pin))//确认KEY2_Pin被按下
      {
        if(color_Temperature>=2750)
        {
          color_Temperature =  color_Temperature - 50;//降低色温
          drive_State_Update = pdTRUE;
        }
      }
    }

    if(HAL_GPIO_ReadPin(GPIOA, KEY_UP_Pin))//KEY_UP_Pin为高电平有效 增加亮度
    {
      osDelay(20);//按键消抖
      if(HAL_GPIO_ReadPin(GPIOA, KEY_UP_Pin))//确认KEY_UP_Pin被按下
      {
        if(brightness<=0.999)
        {
          brightness =  brightness + 0.001;//增加亮度
          drive_State_Update = pdTRUE;
        }
      }
    }

    if(!HAL_GPIO_ReadPin(GPIOE, KEY1_Pin))//KEY1_Pin为低电平有效 降低亮度
    {
      osDelay(20);//按键消抖
      if(!HAL_GPIO_ReadPin(GPIOE, KEY1_Pin))//确认KEY1_Pin被按下
      {
        if(brightness>=0.0004)
        {
          brightness =  brightness - 0.001;//降低亮度
					if(brightness<0) brightness=0;
          drive_State_Update = pdTRUE;
        }
      }
    }
    osDelay(100);
  }
  /* USER CODE END buttonTask01 */
}

/* USER CODE BEGIN Header_driveUpdataTask01 */
/**
* @brief Function implementing the driveUpdata thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_driveUpdataTask01 */
void driveUpdataTask01(void *argument)
{
  /* USER CODE BEGIN driveUpdataTask01 */
  /* Infinite loop */
  for(;;)
  {

    //text
    // drive_Switch=0x77;
    // brightness=0.5;
    // color_Temperature=6500;
    // fan_Ratio=0x64;

    float cold_Out=0;                   //定义冷色温输出值0.0-100%
    float warm_Out=0;                   //定义暖色温输出值0.0-100%
    uint8_t even_parity_flag = 0; //偶校验标志位
    uint16_t first_electric_current=0;  //第一路电流值0000-1000（等效为0.0%-100.0%）
    uint16_t second_electric_current=0; //第二路电流值0000-1000（等效为0.0%-100.0%）
    uint16_t third_electric_current=0;  //第三路电流值0000-1000（等效为0.0%-100.0%）
    uint16_t fourth_electric_current=0; //第四路电流值0000-1000（等效为0.0%-100.0%）
    uint16_t sum_electric_current=0;    //总的电流值0000-4000
    uint16_t set_LED_power=0;           //设定的LED功率值

    //输入LED灯亮度与色温，计算CCT模式冷暖色温两路输出
    cct(brightness,color_Temperature,&cold_Out,&warm_Out);

    //将冷暖输出进行格式转化并赋四路电流数值（这里假设第一路是冷色温）
    first_electric_current=(cold_Out*1000*0.5)+0.5;//+0.5实现float类型转换为uint16_t时四舍五入
    second_electric_current=(cold_Out*1000*0.5)+0.5;
    third_electric_current=(warm_Out*1000*0.5)+0.5;
    fourth_electric_current=(warm_Out*1000*0.5)+0.5;

    //安全检查
    //对四路电流输出值进行限幅
    if(first_electric_current>1000)    {first_electric_current=1000;}
    else if(first_electric_current<0)  {first_electric_current=0;}
    if(second_electric_current>1000)   {second_electric_current=1000;}
    else if(second_electric_current<0) {second_electric_current=0;}
    if(third_electric_current>1000)    {third_electric_current=1000;}
    else if(third_electric_current<0)  {third_electric_current=0;}
    if(fourth_electric_current>1000)   {fourth_electric_current=1000;}
    else if(fourth_electric_current<0) {fourth_electric_current=0;}

    //计算主控板设定的输出功率
    sum_electric_current=first_electric_current+second_electric_current+third_electric_current+fourth_electric_current;
    //判断LED功率是否超出1200W，超出则不输出
    if(sum_electric_current>2000){first_electric_current=second_electric_current=third_electric_current=fourth_electric_current=0;}
    set_LED_power= sum_electric_current *2400/(1000*4);

    drive_TxData[0] =0xaa; //第一帧帧头0xaa
    drive_TxData[1] =(first_electric_current / 256);//第一路恒流输出高八位0x00-0x0a
    drive_TxData[2] =(first_electric_current % 256);//第一路恒流输出低八位0x00-0x63
    drive_TxData[3] =(second_electric_current / 256);//第二路恒流输出高八位0x00-0x0a
    drive_TxData[4] =(second_electric_current % 256);//第二路恒流输出低八位0x00-0x63
    drive_TxData[5] =drive_Switch;//开0x77关0x88机
    even_parity_flag=even_parity(drive_TxData, 0, 5);
    drive_TxData[6] =even_parity_flag;//第一帧前6位偶校验
    drive_TxData[7] =0xfe;//第一帧帧尾

    drive_TxData[8] =0xbb;//第二帧帧头0xbb
    drive_TxData[9] =(third_electric_current / 256);//第三路恒流输出高八位0x00-0x0a
    drive_TxData[10]=(third_electric_current % 256);//第三路恒流输出低八位0x00-0x63
    drive_TxData[11]=(fourth_electric_current / 256);//第四路恒流输出高八位0x00-0x0a
    drive_TxData[12]=(fourth_electric_current % 256);//第四路恒流输出低八位0x00-0x63
    drive_TxData[13]=fan_Ratio;                     //第五路DC0-12V输出0x00-0x64
    even_parity_flag=even_parity(drive_TxData, 8, 13);
    drive_TxData[14]=even_parity_flag;//第二帧前6位偶校验
    drive_TxData[15]=0xff;//第二路帧尾0xff
 
    osDelay(1);
  }
  /* USER CODE END driveUpdataTask01 */
}

/* USER CODE BEGIN Header_canRxTxTask01 */
/**
* @brief Function implementing the canRxTx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_canRxTxTask01 */
void canRxTxTask01(void *argument)
{
  /* USER CODE BEGIN canRxTxTask01 */
  /* Infinite loop */
  for(;;)
  {
    while (drive_State_Update==pdTRUE)
    {
      drive_State_Update=pdFALSE;
      //将drive_TxData数组通过CAN发送给下位机
      for (uint16_t i = 0; i < 2; i++)
      {
        for (uint16_t j = 0; j < 8; j++)
        {
          /* code */
          if(i==0) {CanTxData[j]=drive_TxData[j];}
          else {CanTxData[j]=drive_TxData[j+8];}
        }
        Can_senddata(CanTxData,CANTXDATALONG);
        osDelay(10);//避免帧冲突
      }
    }
    
    osDelay(1);
  }
  /* USER CODE END canRxTxTask01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/*
* 偶校验2
* uint8_t *data 输入数组 
* uint16_t start_i 开始校验数组序号（包含）
* uint16_t start_j 结束校验数组序号（包含）
*/

unsigned char even_parity(uint8_t *data, uint16_t start_i, uint16_t start_j)
{
    unsigned char parity = 0;
    unsigned char n_bits = 0;
    uint16_t data_long = 0;
    unsigned char value = 0;

    uint16_t i=start_i;
    data_long = start_j - start_i + 1;

    for (i=start_i; i < start_i+data_long; i++)
    {
      /* code */
      value = data[i];
			n_bits = sizeof(data[start_i]) * 8;
      while( n_bits >0)
      {
          parity += value & 1;
          value >>=1;
          n_bits -=1;
      }
    }
    
    /*
    * 当实际数据中“1”的个数为偶数，校验位是“1”，否则校验位是“0”
    */
    return (parity % 2) != 0;
	
}

/* USER CODE END Application */

