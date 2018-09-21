/****************************************Copyright (c)****************************************************
**                                        
**                                 合肥艾克姆电子科技有限公司
**                                  论坛：http://930ebbs.com
**--------------File Info---------------------------------------------------------------------------------
** File name:			main.c
** Last modified Date: 2017-3-1         
** Last Version:		   1.3
** Descriptions:		
**						
**--------------------------------------------------------------------------------------------------------
** Created by:			强光手电
** Created date:		2015-12-17
** Version:			    1.1
** Descriptions:		双向通讯无线点灯实验
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:				
** Descriptions:		
**
** Rechecked by:			
**********************************************************************************************************/
#include <reg24le1.h>
#include <stdint.h>
#include <stdbool.h>
#include "hal_nrf.h"
#include "hal_nrf_hw.h"
#include "hal_clk.h"
#include "hal_delay.h"	

/* 本试验用到的nRF24LE1的管脚
P00：输出，驱动指示灯D1	  (需要短接跳线)
P01：输出，驱动指示灯D2	  (需要短接跳线)

P12：输入，按键检测S1	  (需要短接跳线)
P13：输入，按键检测S2	  (需要短接跳线)
*/
			
/*-------------------管脚定义--------------------------------------------------*/
#define  D1    P00  // 开发板上的指示灯D1
#define  D2    P01  // 开发板上的指示灯D2
#define  S1    P12  // 开发板上的按键S1
#define  S2    P13  // 开发板上的按键S2


#define  TX_PAYLOAD_LEN    2     // 无线发送数据长度
#define  RF_CHANNEL        55    // 信道
#define  RX_PAYLOAD_LEN    2     // 接收数据长度

xdata bool  radio_busy;
uint8_t RF_Recv_Flag;
xdata uint8_t  tx_payload[32];
xdata uint8_t  rx_payload[32];
uint8_t RF_Recv_Flag;
uint16_t PipeAndLen;

/*******************************************************************************************************
 * 描  述 : 配置 IO P0.0和P0.1为输出驱动LED，P12,P13输入，按键检测
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/
void IO_Init(void)
{	
	P0DIR &= ~0x03;	   //配置P0.0和P0.1为输出
	P1DIR |= 0x0C;     //P12,P13：输入,按键
	D1 = 1;            //设置D1初始状态为熄灭
	D2 = 1;	           //设置D2初始状态为熄灭
}

/*******************************************************************************************************
 * 描  述 : 配置无线参数
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/
void RfCofig(void)
{
  RFCKEN = 1;	     //使能RF时钟
	
  hal_nrf_close_pipe(HAL_NRF_ALL);           //先关闭所有的通道.
	hal_nrf_open_pipe(HAL_NRF_PIPE0,false);	   //再打开通道0.
	
	hal_nrf_set_operation_mode(HAL_NRF_PRX);	// 模式：接收机
  hal_nrf_set_rf_channel(RF_CHANNEL);		    // RF信道：50。接收和发送必须处于同一信道
  hal_nrf_set_datarate(HAL_NRF_250KBPS);	  // RF速率：250KBPS
  hal_nrf_set_output_power(HAL_NRF_0DBM);	  // 功率：0DBM
  hal_nrf_set_crc_mode(HAL_NRF_CRC_16BIT);       //设置CRC校验：16位CRC。必须和发送设备一致。
  hal_nrf_set_rx_payload_width(HAL_NRF_PIPE0, RX_PAYLOAD_LEN); // 接收模式下需要设置数据长度
  hal_nrf_set_power_mode(HAL_NRF_PWR_UP);	    //接收机上电 
  
	RF = 1;      //使能无线中断
  EA = 1;	     // 使能全局中断
	
	CE_HIGH(); // 使能接收
}

void RF_SendDat(void)
{
	CE_LOW();
	hal_nrf_set_operation_mode(HAL_NRF_PTX);   //模式：发送	
	hal_nrf_write_tx_payload(tx_payload,TX_PAYLOAD_LEN); 
	CE_PULSE();	            //无线发射数据  
  radio_busy = true;    
  while(radio_busy)		    //等待操作完成
    ;
	hal_nrf_set_operation_mode(HAL_NRF_PRX);   //模式：接收
	CE_HIGH();
}
/*******************************************************************************************************
 * 描  述 : 主函数
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/ 
void main()
{		
  IO_Init();    //IO初始化
  hal_clk_set_16m_source(HAL_CLK_XOSC16M);   //使用外部16MHz晶振

  RfCofig();   //无线参数配置


  while(1)
  {
	  if(RF_Recv_Flag == 1)
	  {
	    RF_Recv_Flag = 0;

	    if(rx_payload[0] == 'D')
			{
				switch(rx_payload[1])
				{
					case '1':
						D1 = ~D1; 
						break;
							
					case '2':
						D2 = ~D2;
						break;
							
					default:
						break;
				}
			}
    }
		if(S1 == 0)
	  {
	    delay_ms(10);
	    if(S1 == 0)
	    {
	      D1 = 0;
	      tx_payload[0] = 'D';
				tx_payload[1] = '1';
				RF_SendDat();
		    while(S1 == 0);			    // 等待按键释放
		    D1 = 1;
	    }
	  }
    if(S2 == 0)
	  {
	    delay_ms(10);
	    if(S2 == 0)
	    {
	      D2 = 0;
	      tx_payload[0] = 'D';
				tx_payload[1] = '2';
				RF_SendDat();
		    while(S2 == 0);			    // 等待按键释放
		    D2 = 1;
	    }
	  }			
  }                                           
} 
/*******************************************************************************************************
 * 描  述 : 无线中断服务函数
 * 入  参 : 无
 * 返回值 : 无
 *******************************************************************************************************/
void rf_irq() interrupt INTERRUPT_RFIRQ
{
  uint8_t irq_flags;
   
  irq_flags = hal_nrf_get_clear_irq_flags();//读取并清除无线中断标志 
  
  if(irq_flags & ((1<<HAL_NRF_TX_DS)))				 //transimmter finish 
  {
    radio_busy = false;			
  }

  if(irq_flags & ((1<<HAL_NRF_MAX_RT)))				 //re-transimmter
  {
    radio_busy = false;
    hal_nrf_flush_tx();
  }
	
	if(irq_flags & (1<<HAL_NRF_RX_DR)) //接收中断
   {
      while(!hal_nrf_rx_fifo_empty()) //读取数据
      {
         PipeAndLen = hal_nrf_read_rx_payload(rx_payload);
      }
			hal_nrf_flush_rx();
   	  RF_Recv_Flag = 1;  //接收有效标志置位
   }
}
/********************************************END FILE*****************************************************/																
