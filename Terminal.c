/****************************************Copyright (c)****************************************************
**                                        
**                                 电子科技大学 科研楼B240
**
**--------------File Info---------------------------------------------------------------------------------
** File name:						main.c
** Last modified Date: 	2018-9-19         
** Last Version:		   	1.0
** Descriptions:		
**						
**--------------------------------------------------------------------------------------------------------
** Created by:					JunweiYou
** Created date:				2018-9-19
** Version:			    		1.0
** Descriptions:				携能通信终端
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
#include <string.h>
#include "hal_nrf.h"
#include "hal_nrf_hw.h"
#include "hal_clk.h"
#include "hal_delay.h"	

/*******************************************************************************************************

                                              管脚定义

 *******************************************************************************************************/
#define  D1    P00  																								//通信指示灯

#define  TX_PAYLOAD_LEN    2     																		//发送数据长度
#define  RX_PAYLOAD_LEN    2     																		//接收数据长度
#define  RF_CHANNEL        55    																		//数据传输信道

xdata bool  radio_busy;
uint8_t RF_Recv_Flag;
xdata uint8_t  tx_payload[32];
xdata uint8_t  rx_payload[32];
uint8_t RF_Recv_Flag;
uint16_t PipeAndLen;

/*******************************************************************************************************
                  
									                          配置 IO 状态
									
 *******************************************************************************************************/
void IO_Init(void)
{	
	P0DIR &= ~0x01;	   																								//配置P0.0为输出
	D1 = 1;            																								//设置D1初始状态为熄灭
}

/*******************************************************************************************************

                                            配置无线参数

 *******************************************************************************************************/
void RfCofig(void)
{
  RFCKEN = 1;	     //使能RF时钟
	
  hal_nrf_close_pipe(HAL_NRF_ALL);																	//先关闭所有的通道.
	hal_nrf_open_pipe(HAL_NRF_PIPE0,false);														//再打开通道0.
	
	hal_nrf_set_operation_mode(HAL_NRF_PRX);													//模式：接收机
  hal_nrf_set_rf_channel(RF_CHANNEL);		    												//RF信道：50。接收和发送必须处于同一信道
  hal_nrf_set_datarate(HAL_NRF_250KBPS);	  												//RF速率：250KBPS
  hal_nrf_set_output_power(HAL_NRF_0DBM);	  												//功率：0DBM
  hal_nrf_set_crc_mode(HAL_NRF_CRC_16BIT);													//设置CRC校验：16位CRC。必须和发送设备一致。
  hal_nrf_set_rx_payload_width(HAL_NRF_PIPE0, RX_PAYLOAD_LEN); 			//接收模式下需要设置数据长度
  hal_nrf_set_power_mode(HAL_NRF_PWR_UP);	    											//接收机上电 
  
	RF = 1;      																											//使能无线中断
  EA = 1;	     																											//使能全局中断
	
	CE_HIGH(); 																												//使能接收
}

/*******************************************************************************************************

                                              发送数据

 *******************************************************************************************************/
void RF_SendDat(void)
{
	CE_LOW();
	hal_nrf_set_operation_mode(HAL_NRF_PTX);   												//模式：发送
	D1 = 1;
	hal_nrf_write_tx_payload(tx_payload,TX_PAYLOAD_LEN); 
	CE_PULSE();	            																					//无线发射数据  
  radio_busy = true;    
  while(radio_busy)		    																					//等待操作完成
    ;
	D1 = 0;
	hal_nrf_set_operation_mode(HAL_NRF_PRX);   												//模式：接收
	CE_HIGH();
}

 

/*******************************************************************************************************

                                               主函数

 *******************************************************************************************************/
void main()
{		
  IO_Init();    																										//IO初始化
  hal_clk_set_16m_source(HAL_CLK_XOSC16M);   												//使用外部16MHz晶振

  RfCofig();   																											//无线参数配置


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
						tx_payload[0] = 'D';
						tx_payload[1] = '1';
						RF_SendDat(); 
						break;
							
					case '2':
						tx_payload[0] = 'D';
						tx_payload[1] = '2';
						RF_SendDat(); 
						break;
							
					default:
						break;
				}
			}
    }	
  }                                           
} 
/*******************************************************************************************************

                                         接收数据（中断函数）

 *******************************************************************************************************/
void rf_irq() interrupt INTERRUPT_RFIRQ
{
  uint8_t irq_flags;
   
  irq_flags = hal_nrf_get_clear_irq_flags();												//读取并清除无线中断标志 
  
  if(irq_flags & ((1<<HAL_NRF_TX_DS)))				 											//发送结束 
  {
    radio_busy = false;			
  }

  if(irq_flags & ((1<<HAL_NRF_MAX_RT)))				 											//重新发送
  {
    radio_busy = false;
    hal_nrf_flush_tx();
  }
	
	if(irq_flags & (1<<HAL_NRF_RX_DR)) 																//接收中断
   {
      while(!hal_nrf_rx_fifo_empty()) 															//读取数据
      {
         PipeAndLen = hal_nrf_read_rx_payload(rx_payload);
      }
			hal_nrf_flush_rx();
   	  RF_Recv_Flag = 1;  																						//接收有效标志置位
   }
}
/********************************************END FILE*****************************************************/																
