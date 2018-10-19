
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <param_crc.h>
//#include <tim.h>
//#include <stm32f0xx.h>
//#include <stm32f0xx_flash.h>


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#define FW_ADDRESS			0x08000000
#define FW_ADDRESS_SHIFT    0
int add_linear_address = 0;
//#define NAH_UART		
/* USER CODE END PFP */





/* USER CODE BEGIN 0 */
int delay1 = 1000;
int delay2 = 1000;
uint16_t bl_delay = BL_WAIT;
static __IO uint32_t MillisecondTick = 0;
static __IO uint32_t lastReceivedTick = 0;

extern char new_fw[2080];
extern char new_fw0[109];
extern char new_fw1[93];
extern char new_fw2[93];
char page_tmp;
uint8_t size_data, type_data, check_sum;
uint16_t address_data;
uint8_t calculation_check_sum = 0;
//volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
HAL_StatusTypeDef FLASHStatus;
uint32_t program_data;
uint32_t extented_linear_adress = 0;


typedef struct{
	uint16_t page;
	uint16_t data_lng;
	uint16_t number;
	uint16_t total_pages;
	uint16_t total_lng;
}FW_str;

enum 
{
	FW_Data,
	FW_Info
};
uint8_t is_load = FW_Data;

typedef enum
{
	WRITE_CHECK_SUM_ERROR,
	EXTENDED_LINEAR_ADDRESS_CHECK_SUM_ERROR,
	ADD_LINEAR_ADDRESS_CHECK_SUM_ERROR,
	END_CHECK_SUM_ERROR,
	CHECK_SUM_OK
}checksum_msg;

FW_str msg_param;

typedef enum 
{
	FLAG_FW_NEW,	
	FLAD_FW_UPDATE,
	FLAG_FW_OK
} flag_enum;


typedef struct
{
	uint16_t fw_ver;
	uint16_t hw_ver;
	uint16_t fw_size;
	flag_enum FLAG;
}add_info;

add_info InfoStr;

checksum_msg Get_Raw_FW(FW_str *str, uint8_t *data, uint16_t lng);
void Go_To_User_App(void);
void Read_NB(char *source, uint8_t *buf, uint32_t start_source, uint8_t count);
void Ascii_To_Hex( uint8_t* buff, uint8_t count);
void usart_int_enable( void );
bool get_param(uint8_t* data, uint16_t* page, uint16_t* lng, uint16_t* msg_ln); 
uint8_t Ascii_To_Hex_Short(uint8_t count);
bool Get_Params(FW_str *str, uint8_t *data, uint16_t lng);
bool FLASH_Erase(uint16_t lng_byte);		//max 64Kb
void Remap_Table(void);
uint8_t Pack_buff( char *buff, uint8_t cmd, uint8_t *data, uint8_t data_lng);
void Parce_Packet( uint8_t *buf, uint16_t lng );
void Parce_CMD( uint8_t *buf, uint16_t lng );
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
HAL_StatusTypeDef ans;
//bool is_msg = false;
uint16_t msg_lng = 0;
uint16_t page;
uint16_t data_lng;
uint8_t rdata[BUFER_LNG];			//mov to FW_str
uint16_t ind =0;
uint8_t buff[8];
bool ans1;
bool write = false;
uint8_t pg_num = 10;
uint16_t fw_lng = 2832;
uint8_t counter_tmp = 0;
checksum_msg andww;
uint8_t writed_line = 0;
char send_buf[30];
volatile uint8_t state_flag = 0;
uint8_t tmp11111[300];
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/
	
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* USER CODE BEGIN Init */ 

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	
	
#ifndef  NAH_UART	
	
  MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	NVIC_EnableIRQ(USART2_IRQn);
	__enable_irq();
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

	//__HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);
	//__HAL_UART_ENABLE(&huart2);
#endif		//NAH_UART
	
	

	InfoStr.FLAG = FLAG_FW_NEW;
	InfoStr.fw_ver = 0x11;
	InfoStr.hw_ver = 0x22;
	InfoStr.fw_size = 0x5800;
	HAL_Delay(100);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);	//RESGSM_HIGH
	HAL_Delay(100);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);		//PWRKEY_LOW
	HAL_Delay(1000);
		//read InfoStr
	if(InfoStr.FLAG == FLAD_FW_UPDATE)
		InfoStr.FLAG = FLAG_FW_NEW;
	if( InfoStr.FLAG == FLAG_FW_NEW )
		state_flag = WAIT_FW_CMD;

  while (1)
  { 
		if(((MillisecondTick - lastReceivedTick) > END_OF_MSG_DELAY))
		{
			if( ind > 0 )
			{
//				for(int i = 0; i < 300; i++)
//				{
//					tmp11111[i] = rdata[2047 + i];
//				}

				Parce_Packet( (uint8_t *)rdata, ind );
				ind = 0;
			//	ind = 0;
			}
		}
////////////	 if(is_load == FW_Data) 
////////////	 {
//////////////			HAL_Delay(delay1);
//////////////			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_1);
//////////////			HAL_Delay(delay2);
//////////////			HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_1);
////////////		 HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
////////////		 HAL_Delay(delay1);
////////////		 /*if(msg_lng != 0)
////////////		 {
////////////			 
////////////			 if(Get_Params(&msg_param, rdata, msg_lng))
////////////			 {
////////////				 msg_param.data_lng = 0x65;
////////////					Get_Raw_FW(&msg_param, rdata, msg_lng);
////////////					
////////////			 };		 
////////////			
////////////		 }*/
////////////		 switch(pg_num)
////////////		 {
////////////			 case 0:
////////////				 ans1 = FLASH_Erase(fw_lng);
////////////				 FLASHStatus = HAL_OK;
////////////				 break;
////////////			 case 1:
////////////				 msg_lng = 4002;
////////////				 if(Get_Params(&msg_param, (uint8_t *)new_fw, msg_lng))
////////////				 {
////////////					// msg_param.data_lng = 0x65;
////////////						andww = Get_Raw_FW(&msg_param, (uint8_t *)new_fw, msg_lng);
////////////						
////////////				 };	
////////////				 pg_num = 4;
////////////			 break;
////////////			/* case 2:
////////////				 msg_lng = 93;
////////////				 if(Get_Params(&msg_param, (uint8_t *)new_fw2, msg_lng))
////////////				 {
////////////					// msg_param.data_lng = 0x65;
////////////						Get_Raw_FW(&msg_param, (uint8_t *)new_fw2, msg_lng);
////////////						
////////////				 };	
////////////			   pg_num = 3;
////////////			 break;
////////////			 case 3:
////////////				 msg_lng = 93;
////////////			 	 if(Get_Params(&msg_param, (uint8_t *)new_fw1, msg_lng))
////////////				 {
////////////					// msg_param.data_lng = 0x65;
////////////						Get_Raw_FW(&msg_param, (uint8_t *)new_fw1, msg_lng);
////////////						
////////////				 };	
////////////				 HAL_FLASH_Lock();
////////////				 pg_num = 4;
////////////			 break;*/
////////////			 case 4:
////////////				msg_lng = 0;
////////////				
////////////			 break;
////////////			 case 5:
////////////				 if(counter_tmp > 10)
////////////				 {
////////////					Go_To_User_App();
////////////				 }
////////////				 counter_tmp++;
////////////			 break;
////////////			 case 8:
////////////				 //HAL_UART_Transmit();
////////////				HAL_UART_Transmit(&huart1, (uint8_t *)send_buf, 5, 100);
////////////				 pg_num = 10;
////////////			 break;
////////////			 default:
////////////				 break;
////////////			 
////////////		 }
////////////		 
////////////		 
////////////		 
////////////	 }
////////////	 else if( is_load == FW_Info )
////////////	 {
////////////			
////////////				if(Get_Params(&msg_param, (uint8_t *)rdata, msg_lng))
////////////				{
////////////					HAL_Delay(100);
////////////				}
////////////		 
////////////		 NVIC_SystemReset();
////////////				
////////////		 
////////////		 
////////////	 }
HAL_Delay(1000);
  }


}


//uint8_t buff[8];
uint8_t err_couter = 0;
	uint16_t prev_write_end  = 0;

uint16_t reduce_counter = 250;
void HAL_SYSTICK_Callback(void)
{
	
	MillisecondTick = HAL_GetTick();
	if( (InfoStr.FLAG == FLAG_FW_NEW)&& (state_flag == WAIT_FW_CMD))
	{
		uint8_t lng = Pack_buff((char *)send_buf, WAIT_FW_CMD, 0, 0);
		if(reduce_counter == 0)
		{
//			HAL_UART_Transmit(&huart1, (uint8_t *)send_buf, lng, 100);
			reduce_counter = 3000;
		}
		if(bl_delay > 0)
			bl_delay--;
	}
	if(reduce_counter > 0)
		reduce_counter--;
	
	
}
void Parce_Packet( uint8_t *buf, uint16_t lng )
{
	
		 uint16_t crc16 = param_crc16_init();
			for( uint16_t i  = 0; i < lng; i++ )
			{
				crc16 = param_crc16_update(crc16, buf[i]);
			}
			if( crc16 == 0 )
			{
					if( buf[0] != ADDR)
						return;
					if( (buf[1] != RESET_CMD) && (buf[1] !=DOWNLOAD_CMD) && (buf[1] !=RECV_FW_CMD) && (buf[1] !=NEW_INFO_CMD))
							return;
					
					Parce_CMD((uint8_t *)rdata, ind ); 
			}
			else
				return;
	
};

void Parce_CMD( uint8_t *buf, uint16_t lng )
{
	switch(buf[1])
	{
		case RESET_CMD:
			NVIC_SystemReset();
			while(1);
		break;
		case DOWNLOAD_CMD:
				if(state_flag != WAIT_FW_CMD)
					break;
				state_flag = DOWNLOAD_CMD;
				InfoStr.FLAG = FLAD_FW_UPDATE;
				uint8_t tmp[3];
				tmp[0] = DATA_OK;
				tmp[1] = BUFER_LNG & 0xFF;
				tmp[2] = (BUFER_LNG >> 8)&0xFF;
				uint8_t lng_tmp = Pack_buff((char *)send_buf, DOWNLOAD_CMD, tmp, sizeof(tmp));
		//		ind = 0;
				HAL_UART_Transmit(&huart1, (uint8_t *)send_buf, lng_tmp, 500);
				state_flag = DOWNLOAD_CMD;
		
			break;
		case RECV_FW_CMD:
			if(state_flag != DOWNLOAD_CMD)
				break;
		
			Get_Params(&msg_param, buf, lng);
		
			HAL_FLASH_Unlock();
			FLASH_PageErase( EXTENDED_LINEAR_ADDRESS + FW_SHIFT + msg_param.page * FLASH_PAGE_SIZE );
			while(FLASH->SR & FLASH_SR_BSY )
			{}
			HAL_FLASH_Lock();

				
			if( Get_Raw_FW(&msg_param, buf, lng) == CHECK_SUM_OK )
			{
				uint8_t lng = Pack_buff((char *)send_buf, RECV_FW_CMD, (uint8_t *)DATA_OK, 1);
				HAL_UART_Transmit(&huart1, (uint8_t *)send_buf, lng, 500);
			}
			else
			{
				uint8_t tmp[3];
				tmp[0] = DATA_NOK;
				tmp[1] = msg_param.page & 0xFF;
				tmp[2] = (msg_param.page >> 8)&0xFF;
				uint8_t lng = Pack_buff((char *)send_buf, RECV_FW_CMD, tmp, sizeof(tmp));
				HAL_UART_Transmit(&huart1, (uint8_t *)send_buf, lng, 500);									
				
			}
		break;
		case NEW_INFO_CMD:
			if(state_flag != NEW_INFO_CMD){
				break;}
			break;
		default:
			break;
	}
		
}

uint8_t Pack_buff( char *buff, uint8_t cmd, uint8_t *data, uint8_t data_lng)
{
	 uint8_t lng = 0;
	 uint16_t crc16 = param_crc16_init();
	 crc16 = param_crc16_update(crc16, ADDR);
   buff[lng++] = ADDR;
	 crc16 = param_crc16_update(crc16, cmd);
	 buff[lng++] = cmd;
	 for( int i = 0; i < data_lng; i++ )
	 {
		 buff[lng++] = data[i];
		 crc16 = param_crc16_update(crc16, data[i]);
	 }
   crc16 = param_crc16_finit(crc16);
	 buff[lng++]  = crc16 & 0xFF;
	 buff[lng] = crc16 >> 8;
	 
	 return ++lng;
		
	
};
bool FLASH_Erase(uint16_t lng_byte)
{
	uint16_t page_num = (lng_byte/FLASH_PAGE_SIZE);		// val of pages t errase
	uint32_t addr = FW_ADDRESS + FW_ADDRESS_SHIFT;			// ?FW_ADDRESS to extented_linear_adress
	HAL_FLASH_Unlock();
		do
		{
			addr += (page_num * 1024);
			
			FLASH_PageErase(addr + ( page_num * FLASH_PAGE_SIZE));
			while(FLASH->SR & FLASH_SR_BSY )
			{}
			
			if(page_num > 0)
				page_num--;
		}while( page_num > 0);
		HAL_FLASH_Lock();
};
checksum_msg Get_Raw_FW(FW_str *str, uint8_t *data, uint16_t lng)
{
	bool is_end = false;
	int si = 0;
//	uint16_t prev_lng  = 0;
	HAL_FLASH_Unlock();
	
	while(si <= str->data_lng)
	{
		if(data[si] == ':')
		{
				
				si++;
				Read_NB((char *)data, buff, si, 8); 
				si += 8;
				Ascii_To_Hex(buff,8);
				
				size_data = 2*(buff[1] + 16*buff[0]);
				address_data = buff[5] + 16*buff[4] + 256*buff[3] + 4096*buff[2];
				type_data = buff[7] + 16*buff[6];
				calculation_check_sum = size_data/2 + (uint8_t)address_data + (uint8_t)(address_data>>8) + type_data;
							
				if(type_data == 0x00)
				{
					//int si_tmp = si;
					//uint8_t size_data_tmp = size_data;
//					uint16_t address_data_tmp = address_data;
					while(size_data>0 && (FLASHStatus == HAL_OK))					
					{
						Read_NB((char *)data, buff, si, 8);
						si += 8;
						Ascii_To_Hex(buff, 8);
									
						for(int i=0; i<8;i=i+2)
						{
							buff[i] <<= 4;
							buff[i] = buff[i] | buff[i+1];
							program_data |= buff[i] <<(i*4);
						}	
						
						__disable_irq();

								FLASH->CR &= ~(FLASH_CR_PER);
								FLASH->CR |= FLASH_CR_PG;
								FLASHStatus = HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, ((EXTENDED_LINEAR_ADDRESS + address_data)), program_data);
								
							//	prev_lng += str->data_lng;
								while(FLASH->SR & FLASH_SR_BSY )
								{}
								
						//FLASHStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, extented_linear_adress + address_data, program_data );

						__enable_irq();
									
						calculation_check_sum +=  (uint8_t)program_data + (uint8_t)(program_data>>8) + (uint8_t)(program_data>>16) + (uint8_t)(program_data>>24);	
						size_data -=8;
						address_data +=4;
						program_data = 0;	
					}



					
							
					
					calculation_check_sum =  ~(calculation_check_sum) + 1;			// check after flash programming???
								
					Read_NB((char *)data, buff, si, 2);
					si += 2;
					Ascii_To_Hex(buff,2);
					check_sum = buff[1] + 16*buff[0];		
					//si = si_tmp;
				//	size_data = size_data_tmp;
					if(calculation_check_sum != check_sum )
					{
						return WRITE_CHECK_SUM_ERROR;
					}
				/*	else
					{
						while(size_data>0 && (FLASHStatus == HAL_OK))
						{
								Read_NB((char *)data, buff, si, 8);
								si += 8;
								Ascii_To_Hex(buff, 8);
											
								for(int i=0; i<8;i=i+2)
										{
									buff[i] <<= 4;
									buff[i] = buff[i] | buff[i+1];
									program_data |= buff[i] <<(i*4);
								}	
								
								__disable_irq();

										FLASH->CR &= ~(FLASH_CR_PER);
										FLASH->CR |= FLASH_CR_PG;
										FLASHStatus = HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, (FW_ADDRESS_SHIFT+(extented_linear_adress + address_data)), program_data);
										while(FLASH->SR & FLASH_SR_BSY )
										{}
										if(FLASHStatus != HAL_OK )
										{
											return FLASHStatus;
										}
										
								//FLASHStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, extented_linear_adress + address_data, program_data );

								__enable_irq();
											
	//							calculation_check_sum +=  (uint8_t)program_data + (uint8_t)(program_data>>8) + (uint8_t)(program_data>>16) + (uint8_t)(program_data>>24);	
								size_data -=8;
								address_data +=4;
								program_data = 0;		
						}
					}*/
					calculation_check_sum = 0;
					prev_write_end += str->data_lng;	
					writed_line++;
				}
				else if(type_data == 0x04)
				{
					Read_NB((char *)data, buff, si, 4);
					si +=4;
					Ascii_To_Hex(buff,4);
					extented_linear_adress = (uint32_t)(buff[0]<<28 | buff[1]<<24 | buff[2]<<20 | buff[3]<<16 );//??????? ?????
											
					calculation_check_sum +=  buff[0] + buff[1]+ buff[3] + buff[3];	
					calculation_check_sum =  ~(calculation_check_sum) + 1;
								
					Read_NB((char *)data, buff, si, 2);
					si += 2;
					Ascii_To_Hex(buff,2);
					check_sum = buff[1] + 16*buff[0];		
						if(calculation_check_sum != check_sum )
						{
							return EXTENDED_LINEAR_ADDRESS_CHECK_SUM_ERROR;
						}	
					calculation_check_sum = 0;
				}
				else if(type_data == 0x05)			/////check for MDK_ARM					get linerar address from hex && try to full fw
				{
					Read_NB((char *)data, buff, si, size_data);
					si+=size_data;
					Ascii_To_Hex(buff, size_data);
					
					for(int j = 0; j < size_data; j++)
					{
						add_linear_address += buff[j]<<(4*(size_data - j));
					}
					
					
					for(int i = 0; i < size_data; i++)
					{
						calculation_check_sum +=  buff[i];
					}
					
					
					calculation_check_sum =  ~(calculation_check_sum) + 1;
					
					
					
					Read_NB((char *)data, buff, si, 2);
					si += 2;
					Ascii_To_Hex(buff,2);
					check_sum = buff[1] + 16*buff[0];		
						if(calculation_check_sum != check_sum )
						{
							return ADD_LINEAR_ADDRESS_CHECK_SUM_ERROR;
						}	
					
				}
				else if(type_data == 0x01)		//EOF
				{
						Read_NB((char *)data, buff, si, 2);
						si += 2;
						Ascii_To_Hex(buff,2);
						if( buff[0] != 0xF || buff[1] != 0xF )
						{
								return END_CHECK_SUM_ERROR;
						}
						HAL_FLASH_Lock();
						state_flag = NEW_INFO_CMD;
						is_end = true;
						break;
				}		
					
			
		}
		else
		{
			si++; 
		}
	}
	HAL_FLASH_Lock();
	return CHECK_SUM_OK;


}

void Read_NB(char *source, uint8_t *buf, uint32_t start_source, uint8_t count)
{
	for(int i = 0; i < count; i++)
	{
		buff[i] = (uint8_t)source[start_source + i];
	}
}



void Ascii_To_Hex( uint8_t* buff, uint8_t count)
{
	uint8_t i;
	
	for(i=0; i<count;i++)
	{
		if(buff[i] <= '9' && buff[i] >= '0' )
		{
			buff[i] -= 0x30;
		}
		else
		{
			buff[i] = buff[i] - 0x41 + 10;
		}	
	}	
}

uint8_t Ascii_To_Hex_Short(uint8_t count)
{
//	uint8_t i;
	

		if(count<= '9' && count>= '0' )
		{
			return (count-0x30);
		}
		else if((count <= 'F' && count >='A') || (count <= 'f' && count >='a'))
		{
			return (count - 0x41 + 10);
		}	
	
}
bool Get_Params(FW_str *str, uint8_t *data, uint16_t lng)
{
			uint16_t tmp;
//			uint8_t tmp2;
			bool ans = false;
			for(int i = 0; i <= lng; i++)
			 {
				 

					if(data[i] == '!')
					{
							tmp = i;
							while( data[i] != '@' ){
								i++;}
							i--;
							for( int j = (i - tmp); j > 0; j--)
							{
									str->page += (Ascii_To_Hex_Short(data[i-j+1]) *10^(j-1));
									ans = false;
							}

					}
					else if(data[i] == '@' && !ans)
					{
							tmp = i;
							while( data[i] != '#' ){
								i++;}
							i--;
							for( int j = (i - tmp); j >= 0; j--)
							{
										str->data_lng += (Ascii_To_Hex_Short(data[i-j+1]))<<(4*(j-1));
										ans = true;
							}
					}
					
					
					if(data[i] == '&' )
					{
							tmp = i;
							while( data[i] != '@' ){
								i++;}
							i--;
							for( int j = (i - tmp); j > 0; j--)
							{
									str->total_pages += (Ascii_To_Hex_Short(data[i-j+1]))<<(4*(j-1));
									ans = true;
							}

					}
					else if(data[i] == '@' && ans)
					{
							tmp = i;
							while( data[i] != '*' ){
								i++;}
							i--;
							for( int j = (i - tmp); j >= 0; j--)
							{
										str->total_lng += (Ascii_To_Hex_Short(data[i-j+1]))<<(4*(j-1));
										ans = true;
							}
					}
					
					
					
					
					
					
					
					
					

			 }
			 return ans;
};

typedef void (*pfunc_jump_to_appl)(void);
void Go_To_User_App(void)
{
	
	volatile uint32_t *RAMVectorTable = (volatile uint32_t *)0x20000000;
  for(uint32_t iVector = 0; iVector < 48; iVector++) {
      RAMVectorTable[iVector] = *(__IO uint32_t *)((uint32_t)0x08002800 + (iVector << 2));
  }
  
	
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  //SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_SRAM);
  
	RCC_APB2ENR_SYSCFGEN;
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_MEM_MODE;		
	
  uint32_t ApplJumpAddr = *(__IO uint32_t*)(0x08002800 + 4);
  pfunc_jump_to_appl FuncJumpToAppl = (pfunc_jump_to_appl)ApplJumpAddr;
  __set_MSP(*(__IO uint32_t *)0x08002800);
  FuncJumpToAppl();
	
	/*
    uint32_t app_jump_address;
		Remap_Table();
    typedef void(*pFunction)(void);
    pFunction Jump_To_Application;
     __disable_irq();
    app_jump_address = *( uint32_t*) (0x08002800 + 4);    
    Jump_To_Application = (pFunction)app_jump_address;            
     __set_MSP(*(__IO uint32_t*) 0x08002800);                                                  
    Jump_To_Application();		*/                        
}

void usart_int_enable( void )
{

				
						// ???????????????????????????????????????????????
			/* Enable the UART Parity Error Interrupt */
			__HAL_UART_ENABLE_IT(&huart2, UART_IT_PE);

			/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	//		__HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);

			/* Enable the UART Data Register not empty Interrupt */
			__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	
	

		//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
			
			

};


void USART1_IRQHandler(void)
{
//	  HAL_UART_IRQHandler(&huart1);

	if( huart1.Instance->ISR & USART_ISR_RXNE)
	{
			HAL_UART_RxCpltCallback(&huart1);
	}
	huart1.Instance->ICR |= 0x0F;	//reset error

}
bool get_param(uint8_t* data, uint16_t* page, uint16_t* lng,uint16_t* msg_ln)
{
		uint16_t p = *page;
		uint16_t l = *lng;
		uint16_t m = *msg_ln;
	
		if(p == 0 || l == 0 )
			return false;
		
		
	
}; 

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		if( huart->Instance == USART1)
		{
			rdata[ind++] = (uint8_t)huart->Instance->RDR;
			lastReceivedTick = HAL_GetTick();
			//ind >= BUFER_LNG ? ind = 0: ind++;		//error buffer overflow
		};

}
uint8_t sdata;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if( huart->Instance == USART2)
	{
	//	USART2->ICR = 0x0F;	//clearerr int flags
		sdata = (uint8_t)huart->Instance->TDR;
	} 
};

void Remap_Table(void)
{
    // Copy interrupt vector table to the RAM.
    volatile uint32_t *VectorTable = (volatile uint32_t *)0x20000000;
    uint32_t ui32_VectorIndex = 0;

    for(ui32_VectorIndex = 0; ui32_VectorIndex < 48; ui32_VectorIndex++)
    {
        VectorTable[ui32_VectorIndex] = *(__IO uint32_t*)((uint32_t)0x08002800 + (ui32_VectorIndex << 2));
    }

    __HAL_RCC_AHB_FORCE_RESET();

    //  Enable SYSCFG peripheral clock
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    __HAL_RCC_AHB_RELEASE_RESET();

    // Remap RAM into 0x0000 0000
    __HAL_SYSCFG_REMAPMEMORY_SRAM();
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */

void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
