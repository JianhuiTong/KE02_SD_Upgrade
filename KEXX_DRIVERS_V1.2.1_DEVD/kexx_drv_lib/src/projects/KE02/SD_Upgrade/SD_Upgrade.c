
/******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2013 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
***************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
***************************************************************************//*!
*
* @file SD_Upgrade.c
*
* @author Freescale
*
* @version 0.0.1
*
* @date Jun. 25, 2013
*
* @brief providing framework of demo cases for MCU. 
*
*******************************************************************************/

#include "common.h"
#include "ics.h"
#include "rtc.h"
#include "uart.h"
#include "sysinit.h"
#include "gpio.h"
#include "flash.h"
#include "ff.h"

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Constants and macros
******************************************************************************/
#define RELOCATION_VECTOR_ADDR          0x4000
#define USER_FLASH_START_SECTOR         RELOCATION_VECTOR_ADDR/512
#define PROGRAM_BYTES 16
/******************************************************************************
* Local types
******************************************************************************/

/******************************************************************************
* Local function prototypes
******************************************************************************/

/******************************************************************************
* Local variables
******************************************************************************/

/******************************************************************************
* Local functions
******************************************************************************/
extern void delay_ms(uint32_t cnt);
int main (void);
int8_t is_application_ready_for_executing(uint32_t applicationAddress);
void JumpToUserApplication(uint32_t userStartup);
int8_t Update_User_Application(void);
/******************************************************************************
* Global functions
******************************************************************************/


/********************************************************************/
int main (void)
{
    ICS_ConfigType  sICSConfig;

    /* Perform processor initialization */
    sysinit();
    /* switch clock mode from FEE to FEI */ 
    sICSConfig.u32ClkFreq = 32;     /* NOTE: use value 32 for 31.25KHz to 39.0625KHz of internal IRC */
    ICS_SwitchMode(FEE,FEI, &sICSConfig);

    delay_ms(1000);
    GPIO_Init(GPIOA, GPIO_PTD2_MASK, GPIO_PinInput);//PTD2 nCD Card Detection
    
    //Detect no SD card
    if(GPIO_Read(GPIOA)&GPIO_PTD2_MASK)
    {
      //User Application is Valid
      if(is_application_ready_for_executing(RELOCATION_VECTOR_ADDR))
      {
        SCB->VTOR = RELOCATION_VECTOR_ADDR;
        JumpToUserApplication(RELOCATION_VECTOR_ADDR);
      }
      else
      {
        while(1)
        {}
      }
    }  
    
    if(Update_User_Application() !=0 )
    {
      //printf("Update Failed!\n");
    }
    
    while(1)
    {
      
    }
}

int8_t is_application_ready_for_executing(uint32_t applicationAddress)
{
    if((*(uint32_t*)applicationAddress) != 0xFFFFFFFF)
    { 
      return 1;
    }
    
    return 0;
}

void JumpToUserApplication(uint32_t userStartup)
{
  /* set up stack pointer */  
  asm("LDR     r1, [r0]");
  asm("mov     r13, r1");
  /* jump to application reset vector */
  asm("ADDS      r0,r0,#0x04 ");
  asm("LDR      r0, [r0]");
  asm("BX       r0");
}

int8_t Update_File_Process(FRESULT fr, FIL fil)
{
  uint8_t Erase_Sector_Cnt;
  uint32_t file_size;
  uint32_t i;
  uint8_t read_buf[PROGRAM_BYTES];
  UINT bw;
  
  FLASH_Init(BUS_CLK_HZ);
  file_size = f_size(&fil);
  Erase_Sector_Cnt=file_size/512+1;
  for(i=0;i<Erase_Sector_Cnt;i++)
  {
   FLASH_EraseSector((USER_FLASH_START_SECTOR+i)*FLASH_SECTOR_SIZE); 
  }
  
  for(i=0;;)
  {
      fr = f_read(&fil, read_buf, PROGRAM_BYTES, &bw);
      
      if(fr)
      {
       //printf("\nError reading file\r\n");
        return -1;
      }
      else
      {
        if(bw == PROGRAM_BYTES)
        {
          FLASH_Program( USER_FLASH_START_SECTOR*FLASH_SECTOR_SIZE+i,read_buf,PROGRAM_BYTES );
          i+=PROGRAM_BYTES;
        }
        else
        {
          FLASH_Program( USER_FLASH_START_SECTOR*FLASH_SECTOR_SIZE+i,read_buf,bw );
          return 0;
        }
      }
  }
}

int8_t Update_User_Application(void)
{

  FATFS fs;               
  FRESULT fr;
  FIL			fil;			

  
  fr= f_mount(&fs,"",0); 
  if(fr)
  {
    //printf("\nError mounting file system\r\n");
    return -1;
  }
  
  fr = f_open(&fil, "update.bin", FA_OPEN_EXISTING|FA_READ);
  if(fr)
  {
    //printf("\nError opening text file\r\n");
  }
  else
  {
    if(Update_File_Process(fr,fil)!= 0)
    {
      //printf("Update File Process failed\n");
      return -1;
    }
  }
  
  fr = f_close(&fil);
  
  //printf("Jump to User App!\n");
  SCB->VTOR = RELOCATION_VECTOR_ADDR;
  JumpToUserApplication(RELOCATION_VECTOR_ADDR);
  
  return 0;
}

/********************************************************************/
