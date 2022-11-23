/******************************************************************************************
  Filename    : Clock.c
  
  Core        : ARM Cortex®-M7
  
  MCU         : STM32F746ZGT6
    
  Author      : Chalandi Amine
 
  Owner       : Chalandi Amine
  
  Date        : 22.11.2022
  
  Description : Clock driver implementation
  
******************************************************************************************/

//=========================================================================================
// Includes
//=========================================================================================
#include "STM32F7x6.h"

//-----------------------------------------------------------------------------
/// \brief
///
/// \descr
///
/// \param
///
/// \return
//-----------------------------------------------------------------------------
void STM32F746ZGT6_InitClock(void)
{
  /* Configure the flash wait state (HCLK 216 MHz => 7WS) */
  FLASH->ACR.bit.LATENCY = 7u;
  while(FLASH->ACR.bit.LATENCY != 7u);

  /* configure buses clocks */
  RCC->CFGR.bit.HPRE  = 0;      /* AHB  (>= 25 MHz)  */
  RCC->CFGR.bit.PPRE1 = 5u;     /* APB1 (<= 54 MHz)  */
  RCC->CFGR.bit.PPRE2 = 4u;     /* APB2 (<= 108 MHz) */

  /* Configure the PLL for HCLK 216 MHz */
  RCC->PLLCFGR.bit.PLLSRC = 0u;   /* HSI source (16MHz)  */
  RCC->PLLCFGR.bit.PLLM   = 8u;   /* VCO input  (2MHz)   */
  RCC->PLLCFGR.bit.PLLN   = 216u; /* VCO output (432MHz) */
  RCC->PLLCFGR.bit.PLLP   = 0u;   /* PLL output (216MHz) */
  RCC->PLLCFGR.bit.PLLQ   = 9u;   /* PLL USB    (45MHz)  */

  /* Enable PLL */
  RCC->CR.bit.PLLON = 1u;
  while(!RCC->CR.bit.PLLRDY);

  /* Set PLL as system clock */
  RCC->CFGR.bit.SW = 2u;
  while(RCC->CFGR.bit.SWS != 2u);

  /* Enable the clock for gpio ports */
  RCC->AHB1ENR.reg  = 0x7FFul;
}

//-----------------------------------------------------------------------------
/// \brief
///
/// \descr
///
/// \param
///
/// \return
//-----------------------------------------------------------------------------
void HwInitialization(void)
{
  extern unsigned long __INTVECT_BASE_ADDRESS;
  
  /* Setup the VTOR */
  SCB->VTOR.reg = (unsigned long)&__INTVECT_BASE_ADDRESS;

  /* Init the system clock */
  STM32F746ZGT6_InitClock();
}

