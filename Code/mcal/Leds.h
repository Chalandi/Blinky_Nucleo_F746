/******************************************************************************************
  Filename    : Leds.h
  
  Board       : Nucleo-144

  Author      : Chalandi Amine
 
  Owner       : Chalandi Amine
  
  Date        : 22.11.2022
  
  Description : Nucleo-144 Led macros
  
******************************************************************************************/

#ifndef __LEDS_H__
#define __LEDS_H__

//=========================================================================================
// Includes
//=========================================================================================
#include "STM32F7x6.h"

//=========================================================================================
// Macros
//=========================================================================================
#define LED_G_CONFIGURE()   /* PB0  */ GPIOB->OSPEEDR.bit.OSPEEDR0  = 0u; GPIOB->MODER.bit.MODER0  = 1u
#define LED_B_CONFIGURE()   /* PB7  */ GPIOB->OSPEEDR.bit.OSPEEDR7  = 0u; GPIOB->MODER.bit.MODER7  = 1u
#define LED_R_CONFIGURE()   /* PB14 */ GPIOB->OSPEEDR.bit.OSPEEDR14 = 0u; GPIOB->MODER.bit.MODER14 = 1u

#define LED_G_TOGGLE()   GPIOB->ODR.bit.ODR0  ^= 1
#define LED_B_TOGGLE()   GPIOB->ODR.bit.ODR7  ^= 1
#define LED_R_TOGGLE()   GPIOB->ODR.bit.ODR14 ^= 1

#define LED_G_SET(level)   GPIOB->ODR.bit.ODR0  = level
#define LED_B_SET(level)   GPIOB->ODR.bit.ODR7  = level
#define LED_R_SET(level)   GPIOB->ODR.bit.ODR14 = level

//=========================================================================================
// Types definitions
//=========================================================================================
typedef union
{
  struct{
  unsigned long bit0:1;
  unsigned long bit1:1;
  unsigned long bit2:1;
  unsigned long :29;
  }bits;

  unsigned long reg;

}LedTreeBitCounter;



#endif /* __LEDS_H__ */