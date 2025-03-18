/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An, Niraj Menon
  * @date    Jan 15, 2024
  * @brief   ECE 362 Lab 2 Student template
  ******************************************************************************
*/


/**
******************************************************************************/

// Fill out your username, otherwise your completion code will have the 
// wrong username!
const char* username = "ngollins";

/******************************************************************************
*/ 

#include "stm32f0xx.h"
#include <stdint.h>

void initc();
void initb();
void togglexn(GPIO_TypeDef *port, int n);
void init_exti();
void set_col(int col);
void SysTick_Handler();
void init_systick();
void adjust_priorities();

extern void autotest();
extern void internal_clock();
extern void nano_wait(int);

int main(void) {
    internal_clock();
    // Uncomment when most things are working
     autotest();
    
    initb();
    initc();
    init_exti();
    init_systick();
    adjust_priorities();

    // Slowly blinking
    for(;;) {
        togglexn(GPIOC, 9);
        nano_wait(500000000);
    }
}

/**
 * @brief Init GPIO port C
 *        PC0-PC3 as input pins with the pull down resistor enabled
 *        PC4-PC9 as output pins
 * 
 */
void initc() {
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  GPIOC -> MODER &= 0xffff0000; 
  GPIOC -> MODER |= 0x00055500;
  GPIOC -> PUPDR |= 0x000000aa; 

}

/**
 * @brief Init GPIO port B
 *        PB0, PB2, PB3, PB4 as input pins
 *          enable pull down resistor on PB2 and PB3
 *        PB8-PB11 as output pins
 * 
 */
void initb() {
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  GPIOB -> MODER &= 0xfc000000; 
  GPIOB -> MODER |= 0x00550000;
  GPIOB -> PUPDR |= 0x000000a0; 
}

/**
 * @brief Change the ODR value from 0 to 1 or 1 to 0 for a specified 
 *        pin of a port.
 * 
 * @param port : The passed in GPIO Port
 * @param n    : The pin number
 */
void togglexn(GPIO_TypeDef *port, int n) {
  if(((1 << n) & port -> IDR) == 0)
  {
    //Swap to 1
    port -> BSRR = 1 << n;
  }
  else
  {
    //Swap to 0
    port -> BRR = 1 << n;
  }
}

/**
 * @brief Follow the lab manual to initialize EXTI.  In a gist:
 *        (1-2) Enable the SYSCFG subsystem, and select Port B for
 *            pins 0, 2, 3, and 4.
 *        (3) Configure the EXTI_RTSR register so that an EXTI
 *            interrupt is generated on the rising edge of 
 *            each of the pins.
 *        (4) Configure the EXTI_IMR register so that the EXTI
 *            interrupts are unmasked for each of the pins.
 *        (5) Enable the three interupts for EXTI pins 0-1, 2-3 and
 *            4-15. Don't enable any other interrupts.
 */
void init_exti() {
  //Step 1
  RCC -> APB2ENR |= 0x00000001;
  //Step 2
  SYSCFG -> EXTICR[0] |= 0x00001101;
  SYSCFG -> EXTICR[1] |= 0x00000001;
  //Step 3
  EXTI->RTSR |= (EXTI_RTSR_TR0 | EXTI_RTSR_TR2 | EXTI_RTSR_TR3 | EXTI_RTSR_TR4);
  //Step 4
  EXTI->IMR |= (EXTI_IMR_IM0  | EXTI_IMR_IM2  | EXTI_IMR_IM3  | EXTI_IMR_IM4);
  //Step 5
  NVIC -> ISER[0] |= (1 << EXTI0_1_IRQn) | (1 << EXTI2_3_IRQn) | (1 << EXTI4_15_IRQn);
}

//==========================================================
// Write the EXTI interrupt handler for pins 0 and 1 below.
// Copy the name from the startup file as explained in the 
// lab manual, create a label of that name below, and declare 
// it to be a function.
// It should acknowledge the pending bit for pin 0, and 
// it should call togglexn(GPIOB, 8).

void EXTI0_1_IRQHandler(){
  EXTI -> PR = EXTI_PR_PR0;
  togglexn(GPIOB, 8);
}
//==========================================================
// Write the EXTI interrupt handler for pins 2-3 below.
// It should acknowledge the pending bit for pin 2, and 
// it should call togglexn(GPIOB, 9).

void EXTI2_3_IRQHandler(){
  EXTI -> PR = EXTI_PR_PR2;
  togglexn(GPIOB, 9);
}

//==========================================================
// Write the EXTI interrupt handler for pins 4-15 below.
// It should acknowledge the pending bit for pin 4, and 
// it should call togglexn(GPIOB, 10).
void EXTI4_15_IRQHandler(){
  EXTI -> PR = EXTI_PR_PR4;
  togglexn(GPIOB, 10);
}


/**
 * @brief Enable the SysTick interrupt to occur every 1/16 seconds.
 * 
 */
void init_systick() {
  SysTick -> LOAD = 375000 - 1;
  SysTick -> VAL = 0;
  //SysTick->CTRL |= ( SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_CLKSOURCE_Msk );
  SysTick->CTRL = 0x00000003;
}

volatile int current_col = 1;

/**
 * @brief The ISR for the SysTick interrupt.
 * 
 */
void SysTick_Handler(void) {
    // 1. Read the row pins using GPIOC->IDR
    //    You can check the pins used for rows 
    //    of keypad in lab 1 manual
    int32_t temp = GPIOC -> IDR & 0xF;
    // 2. If the var `current_col` corresponds to
    //    the row value, toggle one of the leds connected 
    //    to PB8-11.
    if(((temp >> (4 - current_col)) & 1))
    {
      togglexn(GPIOB, current_col + 7);
    }
    //    Basically the same we have done in lab 1
    // 3. Increment the `current_col` and wrap around
    //    to 1 if `current_col` > 4. So that next time
    //    we scan the next column
    if(++current_col > 4)
    {
      current_col = 1;
    }
    // 4. Set the changed column pin designated by `current_col`
    //    to 1 and rest of the column pins to 0 to energized that
    //    particular column for next read of keypad.
    set_col(current_col);
}

/**
 * @brief For the keypad pins, 
 *        Set the specified column level to logic "high".
 *        Set the other three three columns to logic "low".
 * 
 * @param col 
 */
void set_col(int col) {
    // Set PC4-7 (i.e. all columns) output to be 0
    GPIOC -> BSRR |= 0xffff0000;
    GPIOC -> BSRR &= 0xffff0000;
    // Set the column `col` output to be 1
    GPIOC -> BSRR |= 1 << (8 - col);
    //  if col = 1, PC7 will be set to 1 as 
    //  it is connected to column 1 of the keypad 
    //  Likewise, if col = 4, PC4 will be set to 1
}

/**
 * @brief Set the priority for EXTI pins 2-3 interrupt to 192.
 *        Set the priority for EXTI pins 4-15 interrupt to 128.
 *        Do not adjust the priority for any other interrupts.
 * 
 */
void adjust_priorities() {
  NVIC_SetPriority(EXTI2_3_IRQn,3);
  NVIC_SetPriority(EXTI4_15_IRQn,2);
}
