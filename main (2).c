/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An, Niraj Menon
  * @date    Jan 31 2024
  * @brief   ECE 362 Lab 5 Student template
  ******************************************************************************
*/

/*******************************************************************************/

// Fill out your username, otherwise your completion code will have the 
// wrong username!
const char* username = "ngollins";

/*******************************************************************************/ 

#include "stm32f0xx.h"
#include <math.h>   // for M_PI

void nano_wait(int);

// 16-bits per digit.
// The most significant 8 bits are the digit number.
// The least significant 8 bits are the segments to illuminate.
uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };
extern const char font[];
// Print an 8-character string on the 8 digits
void print(const char str[]);
// Print a floating-point value.
void printfloat(float f);

void autotest(void);
    
//============================================================================
// PWM Lab Functions
//============================================================================
void setup_tim3(void) {
    //Enable clocks
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    

    GPIOC -> MODER &= ~0x000ff000;
    GPIOC -> MODER |= 0x000aa000;

    //Set GPIOC6-9
    // GPIOC->AFRL_AFR6[3:0] = 0001;
    // GPIOC->AFRL_AFR7[3:0] = 0001;
    GPIOC-> AFR[0] &= ~0xff000000;
    //GPIOC-> AFR[0] |= 0x11000000;
    // GPIOC->AFRH_AFR0[3:0] = 0001;
    // GPIOC->AFRH_AFR1[3:0] = 0001;
    GPIOC-> AFR[1] &= ~0x000000ff;
    //GPIOC-> AFR[1] |= 0x00000011;

    //Set frequency
    TIM3 -> PSC = 48000 - 1;
    TIM3 -> ARR = 1000 - 1;

    //Configure TIM3 for PWM mode 1
    TIM3 -> CCMR1 &= 0x8f8f;
    TIM3 -> CCMR1 |= 0x6060;
    TIM3 -> CCMR2 &= 0x8f8f;
    TIM3 -> CCMR2 |= 0x6060;

    //Enable CCER channels
    //TIM3 -> CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E | TIM_CCER_CC2E | TIM_CCER_CC1E;

    TIM3 -> CCER |= TIM_CCER_CC1E;
    TIM3 -> CCER |= TIM_CCER_CC2E;
    TIM3 -> CCER |= TIM_CCER_CC3E;
    TIM3 -> CCER |= TIM_CCER_CC4E;
    TIM3 -> CR1 |= TIM_CR1_CEN;

    //Set CCR registers
    TIM3 -> CCR1 = 800;
    TIM3 -> CCR2 = 600;
    TIM3 -> CCR3 = 400;
    TIM3 -> CCR4 = 200;

    //Enable timer
   




}

void setup_tim1(void) {
    // Generally the steps are similar to those in setup_tim3
    // except we will need to set the MOE bit in BDTR. 
    // Be sure to do so ONLY after enabling the RCC clock to TIM1.

    // Enable clocks
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    //PA8-11 being set to 10 for alternative funtion mode
    GPIOA -> MODER &= ~0x00ff0000;
    GPIOA -> MODER |= 0x00aa0000;

    //Set GPIOA8-11, contingent on last one working
    // GPIOA->AFRH_AFR2[3:0] = 0001;
    // GPIOA->AFRH_AFR3[3:0] = 0001;
    // GPIOA->AFRH_AFR0[3:0] = 0001;
    // GPIOA->AFRH_AFR1[3:0] = 0001;

    GPIOA-> AFR[1] &= ~0x0000ffff;
    GPIOA-> AFR[1] |= 0x00002222;

    //Set MOE
    TIM1 -> BDTR |= 0x8000;

    //Set freq
    TIM1 -> PSC = 1 - 1;
    TIM1 -> ARR = 2400 - 1;

    //Enable channel outputs
    TIM1 -> CCMR1 &= 0x8f8f;
    TIM1 -> CCMR1 |= 0x6060;
    TIM1 -> CCMR2 &= 0x878f;
    TIM1 -> CCMR2 |= 0x6860;
    //Enabled the OC4CE in channel 4
    TIM1 -> CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E | TIM_CCER_CC2E | TIM_CCER_CC1E;

    //enable timer
    TIM1 -> CR1 |= TIM_CR1_CEN;








}

int getrgb(void);

// Helper function for you
// Accept a byte in BCD format and convert it to decimal
uint8_t bcd2dec(uint8_t bcd) {
    // Lower digit



    
    uint8_t dec = bcd & 0xF;

    // Higher digit
    dec += 10 * (bcd >> 4);
    return dec;
}

void setrgb(int rgb) {
    uint8_t b = bcd2dec(rgb & 0xFF);
    uint8_t g = bcd2dec((rgb >> 8) & 0xFF);
    uint8_t r = bcd2dec((rgb >> 16) & 0xFF);

    // int bTen = 10 * bcd2dec(b & 0xF);
    // int bOne = bcd2dec((b >> 4) & 0xF);
    // int gTen = 10 * bcd2dec(g & 0xF);
    // int gOne = bcd2dec((g >> 4) & 0xF);
    // int rTen = 10 * bcd2dec(r & 0xF);
    // int rOne = bcd2dec((r >> 4) & 0xF);


    //Need to invert since value 30 = on for 70% of the time
    TIM1 -> CCR3 = 24 * (100 - b);
    TIM1 -> CCR2 = 24 * (100 - g);
    TIM1 -> CCR1 = 24 * (100 - r);

    

    // TODO: Assign values to TIM1->CCRx registers
    // Remember these are all percentages
    // Also, LEDs are on when the corresponding PWM output is low
    // so you might want to invert the numbers.
}



//============================================================================
// Lab 4 code
// Add in your functions from previous lab
//============================================================================

// Part 3: Analog-to-digital conversion for a volume level.
uint32_t volume = 2400;

// Variables for boxcar averaging.
#define BCSIZE 32
int bcsum = 0;
int boxcar[BCSIZE];
int bcn = 0;

void dialer(void);

// Parameters for the wavetable size and expected synthesis rate.
#define N 1000
#define RATE 20000
short int wavetable[N];
int step0 = 0;
int offset0 = 0;
int step1 = 0;
int offset1 = 0;

//============================================================================
// enable_ports()
//============================================================================
void enable_ports(void) {

  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  GPIOB -> MODER &= 0xffff0000;
  GPIOC -> MODER &= 0xffff0000; 
  GPIOB -> MODER |= 0x00155555;
  GPIOC -> OTYPER |= 0x000000f0;
  GPIOC -> MODER |= 0x00005500;
  GPIOC -> PUPDR |= 0x00000055;  
  
    
}

//============================================================================
// setup_dma() + enable_dma()
//============================================================================

    void setup_dma(void) {
    //Channel 5

    //Turn off enable
    DMA1_Channel5 -> CCR &= ~DMA_CCR_EN;
    //0xfffffffe

    //Activate
    RCC -> AHBENR |= RCC_AHBENR_DMA1EN;
    //CMAR
    DMA1_Channel5 -> CMAR = (uint32_t) &msg;
    //CPAR
    DMA1_Channel5 -> CPAR = (uint32_t) &(GPIOB -> ODR);
    //CNDTR
    DMA1_Channel5 -> CNDTR = 8;
    //DIR
     DMA1_Channel5 -> CCR |= DMA_CCR_DIR;//page 209 manual
    //0x00000010

    //MINC
    DMA1_Channel5 -> CCR |= DMA_CCR_MINC;
    //0x00000080

    //Memory size
    DMA1_Channel5 -> CCR |= DMA_CCR_MSIZE_0;
    //0x00000400

    //P datum
    DMA1_Channel5 -> CCR |= DMA_CCR_PSIZE_0;
    //0x00000100

    //Circular mode

    DMA1_Channel5 -> CCR |= DMA_CCR_CIRC;
    //0x00000020




}

//============================================================================
// init_tim15()
//============================================================================
void enable_dma(void) {
    DMA1_Channel5 -> CCR |= 0x00000001;
}

//============================================================================
// init_tim15()
//============================================================================
void init_tim15(void) {
    RCC -> APB2ENR |= 0x00010000;
    TIM15 -> DIER |= 0x00000100;
    TIM15 -> PSC = 24000 - 1;
    TIM15 -> ARR = 2 - 1;
    TIM15 -> CR1 |= 0x00000001;

}

//=============================================================================
// Part 2: Debounced keypad scanning.
//=============================================================================

uint8_t col; // the column being scanned

void drive_column(int);   // energize one of the column outputs
int  read_rows();         // read the four row inputs
void update_history(int col, int rows); // record the buttons of the driven column
char get_key_event(void); // wait for a button event (press or release)
char get_keypress(void);  // wait for only a button press event.
float getfloat(void);     // read a floating-point number from keypad
void show_keys(void);     // demonstrate get_key_event()

//============================================================================
// The Timer 7 ISR
//============================================================================
// Write the Timer 7 ISR here.  Be sure to give it the right name.

void TIM7_IRQHandler(void)
{
    TIM7->SR &= ~TIM_SR_UIF;
    int rows = read_rows();
    update_history(col, rows);
    col = (col + 1) & 3;
    drive_column(col);
}


//============================================================================
// init_tim7()
//============================================================================
void init_tim7(void) {
  RCC->APB1ENR |= 0x00000020;
   
  TIM7 -> PSC = 24000 - 1;
  TIM7 -> ARR = 2 - 1;

  TIM7 -> DIER |= TIM_DIER_UIE;

  NVIC -> ISER[0] = 1 << TIM7_IRQn;

  TIM7 -> CR1 |= TIM_CR1_CEN;
}

//============================================================================
// setup_adc()
//============================================================================

void setup_adc(void) {
    GPIOA -> MODER &= 0xfffffff0;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA -> MODER |= 0x0000000C;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; 
    RCC->CR2 |= RCC_CR2_HSI14ON;
    while (!(RCC->CR2 & RCC_CR2_HSI14RDY));
    ADC1->CR |= ADC_CR_ADEN;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
    ADC1 -> CHSELR |= ADC_CHSELR_CHSEL1;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));


}


//============================================================================
// Timer 2 ISR
//============================================================================
// Write the Timer 2 ISR here.  Be sure to give it the right name.
    void TIM2_IRQHandler(void)
    {
        TIM2->SR &= ~TIM_SR_UIF;
        ADC1 -> CR |= 0x00000004;
        while(!(ADC1->ISR & ADC_ISR_EOC));
        bcsum -= boxcar[bcn];
        bcsum += boxcar[bcn] = ADC1->DR;
        bcn += 1;
        if (bcn >= BCSIZE)
            bcn = 0;
        volume = bcsum / BCSIZE;

    }


//============================================================================
// init_tim2()
//============================================================================
void init_tim2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    //0x00000001;
   
    TIM2 -> PSC = 24000 - 1;
    TIM2 -> ARR = 200 - 1;

    TIM2 -> DIER |= TIM_DIER_UIE;

    NVIC -> ISER[0] = 1 << TIM2_IRQn;

    TIM2 -> CR1 |= TIM_CR1_CEN;
}

//============================================================================
// setup_dac()
//============================================================================
    void setup_dac(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA -> MODER |= 3 << 8;
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    //DAC -> CR |= DAC_CR_TSEL6; 
    DAC -> CR &= 0xffffff37;
    //Bits 5:3 set to 000
    DAC -> CR |= DAC_CR_TEN1;
    DAC -> CR |= DAC_CR_EN1;



}

//============================================================================
// Timer 6 ISR
//============================================================================
// Write the Timer 6 ISR here.  Be sure to give it the right name.
void TIM6_DAC_IRQHandler(void)
    {
        TIM6 -> SR &= ~TIM_SR_UIF;
        offset0 += step0;
        offset1 += step1;
        if (offset0 >= (N << 16))
            offset0 -= (N << 16);
        if (offset1 >= (N << 16))
            offset1 -= (N << 16);

        int samp = wavetable[offset0>>16] + wavetable[offset1>>16];
        int sample = ((samp * volume)>>18) + 1200;
        // samp *= volume;TIM1_CCR4 
        // samp = samp >> 17;
        // samp += 2048;
        TIM1 -> CCR4  = sample;
    }

//============================================================================
// init_tim6()
//============================================================================
void init_tim6(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
   
    // TIM6 -> PSC = (24000 / (RATE)) - 1;
    // TIM6 -> ARR = 2 - 1;

    TIM6 -> PSC = 48 - 1;
    TIM6 -> ARR = 1000000 / RATE - 1;

    TIM6 -> DIER |= TIM_DIER_UIE;

    NVIC -> ISER[0] = 1 << TIM6_DAC_IRQn;

    //TIM6 -> CR2 |= 0x00000020;
    //Bits 6-4 = 001

    TIM6 -> CR1 |= TIM_CR1_CEN;
}

//===========================================================================
// init_wavetable()
// Write the pattern for a complete cycle of a sine wave into the
// wavetable[] array.
//===========================================================================

void init_wavetable(void) {
    for(int i=0; i < N; i++)
        wavetable[i] = 32767 * sin(2 * M_PI * i / N);
}

//============================================================================
// set_freq()
//============================================================================
void set_freq(int chan, float f) {
    if (chan == 0) {
        if (f == 0.0) {
            step0 = 0;
            offset0 = 0;
        } else
            step0 = (f * N / RATE) * (1<<16);
    }
    if (chan == 1) {
        if (f == 0.0) {
            step1 = 0;
            offset1 = 0;
        } else
            step1 = (f * N / RATE) * (1<<16);
    }
}

//============================================================================
// All the things you need to test your subroutines.
//============================================================================
int main(void) {
    internal_clock();

    // Uncomment autotest to get the confirmation code.
    autotest();

    // Demonstrate part 1
 #define TEST_TIMER3
#ifdef TEST_TIMER3
    setup_tim3();
    for(;;) { }
#endif

    // Initialize the display to something interesting to get started.
    msg[0] |= font['E'];
    msg[1] |= font['C'];
    msg[2] |= font['E'];
    msg[3] |= font[' '];
    msg[4] |= font['3'];
    msg[5] |= font['6'];
    msg[6] |= font['2'];
    msg[7] |= font[' '];

    enable_ports();
    setup_dma();
    enable_dma();
    init_tim15();
    init_tim7();
    setup_adc();
    init_tim2();
    init_wavetable();
    init_tim6();

    setup_tim1();

    // demonstrate part 2
 #define TEST_TIM1
#ifdef TEST_TIM1
    for(;;) {
        // Breathe in...
        for(float x=1; x<2400; x *= 1.1) {
            TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 2400-x;
            nano_wait(100000000);
        }
        // ...and out...
        for(float x=2400; x>=1; x /= 1.1) {
            TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 2400-x;
            nano_wait(100000000);
        }
        // ...and start over.
    }
#endif

    // demonstrate part 3
 #define MIX_TONES
#ifdef MIX_TONES
    set_freq(0, 1000);
    for(;;) {
        char key = get_keypress();
        if (key == 'A')
            set_freq(0,getfloat());
        if (key == 'B')
            set_freq(1,getfloat());
    }
#endif

    // demonstrate part 4
 #define TEST_SETRGB
#ifdef TEST_SETRGB
    for(;;) {
        char key = get_keypress();
        if (key == 'A')
            set_freq(0,getfloat());
        if (key == 'B')
            set_freq(1,getfloat());
        if (key == 'D')
            setrgb(getrgb());
    }
#endif

    // Have fun.
    dialer();
}
