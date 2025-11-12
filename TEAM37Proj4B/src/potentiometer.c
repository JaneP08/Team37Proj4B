#include "stm32f4xx.h"
#include "potentiometer.h"
#include "UART2.h"
#include "SSD_Array.h"

#define ANALOG_PIN 1
#define ANALOG_PORT GPIOA
#define ADC_CHANNEL 1 // ADC Channel for PA1
#define ADC_SAMPLES 16 // Number of samples for averaging

volatile float speed_angle = 0.0f;
volatile float RPMs = 0.0f;


void Analog2Digital(void) {
    // Set PA1 (ADC) to analog mode
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    ANALOG_PORT->MODER &= ~(0x3 << (ANALOG_PIN * 2));
    ANALOG_PORT->MODER |= (0x3 << (ANALOG_PIN * 2));
    // Initialize ADC, Default resolution is 12 bits
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Enable ADC1 clock
    ADC1->SQR3 = ADC_CHANNEL; // Select channel
    ADC1->SMPR2 = ADC_SMPR2_SMP1_0 | ADC_SMPR2_SMP1_1; // Sample time 56 cycles
    ADC1->CR2 = ADC_CR2_ADON; // Enable ADC
    uint32_t total = 0;
    for(int i = 0; i < ADC_SAMPLES; i++){
        ADC1->CR2 |= ADC_CR2_SWSTART; // Start conversion
        while(!(ADC1->SR & ADC_SR_EOC)); // Wait for conversion to complete
        total += ADC1->DR; // Read data
    }
    uint16_t average = total / ADC_SAMPLES;
   // float voltage = (average / 4095.0) * 3.3; // Convert to voltage
    uart2_send_int32(average);
    speed_angle = (average/4095.0f)*45.0f;
    RPMs = (average/4095.0f)*140.0f;
}

void HallSensorInit(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	// Set PC7 to alternate function (AF2 for TIM3_CH2)
	GPIOC->MODER &= ~(0x3 << (7 * 2));
	GPIOC->MODER |=  (0x2 << (7 * 2)); // Alternate function
	GPIOC->AFR[0] &= ~(0xF << (7 * 4));
	GPIOC->AFR[0] |=  (0x2 << (7 * 4)); // AF2 = TIM3

}

void timer3_set(int prescaler, int micseconds, int priority){
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Enable TIM3 clock in reset and clock control
    TIM3->BDTR |= TIM_BDTR_MOE; // Main output enable for advanced timer
    TIM3->PSC = prescaler - 1;  //Set TIM3 prescaler to to generate clock control
    TIM3->ARR = micseconds - 1; //Set TIM3 to auto reload register to x-1 [remember starts at 0]
    TIM3->CR1 |= TIM_CR1_CEN;
	// Configure TIM3 for simple input capture on CH2 (PC7)
	TIM3->PSC = 15; // 16 MHz / 16 = 1 MHz timer clock (1us resolution)
	TIM3->ARR = 0xFFFF;
	TIM3->CCMR1 &= ~(0x3 << 8); // CC2S bits for CH2
	TIM3->CCMR1 |= (0x01 << 8); // CC2: IC2 mapped to TI2
	TIM3->CCER &= ~(TIM_CCER_CC2P | TIM_CCER_CC2NP); // Rising edge
	TIM3->CCER |= TIM_CCER_CC2E; // Enable capture on CH2
	TIM3->DIER |= TIM_DIER_CC2IE; // Enable capture/compare 2 interrupt
	TIM3->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM3_IRQn); // Enable TIM3 interrupt in NVIC
}

//Uses TIM2 to generate an interrupt every x miliseconds
void timer2_set(int prescaler, int micseconds, int priority){
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //Enable TIM2 clock in reset and clock control
    TIM2->PSC = prescaler - 1;  //Set TIM2 prescaler to to generate clock control
    TIM2->ARR = micseconds - 1; //Set TIM2 to auto reload register to x-1 [remember starts at 0]
    TIM2->DIER |= TIM_DIER_UIE; //Enable TIM2 update interrupt
    TIM2->SR &= ~TIM_SR_UIF;    //Clear any existing interrupt flags
    NVIC_EnableIRQ(TIM2_IRQn);  //Enable the interrupt in the NVIC
    NVIC_SetPriority(TIM2_IRQn, priority); //Set priority
    TIM2->CR1 |= TIM_CR1_CEN;    //Create TIM2_IRQHandler procedure and clear the interrupt flag
}