// ****************************************************************
// * TEAM50: T. LUM and R. MARTIN
// * CPEG222 servo_main, 10/6/25
// * NucleoF466RE CMSIS STM32F4xx example
// * Move the standard servo using PWM on PC6
// * Send angle and pulse width data over UART2
// * measured data shows +/- 45 deg rotation using 1ms to 2ms pulse width
// ****************************************************************
#include "stm32f4xx.h"
#include "UART2.h"
#include <stdio.h>
#include "potentiometer.h"
#include <stdbool.h>
#include "SSD_Array.h"


#define FREQUENCY   16000000UL // 16 MHz  
#define SERVO3_PIN    (6) // Assuming servo motor 3 control pin is connected to GPIOC pin 6
#define SERVO3_PORT   (GPIOC)

// 4B IR sensor definitions
#define IR_PORT GPIOB // PB0 - PB3 = 4 IR sensors
#define IR_MASK 0x0F // Read lower 4 bits (0000 1111)

uint32_t pulse_width = 0; //removed volatile not used in an interrupt
volatile bool FORWARD = true;
volatile bool PAUSE = true;
volatile int digitSelect = 0;
volatile uint16_t displayValue = 0;

void IR_Init(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable GPIOB clock
	IR_PORT->MODER &= ~0x000000FF; // Set PB0 - PB3 as input (00)
}

void follow_line(uint8_t ir)
{
	/** 
	 * IR bit meanings!!!!!!!
	 * 0b0001 = far right sensor
	 * 0b0010 = right center sensor
	 * 0b0100 = left center sensor
	 * 0b1000 = far left sensor
	 */

	if (ir == 0b0000){
		servo_angle_set(0); // no line detected so go straight
		return;
	}

	if (ir & 0b1000){
		servo_angle_set(+30); // far left 
	}
	else if (ir & 0b0100){
		servo_angle_set(+15); // left center
	}
	else if (ir & 0b0010){
		servo_angle_set(-15); // right center
	}
	else if (ir & 0b0001){
		servo_angle_set(-30); // far right
	}
	else {
		servo_angle_set(0); // fall back to straight
	}

}


void PWM_Output_PC6_Init(void) {
	// Enable GPIOC and TIM3 clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	// Set PC6 to alternate function (AF2 for TIM3_CH1)
	GPIOC->MODER &= ~(0x3 << (SERVO3_PIN * 2));
	GPIOC->MODER |=  (0x2 << (SERVO3_PIN * 2)); // Alternate function
	GPIOC->AFR[0] &= ~(0xF << (SERVO3_PIN * 4));
	GPIOC->AFR[0] |=  (0x2 << (SERVO3_PIN * 4)); // AF2 = TIM3
	// Configure TIM3 for PWM output on CH1 (PC6)
	TIM3->PSC = (FREQUENCY/1000000) - 1; // 16 MHz / 16 = 1 MHz timer clock (1us resolution)
	TIM3->ARR = 19999; // Period for 50 Hz
	TIM3->CCR1 = 1500; // Duty cycle (1.475 ms pulse width)
	TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M);
	TIM3->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // PWM mode 1
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE; // Preload enable
	TIM3->CCER |= TIM_CCER_CC1E; // Enable CH1 output (PC6)
	TIM3->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable
	TIM3->EGR = TIM_EGR_UG; // Generate update event
	TIM3->CR1 |= TIM_CR1_CEN; // Enable timer
}

void servo_angle_set(int angle) {
	pulse_width = 1500 - (500 * (angle/45.0)); // Map angle to pulse width (1ms to 2ms)
	TIM3->CCR1 = pulse_width;
}

// External interrupt handler for button press
void EXTI15_10_IRQHandler(void) {
  if (EXTI->PR & (1 << 13)) { // Check if the interrupt is from BTN_PIN
	if (PAUSE == false){
		PAUSE = true;
		EXTI->PR |= (1 << 13);    // Clear the pending interrupt
		uart2_send_string("paused");
	}
	else if (PAUSE == true){
		FORWARD = !FORWARD;
		EXTI->PR |= (1 << 13);
		uart2_send_string("reversing");
		PAUSE = false;
	}
  }
}


volatile int current_angle = 0;
volatile uint32_t last_rising = 0;
volatile uint32_t last_falling = 0;
volatile uint32_t hall_pulse_width = 0;
volatile uint8_t waiting_for_falling = 0;
int min_pulse_width = 32; // minimum encoder pulse width in microseconds
int max_pulse_width = 1076; // maximum encoder pulse width in microseconds

// TIM3 input capture interrupt handler for PC7 (CH2)
void TIM3_IRQHandler(void) {
	if (TIM3->SR & TIM_SR_CC2IF) { // Check if CC2IF is set
		TIM3->SR &= ~TIM_SR_CC2IF; // Clear interrupt flag
		if (!waiting_for_falling) {
			last_rising = TIM3->CCR2;
			// Switch to capture falling edge
			TIM3->CCER |= TIM_CCER_CC2P; // Set to falling edge
			waiting_for_falling = 1;
		} else {
			last_falling = TIM3->CCR2;
			if (last_falling >= last_rising){
				if (last_falling - last_rising < 1100) 
				pulse_width = last_falling - last_rising;
				current_angle = (pulse_width - min_pulse_width)*360/(max_pulse_width - min_pulse_width);
			} else
			hall_pulse_width = (0xFFFF - last_rising) + last_falling + 1;
			// Switch back to capture rising edge
			TIM3->CCER &= ~TIM_CCER_CC2P; // Set to rising edge
			waiting_for_falling = 0;
		}
	}
}

// Timer that generates a 0.5 ms interrupt to update the SSD array every 2 milliseconds
void TIM2_IRQHandler(void){
  if(TIM2->SR & TIM_SR_UIF){ // Check if the update interrupt flag is set
    SSD_update(digitSelect, displayValue, 3); // Update the SSD with the current value
    digitSelect = (digitSelect + 1) % 4; // Cycle through digitSelect values 0 to 3
    TIM2->SR &= ~TIM_SR_UIF; // Clear the update interrupt flag
  }
}

void SysTick_Handler(void) { // SysTick for writing to serial monitor
	if (PAUSE) {
		 displayValue = 0;
	} else { displayValue = 10*RPMs;}
	uart2_send_string("ADC Value: ");
	Analog2Digital();
	uart2_send_string("angle(deg): ");
	uart2_send_int32(speed_angle);
	uart2_send_string("servo pulsewidth(us): ");
	uart2_send_int32(pulse_width);
	uart2_send_string("Dir: ");
	if (!PAUSE){
		if (FORWARD){
			uart2_send_string("FORWARD\n");
		} 
		if (!FORWARD){
			uart2_send_string("BACKWARD\n");
		}
		uart2_send_string("RPM: ");
		uart2_send_int32(RPMs);
	} else {
		uart2_send_string("PAUSED\n");
		uart2_send_string("RPM: ");
		uart2_send_int32(0);
	}

	uart2_send_string("\r\n");	
}


int main(void) {
	UART2_Init();
	SSD_init(); // Initialize SSD GPIO pins
	PWM_Output_PC6_Init();
	uart2_send_string("CPEG222 Standard Servo Demo Program!\r\n");
	uart2_send_string("Setting angle to 0 degrees.\r\n");
	SysTick_Config(16000000/2); // Configure SysTick for 500 millisecond interrupts
	int angle = 0;

	timer2_set(16, 500, 2); //1MHz, 1 tick == 1 microsecond, 500micros gives us .5ms
	timer3_set(16, 0xFFFF, 1);
	HallSensorInit();

	IR_Init();


  // 5. Configure external interrupt for the button (GPIOC)
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //ENABLES GPIOC
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable SYSCFG clock for EXTI
  GPIOC->MODER &= ~(3 << (13 * 2)); // Set BTN_PIN as input by clearing MODER bits
  EXTI->IMR |= (1 << 13);  // Unmask EXTI line 13
  EXTI->FTSR |= (1 << 13); // Trigger on falling edge
  SYSCFG->EXTICR[3] &= ~(0xF << (1 * 4)); // Clear EXTI13 bits
  SYSCFG->EXTICR[3] |= (2 << (1 * 4));    // Map EXTI13 to PC13
  NVIC_EnableIRQ(EXTI15_10_IRQn); // Enable EXTI line[15:10] interrupts in NVIC
  NVIC_SetPriority(EXTI15_10_IRQn, 3);    // Set priority for EXTI




	servo_angle_set(angle);
	for (volatile int i = 0; i < 100000UL; ++i); // long delay to adjust the horn at 90 degrees
	while (1) {

		while (FORWARD == true && PAUSE == false){
			uint8_t ir = IR_PORT->IDR & IR_MASK;
			follow_line(ir);
		}

		while (FORWARD == false && PAUSE == false){
			uint8_t ir = IR_PORT->IDR & IR_MASK;
			follow_line(ir);
		}

		while (PAUSE == true){
			servo_angle_set(0); // holds wheels straight
		}
	}
}
