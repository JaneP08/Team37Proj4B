#ifndef POTENTIOMETER
#define POTENTIOMETER


volatile uint16_t displayValue;
volatile float speed_angle;
volatile float RPMs;



void Analog2Digital(void);
void HallSensorInit(void);
void timer3_set(int prescaler, int micseconds, int priority);
void timer2_set(int prescaler, int micseconds, int priority);

#endif // POTENTIOMETER