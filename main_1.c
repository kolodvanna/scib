#include "MDR32F9Q2I.h"

//SERVO
#define PIN_SERVO_POS   2           // D10 - PA2
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500

//MOTORS
#define MOTORCLK_PORT   MDR_PORTB
#define MOTORCLK_PIN    0           // D4-PB0

#define MOTORLATCH_PORT MDR_PORTF
#define MOTORLATCH_PIN  3           // D12-PF3

#define EN_PORT         MDR_PORTA
#define MOTORENABLE_PIN 7           // D7-PA7

#define DATA_PORT       MDR_PORTA
#define MOTORDATA_PIN   6           // D8-PA6

#define PWM1_PORT       MDR_PORTA
#define PWM1_PIN        4           // D6-PA4
#define PWM2_PORT       MDR_PORTA
#define PWM2_PIN        5           // D5-PA5

#define PWM3_PORT       MDR_PORTA
#define PWM3_PIN        1           // D11-PA1
#define PWM4_PORT       MDR_PORTB
#define PWM4_PIN        2           // D3-PB2

//SOLAR MOTOR
//#define SOLAR_M1_PORT MDR_PORTA
//#define SOLAR_M1_PIN  2               // D6
//#define SOLAR_M2_PORT MDR_PORTA
//#define SOLAR_M2_PIN  1               // D5

//UART
#define TX_PIN_POS 5                  // PB5
#define RX_PIN_POS 6                  // PB6

//HS-SR04
#define TRIG_PIN_POS  1               // D2 - PB1
#define ECHO_PIN_POS  1               // D13 - PF1

//LINE TRACKER
#define LINE_LEFT_PORT   MDR_PORTD
#define LINE_LEFT_PIN    3            // PD3-A3
#define LINE_CENTER_PORT MDR_PORTD
#define LINE_CENTER_PIN  4            // PD4-A4
#define LINE_RIGHT_PORT  MDR_PORTD
#define LINE_RIGHT_PIN   5            // PD5-A5

//PHOTORESISTOR
#define ANALOG_PIN1__PORT MDR_PORTD
#define ANALOG_PIN1_PIN   0           // PD0-A0
#define ANALOG_PIN2_PORT  MDR_PORTD
#define ANALOG_PIN2_PIN   1           // PD1-A1

const int Forward       = 92;                               // forward
const int Backward      = 163;                              // back
const int Stop          = 0;                                // stop
const int Contrarotate  = 172;                              // Counterclockwise rotation
const int Clockwise     = 83;                               // Rotate clockwise
const int Moedl1        = 25;                               // model1
const int Moedl2        = 26;                               // model2
const int Moedl3        = 27;                               // model3
const int Moedl4        = 28;                               // model4
const int MotorLeft     = 230;                              // servo turn left
const int MotorRight    = 231;                              // servo turn right

int Black_Line = 700;
int leftDistance = 0;
int middleDistance = 0;
int rightDistance = 0;

uint16_t angle = 90;
uint8_t order = 0;
char model_var = 0;
int UT_distance = 0;

void Servo_attach(void) {
		MDR_RST_CLK->PER_CLOCK |= (1<<21); //initialization PORTA 
		MDR_RST_CLK->PER_CLOCK |= (1 << 15); //initialization TIM_CLK_2
		MDR_RST_CLK->TIM_CLOCK |= (1 << 25);
	
    //PIN PA2 configuration (PWM)
    MDR_PORTA->OE     |=  (1 << PIN_SERVO_POS);
    MDR_PORTA->FUNC   &= ~(0x3 << 2*PIN_SERVO_POS);
    MDR_PORTA->FUNC   |=  (0x3 << 2*PIN_SERVO_POS);
    MDR_PORTA->ANALOG |=  (1 << PIN_SERVO_POS);
    MDR_PORTA->PWR    |=  (0x3 << 2*PIN_SERVO_POS);
    
    //TIM_CLK_2 configuration (PWM)
    MDR_TIMER2->CNTRL = 0; //of
    MDR_TIMER2->PSG   = 8 - 1;          // Delitel = 1 mks
    MDR_TIMER2->ARR   = 20000 - 1;       // Period = 20ms 

    MDR_TIMER2->CH1_CNTRL = (0x6 << 9); //config
    MDR_TIMER2->CH1_CNTRL  |= (1 << 8); //on
    MDR_TIMER2->CH1_CNTRL1 = (2 << 2);

    MDR_TIMER2->CCR1 = 0; //nach pologenie 0
    MDR_TIMER2->CNTRL |= (1 << 0); //on
}


void Servo_write(uint8_t angle) {
    if (angle > 180) {angle = 180; }
    // map(angle, 0, 180, 50, 250)
    uint32_t ticks = SERVO_MIN_PULSE + (angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / 180;
    MDR_TIMER2->CCR1 = ticks; 
}
void delay(uint32_t ms) {
    uint32_t i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 2000; j++){
            __NOP();
        }
    }
}

void delayMicroseconds(uint32_t us) {
    SysTick->LOAD = (us * 8) - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = 5;
    
    while ((SysTick->CTRL & (1 << 16)) == 0);
    
    SysTick->CTRL = 0;
}
void HCSR04_Init(void) {
	MDR_RST_CLK->PER_CLOCK |= (1 << 22); //inizialization PORTB
	MDR_RST_CLK->PER_CLOCK |= (1 << 29); //inizialization PORTF
	
	//PIN PB1 (Trigger) configuration
	MDR_PORTB->OE     |=   (1 << TRIG_PIN_POS);
	MDR_PORTB->FUNC   &= ~(0x3 << (2*TRIG_PIN_POS));	
	MDR_PORTB->ANALOG |=  (1 << TRIG_PIN_POS);
	MDR_PORTB->PWR    |=  (0x3 << (2 * TRIG_PIN_POS));
	
	//PIN PF1 (Echo) configuration
	MDR_PORTF->OE     &= ~(1 << ECHO_PIN_POS);
	MDR_PORTF->FUNC   &= ~(0x3 << 2 * ECHO_PIN_POS);
	MDR_PORTF->ANALOG |=  (1 << ECHO_PIN_POS);
	
	SysTick->CTRL = 0;
}

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout) {
	uint32_t timeout_ticks = timeout * 8;
	SysTick->LOAD = 0x00FFFFFF;
        SysTick->VAL  = 0;
        SysTick->CTRL = 5;
	uint32_t start_time = SysTick->VAL;
	
	while (((MDR_PORTF->RXTX >> pin) & 1) == state) {
        if ((start_time - SysTick->VAL) > timeout_ticks) { SysTick->CTRL = 0; return 0; }
    }
    
    while (((MDR_PORTF->RXTX >> pin) & 1) != state) {
        if ((start_time - SysTick->VAL) > timeout_ticks) { SysTick->CTRL = 0; return 0; }
    }
    
    uint32_t pulse_start = SysTick->VAL;
    
    while (((MDR_PORTF->RXTX >> pin) & 1) == state) {
        if ((pulse_start - SysTick->VAL) > timeout_ticks) { SysTick->CTRL = 0; return 0; }
    }

    uint32_t pulse_end = SysTick->VAL;
    SysTick->CTRL = 0;
    
    return (pulse_start - pulse_end) / 8;
}

void pin_Mode(MDR_PORT_TypeDef* port, uint8_t pin_num, uint8_t mode) {
    if (port == MDR_PORTA) MDR_RST_CLK->PER_CLOCK |= (1 << 21);
    if (port == MDR_PORTB) MDR_RST_CLK->PER_CLOCK |= (1 << 22);
    if (port == MDR_PORTC) MDR_RST_CLK->PER_CLOCK |= (1 << 23);
    if (port == MDR_PORTD) MDR_RST_CLK->PER_CLOCK |= (1 << 24);
    if (port == MDR_PORTE) MDR_RST_CLK->PER_CLOCK |= (1 << 25);
    if (port == MDR_PORTF) MDR_RST_CLK->PER_CLOCK |= (1 << 29);
	
    port->ANALOG |= (1 << pin_num);                 
    port->FUNC   &= ~(3 << (2 * pin_num));          
    port->PWR    |=  (3 << (2 * pin_num));          
    
    if (mode) port->OE |= (1 << pin_num);           
    else      port->OE &= ~(1 << pin_num);          
}

void digital_Write(MDR_PORT_TypeDef* port, uint8_t pin_num, uint8_t val) {
    if (val) port->RXTX |=  (1 << pin_num);
    else     port->RXTX &= ~(1 << pin_num);
}
uint8_t digital_Read(MDR_PORT_TypeDef* port, uint8_t pin_num) {
    if (port->RXTX & (1 << pin_num)) {
        return 1;
    } else {
        return 0;
    }
}
void pwm_init(void) {
    MDR_RST_CLK->PER_CLOCK |= (1 << 21); // PORTA (PWM1, PWM2, PWM3)
    MDR_RST_CLK->PER_CLOCK |= (1 << 22); // PORTB (PWM4)

    MDR_RST_CLK->PER_CLOCK |= (1 << 14); // TIMER1
    MDR_RST_CLK->PER_CLOCK |= (1 << 16); // TIMER3
    
    MDR_RST_CLK->TIM_CLOCK |= (1 << 24);
    MDR_RST_CLK->TIM_CLOCK |= (1 << 26);

    //PA2,PA1,PA5 (PWM1,PW2,PW3)
    MDR_PORTA->OE     |=  (1 << PWM1_PIN) | (1 << PWM2_PIN) | (1 << PWM3_PIN);
    MDR_PORTA->FUNC   &= ~((3 << (2 * PWM1_PIN)) | (3 << (2 * PWM2_PIN)) | (3 << (2 * PWM3_PIN)));
    MDR_PORTA->FUNC   |=  ((2 << (2 * PWM1_PIN)) | (2 << (2 * PWM2_PIN)) | (2 << (2 * PWM3_PIN)));
    MDR_PORTA->ANALOG |=  (1 << PWM1_PIN) | (1 << PWM2_PIN) | (1 << PWM3_PIN);
    MDR_PORTA->PWR    |=  (3 << (2 * PWM1_PIN)) | (3 << (2 * PWM2_PIN)) | (3 << (2 * PWM3_PIN));

    MDR_TIMER1->CNTRL = 0;
    MDR_TIMER1->PSG   = 8 - 1;
    MDR_TIMER1->ARR   = 255;

    MDR_TIMER1->CH1_CNTRL  = (0x6 << 9) | (1 << 8);
    MDR_TIMER1->CH1_CNTRL1 = (2 << 2) | (2 << 0);
    
    MDR_TIMER1->CH2_CNTRL  = (0x6 << 9) | (1 << 8);
    MDR_TIMER1->CH2_CNTRL1 = (2 << 2) | (2 << 0);
    
    MDR_TIMER1->CH3_CNTRL  = (0x6 << 9) | (1 << 8);
    MDR_TIMER1->CH3_CNTRL1 = (2 << 2) | (2 << 0);

    MDR_TIMER1->CNTRL |= (1 << 0);
 
    //PB2(PW4)
    MDR_PORTB->OE     |=  (1 << PWM4_PIN);
    MDR_PORTB->FUNC   &= ~(3 << (2 * PWM4_PIN));
    MDR_PORTB->FUNC   |=  (2 << (2 * PWM4_PIN));
    MDR_PORTB->ANALOG |=  (1 << PWM4_PIN);
    MDR_PORTB->PWR    |=  (3 << (2 * PWM4_PIN));

    MDR_TIMER3->CNTRL = 0;
    MDR_TIMER3->PSG   = 8 - 1; 
    MDR_TIMER3->ARR   = 255;
    
    MDR_TIMER3->CH3_CNTRL  = (0x6 << 9) | (1 << 8);
    MDR_TIMER3->CH3_CNTRL1 = (2 << 2) | (2 << 0);
    
    MDR_TIMER3->CNTRL |= (1 << 0);

		//Solar
//    MDR_RST_CLK->PER_CLOCK |= (1 << 21);
//    MDR_RST_CLK->PER_CLOCK |= (1 << 14);
//    MDR_RST_CLK->TIM_CLOCK |= (1 << 24);

//    MDR_PORTA->OE     |=  (1 << SOLAR_M1_PIN);
//    MDR_PORTA->FUNC   &= ~(3 << (2 * SOLAR_M1_PIN));
//    MDR_PORTA->FUNC   |=  (2 << (2 * SOLAR_M1_PIN));
//    MDR_PORTA->ANALOG |=  (1 << SOLAR_M1_PIN);
//    MDR_PORTA->PWR    |=  (3 << (2 * SOLAR_M1_PIN));
//		
//    MDR_PORTA->OE     |=  (1 << SOLAR_M2_PIN);
//    MDR_PORTA->FUNC   &= ~(3 << (2 * SOLAR_M2_PIN));
//    MDR_PORTA->FUNC   |=  (2 << (2 * SOLAR_M2_PIN));
//    MDR_PORTA->ANALOG |=  (1 << SOLAR_M2_PIN);
//    MDR_PORTA->PWR    |=  (3 << (2 * SOLAR_M2_PIN));
//    
//    MDR_TIMER1->CH1_CNTRL = (0x6 << 9) | (1 << 8); 
//    MDR_TIMER1->CH1_CNTRL1 = (2 << 2) | (2 << 0);
}



void analog_Write(MDR_PORT_TypeDef* port, uint8_t pin_num, uint8_t spd) {
    if (port == MDR_PORTA && pin_num == 4){
        MDR_TIMER1->CCR2 = spd;
    }
    else if (port == MDR_PORTA && pin_num == 1) {
        MDR_TIMER1->CCR1 = spd;
    }
    else if (port == MDR_PORTA && pin_num == 5){
        MDR_TIMER1->CCR3 = spd;
    }
    else if (port == MDR_PORTB && pin_num == 2) {
        MDR_TIMER3->CCR3 = spd;
    }
}


void Serial_begin(void) {
    MDR_RST_CLK->PER_CLOCK |= (1 << 22);
    
    // PIN PB5 (TX) configuration
    MDR_PORTB->OE      |=  (1 << TX_PIN_POS);
    MDR_PORTB->FUNC    &= ~(3 << (2 * TX_PIN_POS));
    MDR_PORTB->FUNC    |=  (2 << (2 * TX_PIN_POS));
    MDR_PORTB->ANALOG  |=  (1 << TX_PIN_POS);
    MDR_PORTB->PWR     |=  (3 << (2 * TX_PIN_POS));
    
    // PIN PB6 (RX) configuration
    MDR_PORTB->OE      &= ~(1 << RX_PIN_POS);
    MDR_PORTB->FUNC    &= ~(3 << (2 * RX_PIN_POS));
    MDR_PORTB->FUNC    |=  (2 << (2 * RX_PIN_POS));
    MDR_PORTB->ANALOG  |=  (1 << RX_PIN_POS);
    MDR_PORTB->PWR     |=  (3 << (2 * RX_PIN_POS));
    
    // UART configuration
    MDR_RST_CLK->PER_CLOCK |= (1 << 6);
    MDR_RST_CLK->UART_CLOCK = (1 << 24);
    MDR_UART1->CR    = 0;
    
    MDR_UART1->IBRD  = 57; //or 55 in several MDR
    MDR_UART1->FBRD  = 5;
    
    MDR_UART1->LCR_H = (1 << 4) | (3 << 5);
    MDR_UART1->CR    = (1 << 0) | (1 << 8) | (1 << 9);
}

uint8_t Serial_available(void) {
    if ((MDR_UART1->FR & (1 << 4)) == 0) { return 1; } 
    else { return 0; }
}

void Serial_write(uint8_t data) {
    while (MDR_UART1->FR & (1 << 5)); 
    MDR_UART1->DR = data;
}

void Serial_print(char* str) {
    while (*str) {
        Serial_write(*str++);
    }
}

uint8_t Serial_read(void) {
    return (uint8_t)(MDR_UART1->DR & 0xFF);
}

void shiftOut(MDR_PORT_TypeDef* dPort, uint8_t dPin, MDR_PORT_TypeDef* cPort, uint8_t cPin, uint8_t val) {
    for (int i = 0; i < 8; i++) {
        if (val & (1 << (7 - i))) dPort->RXTX |= (1 << dPin);
        else                      dPort->RXTX &= ~(1 << dPin);
        
        cPort->RXTX |= (1 << cPin);
        delayMicroseconds(2);
        cPort->RXTX &= ~(1 << cPin);
        delayMicroseconds(2);
    }
}
void ADC_Init(void) {
    MDR_RST_CLK->PER_CLOCK |= (1 << 17);
		MDR_RST_CLK->PER_CLOCK |= (1 << 24); //PORTD
	
    MDR_ADC->ADC1_CFG = (1 << 12) | (0 << 17);
	
		MDR_PORTD->ANALOG &= ~((1 << 0) | (1 << 1) | (1 << 3) | (1 << 4) | (1 << 5));
		MDR_PORTD->OE     &= ~((1 << 0) | (1 << 1) | (1 << 3) | (1 << 4) | (1 << 5));
		MDR_PORTD->FUNC   &= ~((3 << (2*0)) | (3 << (2*1)) | (3 << (2*3)) | (3 << (2*4)) | (3 << (2*5)));
}

int analogRead(uint8_t channel) {
    MDR_ADC->ADC1_CFG = (1 << 12) | (channel << 4) | (1 << 11);
    while (!(MDR_ADC->ADC1_STATUS & (1 << 0))); 
    return (MDR_ADC->ADC1_RESULT & 0x0FFF) >> 2; 
}
void Light_val() {
  int sensorValue1 = analogRead(ANALOG_PIN1_PIN);
  int sensorValue2 = analogRead(ANALOG_PIN2_PIN);

  if((sensorValue1 >= 200) && (sensorValue1-sensorValue2 >= 100)){
//    analog_Write(SOLAR_M1_PORT, SOLAR_M1_PIN, 0); 
//    analog_Write(SOLAR_M2_PORT, SOLAR_M2_PIN, 150); 
//    delay(30);
//    analog_Write(SOLAR_M1_PORT, SOLAR_M1_PIN, 0);
//		analog_Write(SOLAR_M2_PORT, SOLAR_M2_PIN, 0);
//    delay(30);
  }else if((sensorValue2 >= 200) && (sensorValue2-sensorValue1 >= 100)){
//    analog_Write(SOLAR_M1_PORT, SOLAR_M1_PIN, 140); 
//    analog_Write(SOLAR_M2_PORT, SOLAR_M2_PIN, 0);
//    delay(30);
//    analog_Write(SOLAR_M1_PORT, SOLAR_M1_PIN, 0); 
//    analog_Write(SOLAR_M2_PORT, SOLAR_M2_PIN, 0);
//    delay(30);
  }else{
//    analog_Write(SOLAR_M1_PORT, SOLAR_M1_PIN, 0); 
//    analog_Write(SOLAR_M2_PORT, SOLAR_M2_PIN, 0);
  }
}

void Motor(int Dir, int SpeedPWM1, int SpeedPWM2, int SpeedPWM3, int SpeedPWM4)
{
    digital_Write(EN_PORT, MOTORENABLE_PIN, 0); // LOW
	
    analog_Write(PWM1_PORT, PWM1_PIN, SpeedPWM1);
    analog_Write(PWM2_PORT, PWM2_PIN, SpeedPWM2);
    analog_Write(PWM3_PORT, PWM3_PIN, SpeedPWM3);
    analog_Write(PWM4_PORT, PWM4_PIN, SpeedPWM4);

    digital_Write(MOTORLATCH_PORT, MOTORLATCH_PIN, 0);
    shiftOut(DATA_PORT, MOTORDATA_PIN, MOTORCLK_PORT, MOTORCLK_PIN, Dir);
    digital_Write(MOTORLATCH_PORT, MOTORLATCH_PIN, 1);
}

void motorleft()  //servo
{
    Servo_write(angle);
    angle+=20;
    if(angle >= 180) angle = 180;
    delay(10);
}
void motorright() //servo
{
    Servo_write(angle);
    angle-=20;
    if(angle <= 1) angle = 1;
    delay(10);
}

float SR04(void)
{
    digital_Write(MDR_PORTB, TRIG_PIN_POS, 0);
    delayMicroseconds(2);
    digital_Write(MDR_PORTB, TRIG_PIN_POS, 1);
    delayMicroseconds(10);
    digital_Write(MDR_PORTB, TRIG_PIN_POS, 0);
    float distance = pulseIn(ECHO_PIN_POS, 1, 30000) / 58.00;
    delay(10);
    return distance;
}

void RXpack_func()
{
    while (Serial_available() > 0)
    {
        uint8_t b = Serial_read();
        static int state = 0;
        static uint8_t cmd = 0;

        if (state == 0) {
            if (b == 0xA5) state = 1;
        } 
        else if (state == 1) {
            cmd = b;
            state = 2;
        } 
        else if (state == 2) {
            if (b == 0x5A) {
                order = cmd;
                
                if (order == Moedl1) {
                    model_var = 0;
                } 
                else if (order == Moedl2) {
                    model_var = 1;
                } 
                else if (order == Moedl3) {
                    model_var = 2;
                } 
                else if (order == Moedl4) {
                    model_var = 3;
                }
            }
            state = 0;
        }
    }
}

void model1_func(uint8_t orders)
{
    switch (orders)
    {
    case Stop:
        Motor(Stop, 0, 0, 0, 0);
        Light_val();
        break;
    case Forward:
        Motor(Forward, 255, 255, 255, 255);
        break;
    case Backward:
        Motor(Backward, 255, 255, 255, 255);
        break;
    case Clockwise:
        Motor(Clockwise, 255, 255, 255, 255);
        break;
    case Contrarotate:
        Motor(Contrarotate, 255, 255, 255, 255);
        break;
    case MotorLeft:
        motorleft();
        break;
    case MotorRight:
        motorright();
        break;
    default:
        order = 0;
        Motor(Stop, 0, 0, 0, 0);
        Light_val();
        break;
    }
}

void model2_func()      // OA
{
    Servo_write(90);
    UT_distance = SR04();
    middleDistance = UT_distance;

    if (middleDistance <= 25) 
    {
        Motor(Stop, 0, 0, 0, 0);
        for(int i = 0;i < 500;i++){
          delay(1);
          RXpack_func();
          if(model_var != 1)
            return ;
        }
        Servo_write(10);
        for(int i = 0;i < 300;i++){
          delay(1);
          RXpack_func();
          if(model_var != 1)
            return ;
        }
        rightDistance = SR04();
        Servo_write(90);
        for(int i = 0;i < 300;i++){
          delay(1);
          RXpack_func();
          if(model_var != 1)
            return ;
        }
        Servo_write(170);
        for(int i = 0;i < 300;i++){
          delay(1);
          RXpack_func();
          if(model_var != 1)
            return ;
        }
        leftDistance = SR04();
        Servo_write(90);
        if((rightDistance < 20) && (leftDistance < 20)){

            Motor(Backward, 180, 180, 180, 180);
            for(int i = 0;i < 1000;i++){
              delay(1);
              RXpack_func();
              if(model_var != 1)
                return ;
            }
            Motor(Contrarotate, 250, 250, 250, 250); 
            for(int i = 0;i < 500;i++){
              delay(1);
              RXpack_func();
              if(model_var != 1)
                return ;
            }
        }
        else if(rightDistance < leftDistance) {
            Motor(Stop, 0, 0, 0, 0);
            for(int i = 0;i < 100;i++){
              delay(1);
              RXpack_func();
              if(model_var != 1)
                return ;
            }
            Motor(Backward, 180, 180, 180, 180);
            for(int i = 0;i < 500;i++){
              delay(1);
              RXpack_func();
              if(model_var != 1)
                return ;
            }
            Motor(Contrarotate, 250, 250, 250, 250);
            for(int i = 0;i < 500;i++){
              delay(1);
              RXpack_func();
              if(model_var != 1)
                return ;
            }
        }//turn right
        else if(rightDistance > leftDistance){
            Motor(Stop, 0, 0, 0, 0);
            for(int i = 0;i < 500;i++){
              delay(1);
              RXpack_func();
              if(model_var != 1)
                return ;
            }
            Motor(Backward, 180, 180, 180, 180);
            for(int i = 0;i < 500;i++){
              delay(1);
              RXpack_func();
              if(model_var != 1)
                return ;
            }
            Motor(Clockwise, 250, 250, 250, 250);
            for(int i = 0;i < 500;i++){
              delay(1);
              RXpack_func();
              if(model_var != 1)
                return ;
            }
        }
        else{
            Motor(Backward, 180, 180, 180, 180);
            for(int i = 0;i < 500;i++){
              delay(1);
              RXpack_func();
              if(model_var != 1)
                return ;
            }
            Motor(Clockwise, 250, 250, 250, 250); 
            for(int i = 0;i < 500;i++){
              delay(1);
              RXpack_func();
              if(model_var != 1)
                return ;
            }
        }
    }
    else 
    {
        Motor(Forward, 250, 250, 250, 250);
    }
}

void model3_func()      // follow model
{
    Servo_write(90);  
    UT_distance = SR04();
    if (UT_distance < 15)
    {
        Motor(Backward, 200, 200, 200, 200);
    }
    else if (15 <= UT_distance && UT_distance <= 20)
    {
        Motor(Stop, 0, 0, 0, 0);
    }
    else if (20 <= UT_distance && UT_distance <= 25)
    {
        Motor(Forward, 180, 180, 180, 180);
    }
    else if (25 <= UT_distance && UT_distance <= 50)
    {
        Motor(Forward, 220, 220, 220, 220);
    }
    else
    {
        Motor(Stop, 0, 0, 0, 0);
    }
}

void model4_func()      // tracking model
{
    Servo_write(90);
    int Left_Tra_Value = analogRead(LINE_LEFT_PIN);
    int Center_Tra_Value = analogRead(LINE_CENTER_PIN);
    int Right_Tra_Value = analogRead(LINE_RIGHT_PIN);
    if (Left_Tra_Value < Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line)
    {
        Motor(Forward, 250, 250, 250, 250);
    }
    else if (Left_Tra_Value >= Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value < Black_Line)
    {
        Motor(Contrarotate, 220, 220, 220, 220);
    }
    else if (Left_Tra_Value >= Black_Line && Center_Tra_Value < Black_Line && Right_Tra_Value < Black_Line)
    {
        Motor(Contrarotate, 250, 250, 250, 250);
    }
    else if (Left_Tra_Value < Black_Line && Center_Tra_Value < Black_Line && Right_Tra_Value >= Black_Line)
    {
        Motor(Clockwise, 250, 250, 250, 250);
    }
    else if (Left_Tra_Value < Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value >= Black_Line)
    {
        Motor(Clockwise, 220, 220, 220, 220);
    }
    else if (Left_Tra_Value >= Black_Line && Center_Tra_Value >= Black_Line && Right_Tra_Value >= Black_Line)
    {
        Motor(Stop, 0, 0, 0, 0);
    }
}


int main(void) {
	Serial_begin();
  pwm_init();
  HCSR04_Init();
  Servo_attach();
  ADC_Init();
	
	
	pin_Mode(MOTORCLK_PORT, MOTORCLK_PIN, 1);
  pin_Mode(MOTORLATCH_PORT, MOTORLATCH_PIN, 1);
  pin_Mode(EN_PORT, MOTORENABLE_PIN, 1);
  pin_Mode(DATA_PORT, MOTORDATA_PIN, 1);
	
  Servo_write(angle);
  Motor(Stop, 0, 0, 0, 0);
	
	while(1)
		{
			RXpack_func();
			switch (model_var)
			{
			case 0:
					model1_func(order);
					break;
			case 1:
					model2_func();      // OA model
					break;
			case 2:
					model3_func();      // follow model
					break;
			case 3:
					model4_func();      // Tracking model
					break;
			}
		}
}
