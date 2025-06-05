//**************************************************************************************************
// Name: PRIYANKA,URWANG,SHARAD,PRISHAD
// Date: 07-12-2023
// Course: ELEC3371-00
// Description: This program toggles the LEDs on PORTD and PORTE on an interrupt basis if either
//                                PA0 or PB6 are pressed. This is an example of an external interrupt. Each interrupt
//                                is non-maskable, meaning the CPU cannot ignore the interrupt request.
//**************************************************************************************************
//This program toggles LEDs on PORTE and PORTD if PA0 //or PB6 are pressed respectively on an interrupt basis.

//**************************************************************************************************
//INTERRUPT SERVICE ROUTINES
/*void EXTIPB6_PA7() iv IVT_INT_EXTI9_5 ics ICS_AUTO {
           if(EXTI_PR.B6 = 1){    // Clear pending interrupt flag for PB6
               EXTI_PR.B6 = 1 ;
             GPIOD_ODR = ~ GPIOD_ODR;
           }
           if(EXTI_PR.B7 = 1){    // Clear pending interrupt flag for PB6
               EXTI_PR.B7 = 1 ;
             GPIOC_ODR = ~ GPIOC_ODR;
           }

}

*/
int counter;
unsigned int rcvrd;
unsigned int adcVal;

unsigned int getAdcReading();


void JoyStickLeft() iv IVT_INT_EXTI2 {
EXTI_PR.B2 = 1;
 if(GPIOD_IDR.B2 == 0)            //LEFT: When Joystick button moved to left PE13 and PE14 Leds's will turn on
      GPIOE_ODR = 0b01100000 << 8;

 }
 void JoyStickUp() iv IVT_INT_EXTI4 ics ICS_AUTO {

      if(GPIOD_IDR.B4 == 0)               //UP: When Joystick button moved to Upper side then PE11 and PE15 Leds's will turn on
      GPIOE_ODR =0b10001000  << 8;
}

void JoyStickRightandDown() iv IVT_INT_EXTI9_5 ics ICS_AUTO {

      if(GPIOB_IDR.B5 == 0)                //DOWN: When Joystick button moved to down PE8 and PE12 Leds's will turn on
      GPIOE_ODR = 0b00010001 << 8;

      if(GPIOC_IDR.B13 == 0)                //CLICK: When Joystick button pressed all Leds's will turn on
      GPIOE_ODR = 0b11111111 << 8;
}

void EXTIPA0 () iv IVT_INT_EXTI0 {
EXTI_PR.B0 = 1;     //Clear pending interrupt flag for PA0
GPIOE_ODR = ~ GPIOE_ODR;
}

void TIMER3_ISR () iv IVT_INT_TIM3 {
     TIM3_SR.UIF = 0;                          // Reset UIF flag so next interrupt can be recognized when UIF is set
       adcVal = getAdcReading();
                GPIOD_ODR = adcVal;
                delay_ms(100);                        // Delay between readings.
}

void TIMER2_ISR() iv IVT_INT_TIM2 ics ICS_AUTO {

    TIM3_SR.UIF = 0;                          // Reset UIF flag so next interrupt can be recognized when UIF is set
      // Checking timer status register to perform an action (p570)
                while(!TIM1_SR.UIF){}          // Wait until timer update flag is set, meaning the count val was reached
                if (TIM1_SR.UIF == 1) { // If you have multiple timers, can use IF to check status register
                        TIM1_SR.UIF = 0;                  // Clear this flag which will reset the timer
                        GPIOD_ODR= ~GPIOD_ODR;  // Toggle LEDs
                        GPIOE_ODR= ~GPIOE_ODR;


          if(((USART1_SR & (1 << 5)) == 0x20))
          {
               rcvrd = USART1_DR;                    //read data from receiver data register
                                                     //if i receive the letter P
               if(rcvrd == 0x50 | rcvrd == 0x70) {        //The letter p or P was received
                                                          //Toggle game paused
                  while(USART1_SR.TC == 0){}  ;

                  USART1_DR = counter;


                }
        }
        counter++;
    }

}



//**************************************************************************************************
//GLOBAL VARIABLES

void PinConfiguration();    // Forward declaration of sub function used for pin configuration
void ExternalIntConfiguration(); // Forward declaration of sub function used for EXTI configuration
void Timer3Configuration();
void Timer2Configuration();
void InitializeUSART1();
void JoyStickConfiguration();

void AdcConfiguration(); // Function to configure ADC on PC0
unsigned int getAdcReading();        // Begins conversion and returns 12 bit value
//**************************************************************************************************
//MAIN FUNCTION
void main () {
        PinConfiguration();
        ExternalIntConfiguration();
        Timer3Configuration();
        Timer2Configuration();
        InitializeUSART1();
        JoyStickConfiguration();
         AdcConfiguration();
        PinConfiguration();


        GPIOD_ODR=0XFF00;                // Initialize GPIOD LEDS as off
        GPIOD_CRL = 0x33333333;
        GPIOD_CRH = 0x33333333;         // Configure GPIOD as output for LEDs
        GPIOD_ODR = 0;                  // Initialize all LEDs as OFF
        GPIOE_CRH = 0x33333333;
        GPIOC_CRH = 0x33333333;
        GPIOB_CRH = 0x33333333;
        GPIOE_ODR=0;                // Initialize GPIOE LEDs as on

        // You can write your main program from this point on. In this example, the program will
        // execute in an endless loop to keep the program active
        for(;;){

        }
}
//**************************************************************************************************
//SUB FUNCTIONS
void PinConfiguration() {
        GPIO_Digital_Output(&GPIOD_BASE, _GPIO_PINMASK_ALL);
        GPIO_Digital_Output(&GPIOE_BASE, _GPIO_PINMASK_ALL);
        GPIO_Digital_Input(&GPIOA_BASE, _GPIO_PINMASK_0);
        GPIO_Digital_Input(&GPIOB_BASE, _GPIO_PINMASK_6);


        GPIOD_CRL = 0x33333333;
        GPIOD_CRH = 0x33333333;         // Configure GPIOD as output for LEDs
        GPIOD_ODR = 0;                  // Initialize all LEDs as OFF
}

void ExternalIntConfiguration(){
        RCC_APB2ENR.AFIOEN = 1;     // Enable clock for alternate pin function
        AFIO_EXTICR1  = 0x00000000; // PA0 as External interrupt
        AFIO_EXTICR2 |= 0x00000100; // PB6 as External interrupt
        EXTI_RTSR |= 0x00000041;    // Set interrupt on rising edge for PA0 and PB6
        EXTI_IMR |= 0x00000041;     // Interrupt on PA0 and PB6 are non-maskable
        NVIC_ISER0 |= 1<<6;         // Enable NVIC interrupt for EXTI line zero (PA0)
                                // with position 6 in NVIC table
        NVIC_ISER0 |= 1<<23;        // Enable NVIC interrupt for EXTI9_% (PB6) position 23 in NVIC table
}

void Timer3Configuration(){
        RCC_APB1ENR |= (1 << 1);   // Enable TIMER1 clock. RCC: Clock Configuration Register
                                                                // Different clocks may use different registers.
                                                                // Ex. TIMER4 uses RCC_APB1ENR
        TIM3_CR1 = 0x0000;  // Disable timer until configuration is complete
                                                // If reset value of RCC_CFGR is used, then the 8MHz clock will
                                                // be the clock source for timer
        TIM3_PSC = 7999;    // Clock to TIMx_CNT = 72000000 (clock applied to prescaler register) /
                                            //                     7999 (Value in TIMx_PSC) + 1) = 9000
        TIM1_ARR = 9000;        // Reload timer count register with this value when count register resets
                                                // to 0 after counting from zero to this value

        NVIC_ISER0.B29 = 1;
        TIM3_DIER.UIE = 1;
        TIM3_CR1 = 0x0001;         // Enable TIMER1

// Notice: Bit 4 of TIM1_CR1 specifies whether the counter count up (BIT4=0) or counts down (BIT4=1)
// In this configuration this counting up is used.
}


void Timer2Configuration(){
        RCC_APB1ENR |= (1 << 1);   // Enable TIMER1 clock. RCC: Clock Configuration Register
                                                                // Different clocks may use different registers.
                                                                // Ex. TIMER4 uses RCC_APB1ENR
        TIM2_CR1 = 0x0000;  // Disable timer until configuration is complete
                                                // If reset value of RCC_CFGR is used, then the 8MHz clock will
                                                // be the clock source for timer
        TIM2_PSC = 7999;    // Clock to TIMx_CNT = 72000000 (clock applied to prescaler register) /
                                            //                     7999 (Value in TIMx_PSC) + 1) = 9000
        TIM2_ARR = 9000;        // Reload timer count register with this value when count register resets
                                                // to 0 after counting from zero to this value

        NVIC_ISER0.B29 = 1;
        TIM2_DIER.UIE = 1;
        TIM2_CR1 = 0x0001;         // Enable TIMER1

// Notice: Bit 4 of TIM1_CR1 specifies whether the counter count up (BIT4=0) or counts down (BIT4=1)
// In this configuration this counting up is used.
}

void InitializeUSART1(){ // Sub function which initializes the registers to enable USART1
        RCC_APB2ENR |= 1;                 // Enable clock for Alt. Function. USART1 uses AF for PA9/PA10
        AFIO_MAPR=0X0F000000;             // Do not mask PA9 and PA10 (becaue we are using for USART)
        RCC_APB2ENR |= 1<<2;              // Enable clock for GPIOA
        GPIOA_CRH &= ~(0xFF << 4);        // Clear PA9, PA10
        GPIOA_CRH |= (0x0B << 4);         // USART1 Tx (PA9) output push-pull
        GPIOA_CRH |= (0x04 << 8);         // USART1 Rx (PA10) input floating
        RCC_APB2ENR |= 1<<14;             // enable clock for USART1
        USART1_BRR=0X00000506;            // Set baud rate to 56000
        // Per data sheet (pg. 1010) USART1_CR1 consists of the following:
        //13 12   11  10  9    8     7    6      5      4  3  2   1   0
        //UE  M WAKE PCE PS PEIE TXEIE TCIE RXNEIE IDLEIE TE RE RWU SBK
        //rw rw  rw   rw rw   rw    rw   rw     rw     rw rw rw  rw  rw
        USART1_CR1 &= ~(1<<12);          // Force 8 data bits. M bit is set to 0.
        USART1_CR2 &= ~(3<<12);          // Force 1 stop bit
        USART1_CR3 &= ~(3<<8);           // Force no flow control and no DMA for USART1
        USART1_CR1 &= ~(3<<9);           // Force no parity and no parity control
        USART1_CR1 |= 3<<2;              // RX, TX enable
        //The following two instructions can also be used to enable RX and TX manually
        //USART1_CR1.TE=1; //TX enable
        //USART1_CR1.RE=1; //RX enable
        USART1_CR1 |= 1<<13;            // USART1 enable. This is done after configuration is complete
        Delay_ms(100);                  // Wait for USART to complete configuration and enable. This is
                                                                        // not always necessary, but good practice.
}


void JoyStickConfiguration(){
      //PA6 as input
      GPIOA_CRL = 0x4000000;      //PA6 as floating input
      RCC_APB2ENR.IOPAEN =1; //Enable port A clock


      GPIOB_CRL =  0x4000000;      //PB6 as floating input
      RCC_APB2ENR.IOPBEN =1; //Enable port B clock


      GPIOD_CRL = 0x40400;
      RCC_APB2ENR.IOPDEN =1; //Enable port D clock


      RCC_APB2ENR.AFIOEN =1;  //Enablr alternate function clock for interrupts
      AFIO_EXTICR1 = 0x0300;
      AFIO_EXTICR2 = 0x0013;  //Configure PA6,PB5,PD4 as interrupt
      EXTI_FTSR = 0x0074; //Enablr falling edge trigger
      //EXTI_RTSR  = 0x0074; //Enablr for falling edge
      EXTI_IMR = 0x0074; //Enable interrupt unmasking

      NVIC_ISER0.B8 =1;
      NVIC_ISER0.B10 =1;
      NVIC_ISER0.B23 =1;

}



void AdcConfiguration(){                   // ADC for PC0
        RCC_APB2ENR |= 1 << 4;                 // Enable PORTC clock
        RCC_APB2ENR |= 1 << 9 ;     // Enable ADC1 Clock
        GPIOC_CRL &= ~(0xF << 0);        // Configure PC0 as an Analog Input
        ADC1_SQR1 = (0b0000 << 20);        // 1 conversion
        ADC1_SQR3 = 10;                                // Select Channel 10 as only one in conversion sequence
        ADC1_SMPR1 = 0b100;                        // Set sample time on channel 10
        ADC1_CR2 |= (0b111 << 17);         // Set software start as external event for regular group conversion
        ADC1_CR2.ADON = 1;                        // Enable ADC1
        delay_ms(10);
}

unsigned int getAdcReading(){
        // Bit 20 is set to start conversion of an external channel, bit 22 starts the conversion
        ADC1_CR2 |= (1 << 22) | (1 << 20);
        while(!(ADC1_SR & 0b10));         // Wait until the ADC conversion has ended
        return ADC1_DR;                                // Read value from data register. This also clears start bit
}