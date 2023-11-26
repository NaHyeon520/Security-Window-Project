#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"
#include "lcd.h"
#include "touch.h"
#include "string.h"
#include "math.h"

int color[12] = { WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY };
uint16_t rain;//raindrop sensor value
uint16_t gas;//gas sensor value
uint16_t vib;//vibration sensor value
uint16_t light;//light sensor value
int reserveFlag = 0;//reserve mode on/off
int reserveTime = 0;//reserve mode time set
int modTime = 0;//reserve mode 1,2,3 -> 10sec, 20sec, 30sec each
int timeCount = 0;//after the reserve mode start, count every 1sec

uint8_t temparature;//temperature/humidity sensor temperature value
uint8_t humidity;//temperature/humidity sensor humidity value

void RCC_Configure(void);
void GPIO_Configure(void);
void USART1_Init(void);
void USART2_Init(void);
void ADC_Configure(void);
void speaker_on();
int magnetic();
int magnetic2();
void open();
void close();
void TemHum_read();
void comtophone(void);
void on(void);

void RCC_Configure(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //port C enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // ADC enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); //port D enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //portA enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);//USART1 clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);// USART2 clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //portB enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//Timer2 enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//Timer3 enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); //Alternative functions I/O enable
}

void GPIO_Configure(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PC0->raindrop sensor A0 connected(channel 10) 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //to read the analog value, change the mode to GPIO_Mode_AIN
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//PC1->gas sensor A0 connected(channel 11) 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //to read the analog value, change the mode to GPIO_Mode_AIN
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//PC2->vibration sensor A0 connected(channel 12) 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //to read the analog value, change the mode to GPIO_Mode_AIN
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PC3->light sensor connected(channel 13) 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //to read the analog value, change the mode to GPIO_Mode_AIN
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PD0->magnetic sensor connected(window-moving side)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //input
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;//magnetic sensor 2 connected(window-fixed side)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //input
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//PD5->buzzer
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //output
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    //모터드라이버  시작
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;//PC8->IN1
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //output
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//PC9->IN2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //output
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PC10->IN3
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //output
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PC11->IN4
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //output
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    //모터드라이버 끝

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//led for test
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //output
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//led for test
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //output
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* UART1 pin setting */
    //TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //Alternate function output push-pull
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PA10
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //input with pull down
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* UART2 pin setting */
    //TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //Alternate function output push-pull
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //PA3
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //input with pull down
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //온습도센서
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void USART1_Init(void)
{
    USART_InitTypeDef USART1_InitStructure;

    USART1_InitStructure.USART_BaudRate = 9600;
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART1_InitStructure.USART_Parity = USART_Parity_No;
    USART1_InitStructure.USART_StopBits = USART_StopBits_1;
    USART1_InitStructure.USART_WordLength = USART_WordLength_8b;

    // Enable the USART1 peripheral
    USART_Cmd(USART1, ENABLE);

    //Initialize the USART1
    USART_Init(USART1, &USART1_InitStructure);

    //Enable the USART1 RX interrupts
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void USART2_Init(void)
{
    USART_InitTypeDef USART2_InitStructure;

    USART2_InitStructure.USART_BaudRate = 9600;
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART2_InitStructure.USART_Parity = USART_Parity_No;
    USART2_InitStructure.USART_StopBits = USART_StopBits_1;
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;

    // Enable the USART2 peripheral
    USART_Cmd(USART2, ENABLE);

    //Initialize the USART2
    USART_Init(USART2, &USART2_InitStructure);

    //Enable the USART2 RX interrupts
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void ADC_Configure(void) {
    ADC_InitTypeDef ADC_InitStructure;

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //MODE 
    ADC_InitStructure.ADC_ScanConvMode = ENABLE; //conversion scan enable
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // continuous conversion enable
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //externeral trigger
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //data alignment set
    ADC_InitStructure.ADC_NbrOfChannel = 4; //channel의 수

    ADC_Init(ADC1, &ADC_InitStructure);
    // gpio port와 pin에 맵핑된 channel 할당 
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_7Cycles5); //raindrop sensor
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_7Cycles5); //gas sensor
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_7Cycles5); //vibration sensor
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_7Cycles5); //light sensor

    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);//interrupt enable
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1); //calibration (reduce error value ) reset
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1); //calibration start
    while (ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}

void speaker_on() {//speaker on
    GPIO_SetBits(GPIOD, GPIO_Pin_5);
}
void speaker_off() {//speaker off
    GPIO_ResetBits(GPIOD, GPIO_Pin_5);
}
void fan_on() {//ventilator on
    GPIO_SetBits(GPIOC, GPIO_Pin_10);//IN3 high
    GPIO_ResetBits(GPIOC, GPIO_Pin_11);//IN4 low
    //fan_flag=1;
}

void fan_off() {//ventilator off
    GPIO_ResetBits(GPIOC, GPIO_Pin_10);//IN3 low
    //fan_flag=0;
}

int magnetic() {//magnetic sensor. tells whether the window is opened or not
    if (~GPIOD->IDR & GPIO_Pin_0) {//if the window is closed
        return 1;
    }
    else//opened
        return 0;
}

int magnetic2() {//magnetic sensor 2
    if (~GPIOD->IDR & GPIO_Pin_12) {//if the opposite side window is closed
        return 1;
    }
    else {//opened
        return 0;
    }
}

void open() {//opens the window 
    while (magnetic2() == 0) {//during the opposite window is opened
      //motor works 
        GPIO_SetBits(GPIOC, GPIO_Pin_8);//IN1 high
        GPIO_ResetBits(GPIOC, GPIO_Pin_9);//IN2 low
    }
    //motor off
    GPIO_ResetBits(GPIOC, GPIO_Pin_8);//IN1 low
    GPIO_ResetBits(GPIOD, GPIO_Pin_5);//buzzer off-> when the gas sensor detects any gas, buzzer works and the window closes. After the window is closed, buzzer stops when the detected gas value has fallen enough.
}

void close() {//closes the window
    while (magnetic() == 0) {//while the window is open
      //motor works
        GPIO_ResetBits(GPIOC, GPIO_Pin_8);//IN1 low
        GPIO_SetBits(GPIOC, GPIO_Pin_9);//IN2 high
    }
    //motor off
    GPIO_SetBits(GPIOC, GPIO_Pin_8);//IN1 high
}


void on() {//led for test on
    GPIO_SetBits(GPIOD, GPIO_Pin_2);
}

void ADC1_2_IRQHandler(void) {
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) { //if ADC_IT_EOC(END OF CONVERTION) is SET-> store value

        rain = ADC_GetConversionValue(ADC1);
        gas = ADC_GetConversionValue(ADC1);
        vib = ADC_GetConversionValue(ADC1);
        light = ADC_GetConversionValue(ADC1);

        if (rain > 1000 && rain <= 2300) {//if it detects raining
            if (magnetic() == 0) {//check if the window is open(magnetic sensor)
                close();//close if it is open
            }
        }
        if (gas <= 1200 && gas >= 900) {//if it detects the gas
            if (temparature > 60) {//if temperature is over 60 celsius, open the window and buzzer works(because it recognizes as a fire)
                speaker_on();//buzzer on
                if (magnetic() == 1)//if the window is closed
                    open();//open the window
            }
        }
        if (vib <= 50) {//if it detects vibration
            speaker_on();//buzzer on
            comtophone();//bluetooth warning is sent 
        }
    }
    ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
}

void sendDataUART1(uint16_t data) {
    USART_SendData(USART1, data);
}

void sendDataUART2(uint16_t data) {
    USART_SendData(USART2, data);
}

char warningWord[1] = "!";//warning message
void comtophone(void) {//send the warning message(detection of crash) 
       //transmit to bluetooth module, print at mobile phone screen  
    for (int i = 0; i < 1; i++) {
        sendDataUART2(warningWord[i]);
    }
}


void USART1_IRQHandler(void) {
    uint16_t word;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        // the most recent received data by the USART1 peripheral
        word = USART_ReceiveData(USART1);
        sendDataUART2(word);
        // clear 'Read data register not empty' flag
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}


void USART2_IRQHandler(void) {
    uint16_t word;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        // clear 'Read data register not empty' flag
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
        word = USART_ReceiveData(USART2); //input from phone is transmitted to the board through bluetooth module

        if (word == 'o') {//type "o" ->open the window
            open();//open the window
        }
        else if (word == 'c') {//type "c" ->close the window
            close();//close the window
        }
        if (word == '1') {//reserved mode 1
          //every 10sec, turn on and off the ventilator and open and close the window
            reserveFlag = 1;//reserve mode is on
            reserveTime = 60;//for 60 sec,
            modTime = 10;//open/close every 10sec
        }
        if (word == '2') {//reserved mode 2
            reserveFlag = 1;//reserve mode is on
            reserveTime = 60;//for 60 sec,
            modTime = 20;//open/close every 20sec
        }
        if (word == '3') {//reserved mode 3
            reserveFlag = 1;//reserve mode is on
            reserveTime = 60;//for 60 sec,
            modTime = 30;//open/close every 30sec
        }
        if (word == 'b') {//buzzer off
            speaker_off();
        }

    }
}

void TIM3_IRQHandler(void) {
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        if (reserveFlag == 1) { //if reserve mode is on
            timeCount++; //count the reserve mode time
        }
    }
}

void TIM_Configure(void) {

    TIM_TimeBaseInitTypeDef TIM3_TimeBaseStructure; //1sec timer->for the count of reserve mode
    TIM_TimeBaseInitTypeDef TIM2_TimBaseStructure; //read temperature/humidity sensor

    TIM3_TimeBaseStructure.TIM_Period = 10000;
    TIM3_TimeBaseStructure.TIM_Prescaler = 7200;
    //1/72MHz * 7200 * 10000 = 1s
    TIM3_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM3_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM3_TimeBaseStructure);
    TIM_Cmd(TIM3, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); //interrupt enable

    TIM2_TimBaseStructure.TIM_Period = 84000000 - 1;
    TIM2_TimBaseStructure.TIM_Prescaler = 84;
    TIM2_TimBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM2_TimBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM2_TimBaseStructure);
    TIM_Cmd(TIM2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void NVIC_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // UART1
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // UART2
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_EnableIRQ(ADC1_2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


void TemHum_read() {//read temperature/humidity sensor

    int i, j, temp;
    uint8_t data[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    GPIO_InitTypeDef GPIO_InitStructure;

    // initialization of temperature/humidity sensor pin
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // DHT sensor:don't get input during initial 18ms
    // wait fot 18ms
    GPIO_ResetBits(GPIOB, GPIO_Pin_6);
    TIM2->CNT = 0;
    while ((TIM2->CNT) <= 18000);

    // give HIGH input for 20-40us
  // wait for 40us, initialization done
    GPIO_SetBits(GPIOB, GPIO_Pin_6);
    TIM2->CNT = 0;
    while ((TIM2->CNT) <= 40);

    // to get the data, set input mode
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // count the DHT11 ACK's Timeout
    // signal of DHT11 should be maintained during 80us
    //return if timeout occurs
    TIM2->CNT = 0;
    while (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6)) {
        if (TIM2->CNT > 100)
            return;
    }

    TIM2->CNT = 0;
    while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6)) {
        if (TIM2->CNT > 100)
            return;
    }

    // read 40 bits(8*5)
    for (j = 0; j < 5; ++j) {
        for (i = 0; i < 8; ++i) {

            // maintain LOW during 50us
            while (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6));

            // start Timer Count
            TIM_SetCounter(TIM2, 0);

            // if HIGH signal comes for 26-28us-> 0, for 70us-> 1
            while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6));


            // get the time that has passed until now
            temp = TIM_GetCounter(TIM2);

            //shift 0
            data[j] = data[j] << 1;

            // if the value is larger than 40us(not 26-28us) the array element value is 1
            if (temp > 40)
                data[j] = data[j] + 1;
        }
    }

    // temperature value is stored in data[2], humidity value is stored in data[0]
    temparature = data[2];
    humidity = data[0];

    return;
}

int main() {
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    ADC_Configure();
    USART1_Init();
    USART2_Init();
    NVIC_Configure();
    TIM_Configure();
    LCD_Init();

    while (1) {

        if (reserveFlag == 1) {//if reserve mode is set
            if (timeCount % modTime == 0 && magnetic() == 1) { //if it's reserved time and the window is closed
                open(); //open the window
            }
            else if (timeCount % modTime == 0 && magnetic() == 0) { //if it's reserved time and the window is opened
                close(); //close the window
            }
            if (timeCount > reserveTime) {//after the reserved time
                reserveFlag = 0; //flag reset
                timeCount = 0; //initialize time
            }
        }

        printf("rain= %d, gas=%d, vib=%d, light=%d\n", rain, gas, vib, light);

        TemHum_read();

        //printf("TEMPRATURE %d - HUMIDITY %d\n", temparature, humidity);
        LCD_ShowString(85, 20, "Temperature", BLACK, WHITE);
        LCD_ShowNum(85, 40, temparature, 10, BLACK, WHITE);

        if (humidity < 40) {//print humidity, dry if under 40%
            LCD_ShowString(85, 60, "Humidity: Dry   ", BLACK, WHITE);
        }
        else if (humidity < 60) {//40%~60%
            LCD_ShowString(85, 60, "Humidity: Proper", BLACK, WHITE);
        }
        else {//over 60%
            LCD_ShowString(85, 60, "Humidity: Wet   ", BLACK, WHITE);
        }

        //print light sensor value as bright/dark
        if (light > 3000)
            LCD_ShowString(85, 80, "Light: Bright", BLACK, WHITE);
        else
            LCD_ShowString(85, 80, "Light: Dark  ", BLACK, WHITE);
    }
}
