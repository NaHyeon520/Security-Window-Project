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
uint16_t rain;//빗방울센서 값
uint16_t gas;//연기센서 값
uint16_t vib;//진동센서 값 
uint16_t light;//조도센서 값
int reserveFlag = 0;//예약모드 설정여부 표시
int reserveTime = 0;//예약모드를 설정한 시간
int modTime = 0;//예약모드 1, 2, 3 각각 10초, 20초, 30초
int timeCount = 0;//예약모드 시작후 1초씩 카운트

uint8_t temparature;//온습도센서 온도값
uint8_t humidity;//온습도센서 습도값

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

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PC0에 빗방울 센서 A0 연결(채널 10) 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //아날로그 값을 읽어오기 위해 모드를 GPIO_Mode_AIN로 설정
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//PC1에 연기 센서 A0 연결(채널 11) 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //아날로그 값을 읽어오기 위해 모드를 GPIO_Mode_AIN로 설정
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//PC2에 진동 센서 A0 연결(채널 12) 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //아날로그 값을 읽어오기 위해 모드를 GPIO_Mode_AIN로 설정
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PC3에 조도센서 연결(채널 13) 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //아날로그 값을 읽어오기 위해 모드를 GPIO_Mode_AIN로 설정
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PD0에 마그네틱 센서 연결(움직이는쪽에 달림)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //input
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;//마그네틱 센서2 연결(고정된쪽에 달림)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //input
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//PD5에 부저 연결
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

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//테스트용 led
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //output
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//테스트용 led
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
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_7Cycles5); //빗방울센서
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_7Cycles5); //연기센서
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_7Cycles5); //진동센서
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_7Cycles5); //조도센서

    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);//interrupt enable
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1); //calibration (reduce error value ) reset
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1); //calibration start
    while (ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}

void speaker_on() {//스피커 on
    GPIO_SetBits(GPIOD, GPIO_Pin_5);
}
void speaker_off() {//스피커 off
    GPIO_ResetBits(GPIOD, GPIO_Pin_5);
}
void fan_on() {//환풍기 on
    GPIO_SetBits(GPIOC, GPIO_Pin_10);//IN3 high
    GPIO_ResetBits(GPIOC, GPIO_Pin_11);//IN4 low
    //fan_flag=1;
}

void fan_off() {//환풍기 off
    GPIO_ResetBits(GPIOC, GPIO_Pin_10);//IN3 low
    //fan_flag=0;
}

int magnetic() {//마그네틱 센서. 문개폐 여부
    if (~GPIOD->IDR & GPIO_Pin_0) {//문이 닫혔으면
        return 1;
    }
    else//문이 열려있으면 
        return 0;
}

int magnetic2() {//마그네틱 센서2
    if (~GPIOD->IDR & GPIO_Pin_12) {//문 반대쪽이 닫혔으면
        return 1;
    }
    else {//문 반대쪽이 열려있으면
        return 0;
    }
}

void open() {//문여는 함수
    while (magnetic2() == 0) {//문 반대쪽이 열려있는 동안
      //모터 동작
        GPIO_SetBits(GPIOC, GPIO_Pin_8);//IN1 high
        GPIO_ResetBits(GPIOC, GPIO_Pin_9);//IN2 low
    }
    //모터 off
    GPIO_ResetBits(GPIOC, GPIO_Pin_8);//IN1 low
    GPIO_ResetBits(GPIOD, GPIO_Pin_5);//buzzer off-> 연기 감지시 speaker가 작동하고 문이 닫히는데, 문이 닫히고 연기 센서 값이 충분히 떨어지고 난 후 멈추도록 함
}

void close() {//문닫는 함수 
    while (magnetic() == 0) {//문이 열려있는동안
      //모터 동작
        GPIO_ResetBits(GPIOC, GPIO_Pin_8);//IN1 low
        GPIO_SetBits(GPIOC, GPIO_Pin_9);//IN2 high
    }
    //모터 off
    GPIO_SetBits(GPIOC, GPIO_Pin_8);//IN1 high
}


void on() {//test용 led on
    GPIO_SetBits(GPIOD, GPIO_Pin_2);
}

void ADC1_2_IRQHandler(void) {
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) { //ADC_IT_EOC(END OF CONVERTION)이 SET된 상태이면 value값에 저장

        rain = ADC_GetConversionValue(ADC1);
        gas = ADC_GetConversionValue(ADC1);
        vib = ADC_GetConversionValue(ADC1);
        light = ADC_GetConversionValue(ADC1);

        if (rain > 1000 && rain <= 2300) {//비 내리면
            if (magnetic() == 0) {//문 열렸는지 확인(마그네틱 센서)
                close();//문 열렸으면 닫기
            }
        }
        if (gas <= 1200 && gas >= 900) {//가스 감지하면
            if (temparature > 10) {//60도 이상이면 문열고 스피커 동작//시연을 위해 10도로 조정했음
                speaker_on();//경보음 울림
                if (magnetic() == 1)//문 닫혀있으면
                    open();//문 열기
            }
        }
        if (vib <= 50) {//진동 감지하면
            speaker_on();//경보음 울림
            comtophone();//블루투스로 알림 전송
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

char warningWord[1] = "!";//경고문
void comtophone(void) {//충격 감지 경고문 전송 
       //bluetooth 모듈로 전송하여 핸드폰화면에 출력  
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
        word = USART_ReceiveData(USART2); //핸드폰에서 입력한 문자가 bluetooth 모듈을 통해 보드로 전송

        if (word == 'o') {//o입력시 문 열림
            open();//문 열림
        }
        else if (word == 'c') {//c입력시 문 닫힘
            close();//문 닫힘
        }
        if (word == '1') {//예약모드1
          //타이머 10초마다 환풍기, 문개폐
            reserveFlag = 1;//예약모드 설정됨
            reserveTime = 60;//예약모드 동작 시간 60초
            modTime = 10;//10초마다 개폐
        }
        if (word == '2') {//예약모드2
            reserveFlag = 1;//예약모드 설정됨
            reserveTime = 60;//예약모드 동작시간 60초
            modTime = 20;//20초마다 개폐
        }
        if (word == '3') {//예약모드3
            reserveFlag = 1;//예약모드 설정됨
            reserveTime = 60;//예약모드 동작시간 60초
            modTime = 30;//30초마다 개폐
        }
        if (word == 'b') {//buzzer off
            speaker_off();//스피커 끄기
        }

    }
}

void TIM3_IRQHandler(void) {
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        if (reserveFlag == 1) { //예약모드가 설정되면 
            timeCount++; //예약모드 시간 카운트
        }
    }
}

void TIM_Configure(void) {

    TIM_TimeBaseInitTypeDef TIM3_TimeBaseStructure; //1초 timer->예약모드 카운트용
    TIM_TimeBaseInitTypeDef TIM2_TimBaseStructure; //온습도센서 읽기용

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


void TemHum_read() {//온습도센서 읽는 함수

    int i, j, temp;
    uint8_t data[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    GPIO_InitTypeDef GPIO_InitStructure;

    // 온습도 센서 핀 초기화
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // DHT 센서는 초기 18ms 동안 입력을 보류해야 함
    // 18ms를 기다림
    GPIO_ResetBits(GPIOB, GPIO_Pin_6);
    TIM2->CNT = 0;
    while ((TIM2->CNT) <= 18000);

    // 이후 20-40us동안 HIGH 입력을 줘야 함
  // 40us를 기다린다, 초기화 끝
    GPIO_SetBits(GPIOB, GPIO_Pin_6);
    TIM2->CNT = 0;
    while ((TIM2->CNT) <= 40);

    // Data를 받아오기 위해 Input 모드로 설정한다
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // DHT11의 ACK의 Timeout을 count한다
    // DHT11의 신호는 적어도 80us동안 유지되어야 함
    //timeout발생시 return
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

    // 40비트(8*5)를 읽어온다
    for (j = 0; j < 5; ++j) {
        for (i = 0; i < 8; ++i) {

            // 50us동안 LOW를 유지한다
            while (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6));

            // Timer Count 시작
            TIM_SetCounter(TIM2, 0);

            // 26-28us동안 HiGH 신호가 들어오면 0으로, 70us동안 HIGH 신호가 들어오면 1로 취급한다
            while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6));


            // 현재 시점까지 지난 시간을 받아온다
            temp = TIM_GetCounter(TIM2);

            //shift 0
            data[j] = data[j] << 1;

            // 40us보다 값이 크면(26-28us가 아니면)  해당 배열의 값은 1이 된다
            if (temp > 40)
                data[j] = data[j] + 1;
        }
    }

    // 온도값은 data[2]에, 습도값은 data[0]에 저장된다
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

        if (reserveFlag == 1) {//예약모드가 설정되었을 때
            if (timeCount % modTime == 0 && magnetic() == 1) { //예약시간이 되었고 문이 닫혀있으면
                open(); //문을 연다
            }
            else if (timeCount % modTime == 0 && magnetic() == 0) { //예약시간이 되었고 문이 열려있으면
                close(); //문을 닫는다.
            }
            if (timeCount > reserveTime) {//예약시간이 넘어가면
                reserveFlag = 0; //flag reset
                timeCount = 0; //시간 초기화
            }
        }

        printf("rain= %d, gas=%d, vib=%d, light=%d\n", rain, gas, vib, light);

        TemHum_read();

        //printf("TEMPRATURE %d - HUMIDITY %d\n", temparature, humidity);
        LCD_ShowString(85, 20, "Temperature", BLACK, WHITE);
        LCD_ShowNum(85, 40, temparature, 10, BLACK, WHITE);

        if (humidity < 40) {//습도 출력, 40% 이하이면 건조
            LCD_ShowString(85, 60, "Humidity: Dry   ", BLACK, WHITE);
        }
        else if (humidity < 60) {//40%~60%이면 적절
            LCD_ShowString(85, 60, "Humidity: Proper", BLACK, WHITE);
        }
        else {//60% 이상이면 습함
            LCD_ShowString(85, 60, "Humidity: Wet   ", BLACK, WHITE);
        }

        //조도센서값 밝음/어두움으로 출력
        if (light > 3000)
            LCD_ShowString(85, 80, "Light: Bright", BLACK, WHITE);
        else
            LCD_ShowString(85, 80, "Light: Dark  ", BLACK, WHITE);
    }
}
